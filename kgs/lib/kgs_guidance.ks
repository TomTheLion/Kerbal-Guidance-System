@lazyglobal off.

//
// guidance algorithm functions
//

// set initial guidance
global guidance_command is list(lookdirup(ship:up:vector, ship:facing:topvector), 1).
lock steering to guidance_command[0].
lock throttle to guidance_command[1].

// main guidance function, guidance begins in open loop mode where ship attitude and throttle are functions of time
// based on the specified inputs, after the timed event "activate guidance" closed loop guidance is initialized and the
// guidance equations are continually solved to update the current guidance solution, when time to go is less than ten
// seconds terminal guidance starts where the last guidance solution is used until time to go reaches zero
function guidance {

	if not kgs_data:clg:active_guidance {

		// open loop guidance

		// set mission elapsed time and roll and pitch flags
		local met is time:seconds - kgs_data:olg:launch_time.
		local roll_flag is met > kgs_data:olg:roll_time.
		local pitch_flag is met > kgs_data:olg:pitch_time.

		// after pitch time, pitch is increased at the pitch rate (default 1 deg/sec) until specified angle of attack has been
		// reached, the angle of attack is held for the specified duration, then pitch is decreased at the pitch rate until it
		// reaches zero, before roll or pitch time roll is held constant, after roll or pitch time roll is set to specified value
		// as an offset from launch azimuth
		if pitch_flag {
			local aoa is min(
				kgs_data:olg:pitch_rate * (met - kgs_data:olg:pitch_time),
				kgs_data:olg:pitch_aoa - kgs_data:olg:pitch_rate * (met - kgs_data:olg:pitch_time - kgs_data:olg:pitch_duration)
			).
			set aoa to max(0, min(aoa, kgs_data:olg:pitch_aoa)).
			local look_at is heading(kgs_data:olg:launch_azimuth, 90 - vang(ship:up:vector, ship:velocity:surface) - aoa):vector.
			local look_up is ship:up:vector * angleaxis(kgs_data:olg:roll_angle, look_at).
			set guidance_command to list(lookdirup(look_at, look_up), kgs_data:olg:throttle).
		}
		else {
			local look_up is ship:facing:topvector.
			if roll_flag {
				set look_up to heading(kgs_data:olg:roll_angle + kgs_data:olg:launch_azimuth + 180, 0, 0):vector.
			}
			set guidance_command to list(lookdirup(ship:up:vector, look_up), kgs_data:olg:throttle).
		}
	}
	else {

		// closed loop guidance

		// after guidance is initializes ship attitude and throttle are updated by the guidance trigger based on the current
		// guidance solution, unless the vehicle is staging the state and guidance solution are continually updated until time
		// to go reaches ten seconds, time warp rates are reduced towards the end of the trajectory
		if not kgs_data:clg:staging and not kgs_data:clg:terminal_guidance {

			local tgo is get_tgo().
			update_state().
			update_guidance().

			until kuniverse:timewarp:rate = 1 or tgo > 10 * kuniverse:timewarp:rate + 5 {
				set kuniverse:timewarp:rate to kuniverse:timewarp:rate - 1.
			}

			if tgo < 10 {
				set kgs_data:clg:time_prev to time:seconds.
				set kgs_data:clg:time_final to kgs_data:clg:time_guide + kgs_data:clg:x_guide[6] * kgs_data:scale:time.
				set kgs_data:clg:terminal_guidance to true.
			}
		} 
		else if kgs_data:clg:terminal_guidance {

			// terminal guidance

			// attitude and throttle are updated based on the last guidance solution, when the difference between the final time
			// and the current time is less than the amount of time required to execute one guidance loop the throttle is locked to
			// zero and guidance is stopped
			local tgo is kgs_data:clg:time_final - (time:seconds + (time:seconds - kgs_data:clg:time_prev)).
			set kgs_data:clg:time_prev to time:seconds.
			set guidance_command to list(get_attitude(), get_throttle()).

			if tgo < 0 {
				lock throttle to 0.
				set kgs_data:clg:stop_guidance to true.
				refresh_ui().
			}
			else if tgo < 2 * kgs_settings:ui_refresh_rate {
				set kgs_data:clg:stop_ui to true.
			}
		}
	}
}

// adds closed loop guidance events, generates initial guess for guidance solution, and activates closed loop guidance
function initialize_closed_loop_guidance {
	
	// set state variables
	local t is time:seconds.
	local met is t - kgs_data:olg:launch_time.
	local r_ship is swap_yz(-ship:orbit:body:position) / kgs_data:scale:distance.
    local v_ship is swap_yz(ship:orbit:velocity:surface) / kgs_data:scale:velocity.
	local m_ship is ship:mass * 1000.
		
	// velocity to go is estimated based on the current velocity and the orbital velocity for a circular orbit (v = 1), the
	// difference is increased by 20% to account for inefficiencies, time to go is then estimated as the amount of time
	// required for the ship to generate an amount of delta v equal to the estimated velocity to go
	local vgo is 1.2 * (1 - v_ship:mag).
	local tgo is time_to_delta_v(kgs_data:stages, vgo, m_ship).
	
	// each closed loop guidance event needs to be added to the stage list and will increase the number of integration
	// steps since they cause discontinuities in the mass and acceleration
	for event in kgs_data:clg:events {

		// the time to reach the end of each stages is calculated until it exceeds the time of the event to be added, then
		// for a jettison event the stage is split into two stages and for a coast event the stage is split into three
		// stages
		local stage_ptr is kgs_data:stages:iterator.
		local delta_time_total is 0.
		local delta_time_prev is 0.
		local m is m_ship.
	
		until not stage_ptr:next {

			local s is stage_ptr:value.
				
			// update mass for current stage
			if stage_ptr:index <> 0 {
				set m to s:mass_total.
			}
			
			// calculate time at end of stage
			set delta_time_total to delta_time_total + stage_burn_time(s, met + delta_time_total, m) * kgs_data:scale:time.

			// if event occurs this stage, add event
			if event:time < met + delta_time_total {
			
				// calculate mass when staging occures
				local delta_time is event:time - (met + delta_time_prev).
				local m_event is m.
				if s:mode = "c_thrust" {
					set m_event to m -  s:thrust / s:vex * delta_time / kgs_data:scale:time.
				}
				else if s:mode = "c_accel" {	
					local dv is delta_time / kgs_data:scale:time * s:a_limit.
					set m_event to m / constant:e ^ (dv / s:vex).
				}
				
				// add event based on type
				if event:type = "jettison" {
				
					// for a jettison event the stage will be split into two parts, the first part is identical to the
					// original stage except for its ending condition and staging sequence, the second part is also
					// identical except for its initial and final mass

					// set first stage part
					local stage_data is s:copy.
					set stage_data:staging_sequence to list(0).
					set stage_data:mass_dry to m_event.
					if s:mode = "coast" {
						set stage_data:time_end to event:time.
					}
					
					// set second stage part
					set s:event_name to event:description.
					set s:mass_total to m_event - event:mass.
					set s:mass_dry to s:mass_dry - event:mass.
					
					// remove jettisoned mass from following stages
					for i in range(stage_ptr:index + 1, kgs_data:stages:length) {
						if not event:haskey(additional_stages) or kgs_data:stages[i]:index <= s:index + event:additional_stages {
							set kgs_data["stages"][i]["mass_total"] to kgs_data["stages"][i]["mass_total"] - event:mass.
							set kgs_data["stages"][i]["mass_dry"] to kgs_data["stages"][i]["mass_dry"] - event:mass.
						}
					}
					
					// add stage
					kgs_data:stages:insert(stage_ptr:index, stage_data).
				}
				else if event:type = "coast" {
				
					// for coast event the stage will be split into three parts, the first will be identical to the original stage except
					// for its ending condition, the second will be the coast stage, and the third will be identical to the original except
					// for its initial and final mass
					local stage_pre is s:copy.
					local stage_coast is s:copy.
					
					// the event name for the original stage should only be applied to the first part
					if s:haskey("event_name") {
						s:remove("event_name").
					}
					set stage_coast:event_name to event:description.
					
					// set coast stage name and mode
					if stage_coast:name:contains(" (C-Accel)") {
						set stage_coast:name to stage_coast:name:substring(0, stage_coast:name:find(" (C-Accel)")).
					}	
					set stage_coast:name to stage_coast:name + " (Coast)".
					set stage_coast:mode to "coast".
					
					// the staging sequence should only be on the third part
					stage_pre:remove("staging_sequence").
					stage_coast:remove("staging_sequence").
				
					// the first part ends at the event time with the event mass
					set stage_pre:time_end to event:time.
					set stage_pre:mass_dry to m_event.
					
					// during a coast stage the mass does not change and it ends after the event duration
					set stage_coast:mass_total to m_event.
					set stage_coast:mass_dry to m_event.
					set stage_coast:time_end to event:time + event:duration.
					set stage_coast:throttle to 0.
					set stage_coast:thrust to 0.	
					
					// set mass total for third part
					set s:mass_total to m_event.
					
					// add stages
					kgs_data:stages:insert(stage_ptr:index, stage_coast).
					kgs_data:stages:insert(stage_ptr:index, stage_pre).
					
					// increase time to go estimate by coast stage duration
					set tgo to tgo + event:duration / kgs_data:scale:time.
				}
				
				break.
			}
			else {
				// if the event did not occur on this stage, add its burn time plus any staging time to the total time
				if s:haskey("staging_sequence") {
					for val in s:staging_sequence {
						set delta_time_total to delta_time_total + val.
					}
				}
				set delta_time_prev to delta_time_total.
			}
		}	
	}
	
	// the initial guess for the guidance solution will be an approximately prograde trajectory assuming a constant
	// acceleration of 1g aligned with the velocity vector
	local lambda is v_ship:normalized.
	local lambda_dot is (v_ship:normalized - r_ship:normalized).

	// set current guidance solution to the initial guess, if initial guidance is later loaded the ship will fly the
	// approximately prograde trajectory until it converges
	set kgs_data:clg:x_guide to vector_to_list(list(lambda, lambda_dot, tgo)).
	
	// load initial guidance if available
	if kgs_inputs:objective:haskey("load_initial_guidance") {
		local out is load_initial_guidance().
		set lambda to out[0].
		set lambda_dot to out[1].
		set tgo to out[2].
	}

	// set initial guess values and guidance times
	set kgs_data:clg:x to vector_to_list(list(lambda, lambda_dot, tgo)).
    set kgs_data:clg:time to t.
	set kgs_data:clg:time_guide to t.
    set kgs_data:clg:time_prev to t.

	// set stage and guidance trigger
	if kgs_data:stages:length > 1 {
		set_stage_trigger().
	}
	else {
		set kgs_data:ui:clg_event_time to 0.
		set kgs_data:ui:clg_event_name to "".
	}
	
	set_guidance_trigger().

	// set active guidance flag
	set kgs_data:clg:active_guidance to true.
}

// calculates time required for vehicle to produce given delta v
function time_to_delta_v {

	parameter stages.	// stages of the vehicle
	parameter vgo.		// velocity to go
	parameter m_ship.	// mass of the ship
	
	// calculate the delta v and burn time of each stage, subtract delta v from vgo and add burn time to total until vgo is
	// exceeded, for final stage only add the required part of the stage burn time to reach vgo
	local tgo is 0.
	local stage_ptr is stages:iterator.
	until not stage_ptr:next {

		local s is stage_ptr:value.
			
		// update mass for current stage
		if stage_ptr:index <> 0 {
			set m_ship to s:mass_total.
		}
		
		// calculate stage final mass, delta velocity, mass rate
		local mf is s:mass_dry.
		local dv is ln(m_ship / mf) * s:vex.
		local mdot is s:thrust / s:vex.
		
		// if stage has insufficient delta v add burn time to tgo, otherwise add required burn time and break
		if vgo < dv {
		    if s:mode = "c_accel" {
                set tgo to tgo + vgo / s:a_limit.
            }
            else {
                set tgo to tgo + m_ship / mdot * (1 - constant:e ^ (-vgo / s:vex)).
            }
			break.
		}
		else {
			set vgo to vgo - dv.
			
		    if s:mode = "c_accel" {
                set tgo to tgo + dv / s:a_limit.
            }
            else {        
                set tgo to tgo + (m_ship - mf) / mdot.
            }
		}
	}
	
	return tgo.
}

// saves initial guidance, initial guidance is coded as magnitude of lambda/lambda_dot, their angles from the local
// vertical direction, and initial time to go
function save_initial_guidance {
	local r_ship is kgs_data:clg:y[1].
	local lambda is list_to_vector(kgs_data:clg:x_guide, 0).
	local lambda_dot is list_to_vector(kgs_data:clg:x_guide, 3).
	
	local out is list().
	out:add(lambda:mag).
	out:add(vang(r_ship, lambda)).	
	out:add(lambda_dot:mag).
	out:add(vang(r_ship, lambda_dot)).
	out:add(kgs_data:clg:x_guide[6]).
	
	writejson(out, kgs_inputs:objective:save_initial_guidance).
}

// load initial guidance
function load_initial_guidance {

	local inp is readjson(kgs_inputs:objective:load_initial_guidance).
	
	local r_ship is swap_yz(-ship:orbit:body:position).
    local v_ship is swap_yz(ship:orbit:velocity:surface).
	local n is vcrs(r_ship, v_ship):normalized.
	
	local lambda is inp[0] * r_ship:normalized * angleaxis(inp[1], n).
	local lambda_dot is inp[2] * r_ship:normalized * angleaxis(inp[3], n).
	
	return list(lambda, lambda_dot, inp[4]).
}

// sets guidance trigger based on guidance refresh rate until terminal guidance
function set_guidance_trigger {
	local t0 is time:seconds.
	when time:seconds > t0 + kgs_settings:guidance_refresh_rate then {
		if not kgs_data:clg:terminal_guidance {
			set guidance_command to list(get_attitude(), get_throttle()).
			set_guidance_trigger().
		}
	}
}

// updates state in data lexicon, calculates desired node vector if lan is specified
function update_state {

	if kgs_data:clg:mode = 2 {
		set kgs_data:clg:node_d to swap_yz(solarprimevector * angleaxis(-kgs_inputs:objective:lan, v(0, 1, 0))):normalized.
	}
	
    set kgs_data:clg:time_prev to kgs_data:clg:time.
    set kgs_data:clg:time to time:seconds.
    set kgs_data:clg:y to deep_copy(kgs_data:clg:yi).
    set kgs_data["clg"]["y"][0] to ship:mass * 1000.
    set kgs_data["clg"]["y"][1] to swap_yz(-ship:orbit:body:position / kgs_data:scale:distance).
    set kgs_data["clg"]["y"][2] to swap_yz(ship:orbit:velocity:orbit / kgs_data:scale:velocity).
	set kgs_data:clg:stages to deep_copy(kgs_data:stages).	
}

// calls newton algorithm to solve guidance equations and update guidance solution
function update_guidance {

	// update solution vector based on elapsed time between guidance updates
	local dt is (kgs_data:clg:time - kgs_data:clg:time_prev) / kgs_data:scale:time.
	local tf is kgs_data:clg:x[6] - dt.

	local lambda is list_to_vector(kgs_data:clg:x, 0) * cos(dt * constant:radtodeg) + list_to_vector(kgs_data:clg:x, 3) * sin(dt * constant:radtodeg).
	local lambda_dot is -list_to_vector(kgs_data:clg:x, 0) * sin(dt * constant:radtodeg) + list_to_vector(kgs_data:clg:x, 3) * cos(dt * constant:radtodeg).
	set kgs_data:clg:x to vector_to_list(list(lambda, lambda_dot, tf)).
	
	// solve guidance equations
	local solution is newton(jacobian_function@, kgs_data:clg:x, kgs_data, kgs_settings:eps_newton, kgs_settings:debug_function).
	
	// update data lexicon with solution
	set kgs_data:clg:x to solution[0].
    set kgs_data:clg:last_iter_count to solution[1].
    set kgs_data:clg:last_err to solution[2].
	set kgs_data:ui:status_index to kgs_data:ui:status_index + 1.
	set kgs_data:clg:first_pass to true.
	
	// if last error value is below tolerance update guidance
	if kgs_data:clg:last_err < kgs_settings:eps_newton {
		set kgs_data:clg:time_guide to kgs_data:clg:time.
		set kgs_data:clg:x_guide to kgs_data:clg:x:copy.
		set kgs_data:clg:final_mass to kgs_data:clg:yf[0].
		
		// on first converged solution reset time warp, save guidance, and set converged flag
		if not kgs_data:clg:converged {	
			set kuniverse:timewarp:mode to "physics".
			set kuniverse:timewarp:rate to kgs_data:clg:warp_rate.
			if kgs_inputs:objective:haskey("save_initial_guidance") {
				save_initial_guidance().
			}
			set kgs_data:clg:converged to true.
		}	
	}
}

// function called by integrator that calculates derivatives of the 43 equations to be solved
function trajectory_function {

	parameter t.	// time
	parameter y.	// state
	parameter p.	// params

	// initialize derivative list
	local df is list().
	
	// calculate attitude
	local cos_t is cos(t * constant:radtodeg).
	local sin_t is sin(t * constant:radtodeg).
    local lambda is p:lambda_i[0] * cos_t + p:lambda_i[1] * sin_t.

	// calculate acceleration magnitude
	local a_thrust is 0.
	if p:stage:mode = "c_accel" {
        set a_thrust to p:stage:a_limit.     
    }
    else if p:stage:mode = "c_thrust" {
        set a_thrust to p:stage:thrust / y[0].
    }

	// add mass, position, and velocity derivatives
	local r3 is y[1]:mag ^ 3.
	df:add(-a_thrust * y[0] / p:stage:vex).
	df:add(y[2]).
	df:add(-y[1] / r3 + a_thrust * lambda:normalized).

	// calculate derivative of acceleration with respect to lambda
	local dl is list().
	local at_lam3 is a_thrust / lambda:mag ^ 3.
	dl:add(at_lam3 * v(lambda:mag ^ 2 - lambda:x ^ 2, -lambda:x * lambda:y, -lambda:x * lambda:z)).
	dl:add(at_lam3 * v(-lambda:x * lambda:y, lambda:mag ^ 2 - lambda:y ^ 2, -lambda:y * lambda:z)).
	dl:add(at_lam3 * v(-lambda:x * lambda:z, -lambda:y * lambda:z, lambda:mag ^ 2 - lambda:z ^ 2)).
	
	// calculate the derivative of acceleration with respect to position
	local dr is list().
	local r5 is y[1]:mag ^ 5.
	dr:add(v(3 * y[1]:x ^ 2 / r5 - 1 / r3, 3 * y[1]:x * y[1]:y / r5, 3 * y[1]:x * y[1]:z / r5)).
	dr:add(v(3 * y[1]:x * y[1]:y / r5, 3 * y[1]:y ^ 2 / r5 - 1 / r3, 3 * y[1]:y * y[1]:z / r5)).
	dr:add(v(3 * y[1]:x * y[1]:z / r5, 3 * y[1]:y * y[1]:z / r5, 3 * y[1]:z ^ 2 / r5 - 1 / r3)).
	
	// calculate time derivative of the derivative of the final state with respect to initial lambda
	for i in range(3) {
		local j is 2 * i + 3.
		df:add(y[j + 1]).
		local dri is v(vdot(dr[0], y[j]), vdot(dr[1], y[j]), vdot(dr[2], y[j])).
		df:add(dri + cos_t * dl[i]).
	}

	for i in range(3) {
		local j is 2 * i + 9.
		df:add(y[j + 1]).
		local dri is v(vdot(dr[0], y[j]), vdot(dr[1], y[j]), vdot(dr[2], y[j])).
		df:add(dri + sin_t * dl[i]).
	}

	return df.
}

// function called by newton algorithm to calculate jacobian and error information
function jacobian_function {

	parameter x.	// solution vector
	parameter p.	// params
	
	// initialize jacobian/error matrix
	local fdf is list(0, 0, 0, 0, 0, 0).

	// set initial values for integration
    local t is 0.
	local met is (kgs_data:clg:time - kgs_data:olg:launch_time) / kgs_data:scale:time.
    local tf is x[6].
	local pt is lexicon(
		"lambda_i", list(list_to_vector(x, 0), list_to_vector(x, 3)),
		"settings", lexicon("max_step", kgs_settings:max_step)
	).	
	if kgs_settings:integrator = rkf45@ {
		pt:settings:add("eps_rkf45", kgs_settings:eps_rkf45).
	}
    local y is deep_copy(p:clg:y).
	
	// integration loop
	local stage_ptr is p:clg:stages:iterator.
	local last is false.
	
	until not stage_ptr:next {

		local s is stage_ptr:value.
		set pt:stage to s.
		
		// update mass for current stage
		if stage_ptr:index <> 0 {
			set y[0] to s:mass_total.
		}
		
		// calculate stage burn time	
	    local tb is stage_burn_time(s, (met + t) * kgs_data:scale:time, y[0]).
		
		// set final integration time
		local tout is t + tb.
		
		// limit final integration time to final time and prevent negative integration time
		if tout > tf {
			set tout to tf.
			set last to true.
		} else if tout < t {
			set tout to t.
		}
		
		// integrate to tout and update time and state
		local output is kgs_settings:integrator(trajectory_function@, pt, 15, t, tout, y, kgs_settings).
        set t to output[0].
        set y to output[1].
  
		// if last is true final time has been reached
		if last {
            break.
        }
	}
	
	// if tout has not been reach coast to tout
	if not last {
		set pt:stage to lexicon(
			"mode", "coast",
			"vex", 1
		).
		
		local tout is tf.
		
		local output is kgs_settings:integrator(trajectory_function@, pt, 15, t, tout, y, kgs_settings).
        set t to output[0].
        set y to output[1].
	}	
	
	// save final state to data lexicon
	set p:clg:yf to y.

	// calculate final lambda, lambda_dot, and state
	local yf is y.
	local cos_t is cos(x[6] * constant:radtodeg).
	local sin_t is sin(x[6] * constant:radtodeg).
	local lambda is pt:lambda_i[0] * cos_t + pt:lambda_i[1] * sin_t.
	local lambda_dot is -pt:lambda_i[0] * sin_t + pt:lambda_i[1] * cos_t.

	local m is yf[0].
	local rf is yf[1].
	local vf is yf[2].
	local hf is vcrs(rf, vf).
	
    local a_thrust is 0.
	if pt:stage:mode = "c_accel" {
        set a_thrust to pt:stage:a_limit.     
    }
    else if pt:stage:mode = "c_thrust" {
        set a_thrust to pt:stage:thrust / m.
    }

	// calculate jacobian and error vector

	local af is -rf / rf:mag ^ 3 + a_thrust * lambda:normalized.

	local d_rdotr_d_r is 2 * rf.
	local d_vdotv_d_v is 2 * vf.
	
	local sigma is vcrs(rf, lambda_dot) - vcrs(vf, lambda).
	local d_sigma_d_t is -vcrs(rf, lambda) - vcrs(af, lambda).
	
	local d_sigma_d_lambda_i is list(
		vcrs(yf[ 0 + 3], lambda_dot) + vcrs(rf, v(-sin_t, 0, 0)) - vcrs(yf[1 + 3], lambda) - vcrs(vf, v(cos_t, 0, 0)),
		vcrs(yf[ 2 + 3], lambda_dot) + vcrs(rf, v(0, -sin_t, 0)) - vcrs(yf[3 + 3], lambda) - vcrs(vf, v(0, cos_t, 0)),
		vcrs(yf[ 4 + 3], lambda_dot) + vcrs(rf, v(0, 0, -sin_t)) - vcrs(yf[5 + 3], lambda) - vcrs(vf, v(0, 0, cos_t)),
		vcrs(yf[ 6 + 3], lambda_dot) + vcrs(rf, v( cos_t, 0, 0)) - vcrs(yf[5 + 5], lambda) - vcrs(vf, v(sin_t, 0, 0)),
		vcrs(yf[ 8 + 3], lambda_dot) + vcrs(rf, v(0,  cos_t, 0)) - vcrs(yf[5 + 7], lambda) - vcrs(vf, v(0, sin_t, 0)),
		vcrs(yf[10 + 3], lambda_dot) + vcrs(rf, v(0, 0,  cos_t)) - vcrs(yf[5 + 9], lambda) - vcrs(vf, v(0, 0, sin_t))
	).
	
	// set radius constraint derivative and error
	set fdf[0] to list(
		vdot(d_rdotr_d_r, yf[0 + 3]),
		vdot(d_rdotr_d_r, yf[2 + 3]),
		vdot(d_rdotr_d_r, yf[4 + 3]),
		vdot(d_rdotr_d_r, yf[6 + 3]),
		vdot(d_rdotr_d_r, yf[8 + 3]),
		vdot(d_rdotr_d_r, yf[10 + 3]),
		vdot(d_rdotr_d_r, vf),
		vdot(rf, rf) - kgs_data:clg:position_constraint
	).
	
	// set velocity constraint derivative and error
	set fdf[1] to list(
		vdot(d_vdotv_d_v, yf[1 + 3]),
		vdot(d_vdotv_d_v, yf[3 + 3]),
		vdot(d_vdotv_d_v, yf[5 + 3]),
		vdot(d_vdotv_d_v, yf[7 + 3]),
		vdot(d_vdotv_d_v, yf[9 + 3]),
		vdot(d_vdotv_d_v, yf[11 + 3]),
		vdot(d_vdotv_d_v, af),
		vdot(vf, vf) - kgs_data:clg:velocity_constraint
	).

	// set flight path angle constraint derivative and error
	set fdf[2] to list(
		vdot(vf, yf[0 + 3]) + vdot(rf, yf[1 + 3]),
		vdot(vf, yf[2 + 3]) + vdot(rf, yf[3 + 3]),
		vdot(vf, yf[4 + 3]) + vdot(rf, yf[5 + 3]),
		vdot(vf, yf[6 + 3]) + vdot(rf, yf[7 + 3]),
		vdot(vf, yf[8 + 3]) + vdot(rf, yf[9 + 3]),
		vdot(vf, yf[10 + 3]) + vdot(rf, yf[11 + 3]),
		vdot(vf, vf) + vdot(rf, af),
		vdot(rf, vf) - kgs_data:clg:flight_path_angle_constraint
	).

    if p:clg:mode = 0 {
		// set auxiliary constraint derivatives and errors
		set fdf[3] to list(
			d_sigma_d_lambda_i[0]:x,
			d_sigma_d_lambda_i[1]:x,
			d_sigma_d_lambda_i[2]:x,
			d_sigma_d_lambda_i[3]:x,
			d_sigma_d_lambda_i[4]:x,
			d_sigma_d_lambda_i[5]:x,
			d_sigma_d_t:x,
			sigma:x
		).
		set fdf[4] to list(
			d_sigma_d_lambda_i[0]:y,
			d_sigma_d_lambda_i[1]:y,
			d_sigma_d_lambda_i[2]:y,
			d_sigma_d_lambda_i[3]:y,
			d_sigma_d_lambda_i[4]:y,
			d_sigma_d_lambda_i[5]:y,
			d_sigma_d_t:y,
			sigma:y
		).
		set fdf[5] to list(
			d_sigma_d_lambda_i[0]:z,
			d_sigma_d_lambda_i[1]:z,
			d_sigma_d_lambda_i[2]:z,
			d_sigma_d_lambda_i[3]:z,
			d_sigma_d_lambda_i[4]:z,
			d_sigma_d_lambda_i[5]:z,
			d_sigma_d_t:z,
			sigma:z
		).
    }

	if p:clg:mode > 0 {
		// set inclination constraint and derivative and error
		local d_hz_d_r is v(vf:y, -vf:x, 0).
		local d_hz_d_v is v(-rf:y, rf:x, 0).
		
		set fdf[3] to list(
			vdot(d_hz_d_r, yf[0 + 3]) + vdot(d_hz_d_v, yf[1 + 3]),
			vdot(d_hz_d_r, yf[2 + 3]) + vdot(d_hz_d_v, yf[3 + 3]),
			vdot(d_hz_d_r, yf[4 + 3]) + vdot(d_hz_d_v, yf[5 + 3]),
			vdot(d_hz_d_r, yf[6 + 3]) + vdot(d_hz_d_v, yf[7 + 3]),
			vdot(d_hz_d_r, yf[8 + 3]) + vdot(d_hz_d_v, yf[9 + 3]),
			vdot(d_hz_d_r, yf[10 + 3]) + vdot(d_hz_d_v, yf[11 + 3]),
			vdot(d_hz_d_r, vf) + vdot(d_hz_d_v, af),
			hf:z - cos(kgs_inputs:objective:inclination)
		).
		
		// set auxiliary constraint derivative and error
		set fdf[4] to list(
			vdot(d_sigma_d_lambda_i[0], hf) + vdot(sigma, vcrs(-vf, yf[ 0 + 3]) + vcrs(rf, yf[ 1 + 3])),
			vdot(d_sigma_d_lambda_i[1], hf) + vdot(sigma, vcrs(-vf, yf[ 2 + 3]) + vcrs(rf, yf[ 3 + 3])),
			vdot(d_sigma_d_lambda_i[2], hf) + vdot(sigma, vcrs(-vf, yf[ 4 + 3]) + vcrs(rf, yf[ 5 + 3])),
			vdot(d_sigma_d_lambda_i[3], hf) + vdot(sigma, vcrs(-vf, yf[ 6 + 3]) + vcrs(rf, yf[ 7 + 3])),
			vdot(d_sigma_d_lambda_i[4], hf) + vdot(sigma, vcrs(-vf, yf[ 8 + 3]) + vcrs(rf, yf[ 9 + 3])),
			vdot(d_sigma_d_lambda_i[5], hf) + vdot(sigma, vcrs(-vf, yf[10 + 3]) + vcrs(rf, yf[11 + 3])),
			vdot(d_sigma_d_t, hf) + vdot(sigma, vcrs(vf, vf) + vcrs(rf, af)),
			vdot(sigma, hf)
		).
	}
	
	if p:clg:mode = 1 {
		// set auxiliary constraint derivative and error
		set fdf[5] to list(
			d_sigma_d_lambda_i[0]:z,
			d_sigma_d_lambda_i[1]:z,
			d_sigma_d_lambda_i[2]:z,
			d_sigma_d_lambda_i[3]:z,
			d_sigma_d_lambda_i[4]:z,
			d_sigma_d_lambda_i[5]:z,
			d_sigma_d_t:z,
			sigma:z
		).
	}
	
	if p:clg:mode = 2 {
		// set node vector constraint derivative and error	
		local d_lan_d_r is vcrs(vf, kgs_data:clg:node_d).
		local d_lan_d_v is vcrs(kgs_data:clg:node_d, rf).
		set fdf[5] to list(
			vdot(d_lan_d_r, yf[0 + 3]) + vdot(d_lan_d_v, yf[1 + 3]),
			vdot(d_lan_d_r, yf[2 + 3]) + vdot(d_lan_d_v, yf[3 + 3]),
			vdot(d_lan_d_r, yf[4 + 3]) + vdot(d_lan_d_v, yf[5 + 3]),
			vdot(d_lan_d_r, yf[6 + 3]) + vdot(d_lan_d_v, yf[7 + 3]),
			vdot(d_lan_d_r, yf[8 + 3]) + vdot(d_lan_d_v, yf[9 + 3]),
			vdot(d_lan_d_r, yf[10 + 3]) + vdot(d_lan_d_v, yf[11 + 3]),
			vdot(d_lan_d_r, vf) + vdot(d_lan_d_v, af),
			vdot(kgs_data:clg:node_d, hf)
		).
	}

	return fdf.
}

// returns attitude based on last guidance solution at current time
function get_attitude {

	// if coasting set attitude to prograde
	if kgs_data:stages[0]:mode = "coast" and (kgs_data:stages[0]:time_end - (time:seconds - kgs_data:olg:launch_time)) > 30 {
		return ship:prograde:vector.
	}
	
	local t is (time:seconds - kgs_data:clg:time_guide) / kgs_data:scale:time * constant:radtodeg.
	
	// if coasting and time to next stage is less than 30 seconds, set attitude based on next stage start time
	if kgs_data:stages[0]:mode = "coast" and (kgs_data:stages[0]:time_end - (time:seconds - kgs_data:olg:launch_time)) > 0 {
		set t to (kgs_data:stages[0]:time_end + kgs_data:olg:launch_time - kgs_data:clg:time_guide) / kgs_data:scale:time * constant:radtodeg.
	}
	
	local attitude is swap_yz(list_to_vector(kgs_data:clg:x_guide, 0) * cos(t) + list_to_vector(kgs_data:clg:x_guide, 3) * sin(t)):normalized.

	return lookdirup(attitude, ship:up:vector * angleaxis(kgs_data:olg:roll_angle, attitude)).
}

// returns tgo based on last guidance solutions
function get_tgo {

	if not kgs_data:clg:terminal_guidance {
		return kgs_data:clg:x_guide[6] * kgs_data:scale:time - (time:seconds - kgs_data:clg:time_guide).
	}
	else {
		return kgs_data:clg:time_final - time:seconds.
	}
}

// returns throttle value
function get_throttle {

    local throttle_command is 0.
	
	local stage_throttle is (kgs_data:stages[0]:throttle - kgs_data:stages[0]:min_throttle) / (1 - kgs_data:stages[0]:min_throttle).

    if kgs_data:stages[0]:mode = "c_accel" {
		// if phase is acceleration limited determine required throttle
		
        local m to ship:mass * 1000.
		
		// calculate throttle required
        local throttle_alimit is (m * kgs_data:stages[0]:a_limit) / kgs_data:stages[0]:thrust * kgs_data:stages[0]:throttle.
		
		// adjust throttle for throttle limit
		set throttle_alimit to (throttle_alimit - kgs_data:stages[0]:min_throttle) / (1 - kgs_data:stages[0]:min_throttle).

		// use calculate throttle only if it is less than stage specified throttle
        if stage_throttle > throttle_alimit {
            set throttle_command to throttle_alimit.
        }
        else {
            set throttle_command to stage_throttle.
        }
    }
    else {
        set throttle_command to stage_throttle.
    }
	
    return throttle_command.
}

// set attitude to prograde and return false unless attitude has settled
function settle {

	local look_at is ship:prograde:vector.
	local look_up is ship:up:vector * angleaxis(kgs_data:olg:roll_angle, look_at).
	set guidance_command to list(lookdirup(look_at, look_up), 0).

	if vang(ship:prograde:vector, ship:facing:vector) < 1 and ship:angularvel:mag < 0.02 {
		return true.
	}
	else {
		return false.
	}
}