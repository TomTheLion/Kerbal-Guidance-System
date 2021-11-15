@lazyglobal off.

//
// kerbal guidance system function library
//

// intializes global data lexicon and sets open loop guidance event trigger, inputs are first validated then open and
// closed loop guidance parameters are set, vehicle information is parsed to build a list of the guided stages, lastly
// timed stage events are seperated into open and closed loop events and the open loop guidance trigger is set
function initialize {

	// create data lexicon and set initial flags
	global kgs_data is lexicon(
		"scale", lexicon(),
		"clg", lexicon(
			"events", list(),
			"active_guidance", false,
			"first_pass", false,
			"converged", false,
			"terminal_guidance", false,
			"stop_ui", false,
			"stop_guidance", false,	
			"staging", false,
			"final_mass", 0
		),
		"olg", lexicon(			
			"events", list()
		),
		"stages", list(),
		"ui", lexicon(
			"status_index", 0,
			"status_icon", list(" ", "*", "**", "***")
		)
	).

	// validate inputs
	validate_inputs().

	// set newton's method options
	if kgs_settings:debug_level = 0 {
		set kgs_settings:debug_function to 0.
	}
	else {
		set kgs_settings:debug_function to debug_function@.
	}

	set kgs_settings:newton_max_time to 20.
	
	// set dimensional scale factors
	local d_ref is body:radius + kgs_inputs:objective:altitude.
	local v_ref is sqrt(body:mu / d_ref).
	set kgs_data:scale to lexicon(
		"distance", d_ref,
		"velocity", v_ref,
		"time", d_ref / v_ref,
		"acceleration", v_ref * v_ref / d_ref
	).
	
	// set open loop guidance parameters
	add_keys(kgs_data:olg, kgs_inputs:open_loop_guidance, list("pitch_time", "pitch_aoa", "pitch_duration")).
	add_keys_if(kgs_data:olg, kgs_inputs:open_loop_guidance, list("launch_direction")).
	add_keys_if_else(
		kgs_data:olg, kgs_inputs:open_loop_guidance,
		list("booster_name", "throttle", "pitch_rate", "roll_time", "roll_angle", "azimuth_adjustment", "launch_time_adjustment", "launch_countdown"),
		list("Boost Stage", 1, 1, kgs_data:olg:pitch_time + 1, 0, 0, 0, 10)
	).
	
	set_launch_time().
	set_launch_azimuth().
	
	// set closed loop guidance parameters

	// set position, velocity, and flight path angle constraints
	local r_des is body:radius + kgs_inputs:objective:altitude.
	local r_apo is r_des.
	local r_per is r_des.

	if kgs_inputs:objective:haskey("apoapsis") {
		set r_apo to max(r_des, body:radius + kgs_inputs:objective:apoapsis).
	}

	if kgs_inputs:objective:haskey("periapsis") {
		set r_per to min(r_des, body:radius + kgs_inputs:objective:periapsis).
	}

	local sma is (r_apo + r_per) / 2.
	local v_des is sqrt(body:mu * (2 / r_des - 1 / sma)).
	local v_per is sqrt(body:mu * (2 / r_per - 1 / sma)).
	local theta_des is safe_arccos(r_per * v_per / r_des / v_des).

	kgs_data:clg:add("position_constraint", r_des ^ 2 / kgs_data:scale:distance ^ 2).
	kgs_data:clg:add("velocity_constraint", v_des ^ 2 / kgs_data:scale:velocity ^ 2).
	kgs_data:clg:add("flight_path_angle_constraint", r_des * v_des * cos(90 - theta_des) / kgs_data:scale:distance / kgs_data:scale:velocity).

	kgs_data:ui:add("velocity_desired", v_des).
	kgs_data:ui:add("flight_path_angle_desired", theta_des).

	// set guidance mode	
	if kgs_inputs:objective:haskey("inclination") and kgs_inputs:objective:haskey("lan") {
		kgs_data:clg:add("mode", 2).
	}
	else if kgs_inputs:objective:haskey("inclination") {
		kgs_data:clg:add("mode", 1).
	}
	else {
		kgs_data:clg:add("mode", 0).
	}

	// set initial state vector
	local yi is list(0).
	for i in range(14) {
        yi:add(v(0, 0, 0)).
    }
	kgs_data:clg:add("yi", yi).
	
	// set vehicle parameters
	set_stages().

	// set timed stages
	local event_ptr is kgs_inputs:timed_events:iterator.
	
	until not event_ptr:next {
		if in_list(event_ptr:value:type, list("jettison", "coast")) {
			kgs_data:clg:events:add(event_ptr:value).
		}
		else {
			kgs_data:olg:events:add(event_ptr:value).
		}
	}

	// set open loop guidance trigger
	global olg_event_ptr is kgs_data:olg:events:iterator.
	olg_event_ptr:next.
	set_olg_event_trigger().
}

// input validation
function validate_inputs {

	local objective_keys is list(
		"altitude", 
		"apoapsis", 
		"periapsis", 
		"inclination", 
		"lan", 
		"payload", 
		"save_initial_guidance", 
		"load_initial_guidance"
		).
	local open_loop_guidance_keys is list(
		"pitch_time", 
		"pitch_aoa", 
		"pitch_duration", 
		"pitch_rate", 
		"roll_time",
		"roll_angle",
		"booster_name", 
		"throttle", 
		"azimuth_adjustment", 
		"launch_direction", 
		"launch_time_adjustment", 
		"launch_countdown"
		).
	local vehicle_keys is list(
		"mass_total",
		"mass_dry",
		"thrust",
		"isp",
		"name",
		"g_limit",
		"staging_sequence",
		"rcs_ullage",
		"throttle",
		"min_throttle"
	).

	// validate inputs
	require_variable(defined kgs_settings, "global variable", "kgs_settings").
	require_variable(defined kgs_inputs, "global variable", "kgs_inputs").
	require_keys(kgs_settings, list("ipu", "integrator", "max_step", "eps_newton", "ui_refresh_rate", "guidance_refresh_rate", "debug_level"), " in kgs_settings").
	require_keys(kgs_inputs, list("objective", "open_loop_guidance", "timed_events", "vehicle"), " in kgs_inputs").
	
	if kgs_settings:integrator = rkf45@ {
		require_key(kgs_settings, "eps_rkf45", " in kgs_settings").
	}

	// validate objective
	require_key(kgs_inputs:objective, "altitude", " in kgs_inputs:objective").
	limit_keys(kgs_inputs:objective, objective_keys, " in kgs_inputs:objective").

	// validate open loop guidance
	require_keys(kgs_inputs:open_loop_guidance, list("pitch_time", "pitch_aoa", "pitch_duration"), " in kgs_inputs:open_loop_guidance").
	limit_keys(kgs_inputs:open_loop_guidance, open_loop_guidance_keys, " in kgs_inputs:open_loop_guidance").
	
	// validate timed staging
	for event in kgs_inputs:timed_events {
		if not in_list(event:type, list("launch", "stage", "throttle", "action group", "activate guidance", "jettison", "coast")) {
			crash("Unknown event type " + char(34) + event:type + char(34) + " in kgs_inputs:timed_events.").
		}

		require_keys(event, list("time", "type", "description"), " in event in kgs_inputs:timed_events").
		
		if event:type = "throttle" {
			require_key(event, "setting", " in throttle event in kgs_inputs:timed_events").
		}
		
		if event:type = "coast" {
			require_key(event, "duration", " in coast event in kgs_inputs:timed_events").
		}
		
		if event:type = "jettison" {
			require_key(event, "mass", " in jettison event in kgs_inputs:timed_events").
		}
		
		if event:type = "action group" {
			require_key(event, "group", " in action group event in kgs_inputs:timed_events").
		}
	}	
	
	// validate vehicle
	for vehicle_stage in kgs_inputs:vehicle {
		require_keys(vehicle_stage, list("mass_total", "mass_dry", "thrust", "isp"), " in kgs_inputs:vehicle").
		limit_keys(vehicle_stage, vehicle_keys, " in kgs_inputs:vehicle").
	}
}

// calculate time to desired node
function time_to_node {

	parameter launch_direction.	// desired launch direction (north or south)

	// calculate the desired node vector
	local k is v(0, 1, 0).
	local node_d is solarprimevector * angleaxis(-kgs_inputs:objective:lan, k).

	// calculate the angle between the projection of the ships position onto the equator and the node vector for the
	// desired inclination
	local side_a is ship:geoposition:lat.
	local angle_a is kgs_inputs:objective:inclination.
	local side_b is safe_arcsin(tan(side_a) / tan(angle_a)).
	
	if launch_direction = "south" {
		set side_b to 180 - side_b.
	}
	
	// calculate current node vector for desired inclination
	local node_c is vxcl(k, -ship:orbit:body:position):normalized * angleaxis(side_b, k).

	// calculate time between current and desired nodes based on body rotation rate
	local delta_angle is  vang(node_d, node_c).
	
	if  vdot(k, vcrs(node_d, node_c)) < 0 {
		set delta_angle to 360 - delta_angle.
	}
	
	local delta_time is ship:body:rotationperiod * delta_angle / 360 + kgs_data:olg:launch_time_adjustment.
	
	if delta_time < 0 {
		set delta_time to delta_time + ship:body:rotationperiod.
	}
	
	return delta_time.
}

// set launch time
function set_launch_time {

	local delta_time is 0.

	// calculate delta time to desired node if lan is specified
	if kgs_inputs:objective:haskey("inclination") and kgs_inputs:objective:haskey("lan") {
		if kgs_data:olg:haskey("launch_direction") {	
			set delta_time to time_to_node(kgs_data:olg:launch_direction).
		}
		else {
			local delta_time_north is time_to_node("north").
			local delta_time_south is time_to_node("south").
			
			if delta_time_north < delta_time_south {
				kgs_data:olg:add("launch_direction", "north").
				set delta_time to delta_time_north.
			}
			else {
				kgs_data:olg:add("launch_direction", "south").
				set delta_time to delta_time_south.
			}
		}
	}
	
	if delta_time < kgs_data:olg:launch_countdown {
		set delta_time to kgs_data:olg:launch_countdown.
	}

	kgs_data:olg:add("launch_time", time:seconds + delta_time).
}

// calculate and set launch azimuth
function set_launch_azimuth {

	local launch_azimuth is 0.

	// if inclination is specified use spherical trig to calculate the launch azimuth as a function of inclination and latitude
	if kgs_inputs:objective:haskey("inclination") {
		local beta_inertial is safe_arcsin(cos(kgs_inputs:objective:inclination) / cos(geoposition:lat)).
		local vorbit is sqrt(body:mu / (body:radius + kgs_inputs:objective:altitude)).
		local vx is vorbit * sin(beta_inertial) - vcrs(body:angularvel, body:position):mag.
		local vy is vorbit * cos(beta_inertial).
		set launch_azimuth to arctan2(vy, vx).
	}
	
	if not kgs_data:olg:haskey("launch_direction") {	
		kgs_data:olg:add("launch_direction", "north").
	}
	
    if kgs_data:olg:launch_direction = "north" {
        kgs_data:olg:add("launch_azimuth", 90 - launch_azimuth + kgs_data:olg:azimuth_adjustment).
    }
    else {
        kgs_data:olg:add("launch_azimuth", 90 + launch_azimuth + kgs_data:olg:azimuth_adjustment).
    }
}

// converts input vehicle data and to a list of stages that can be passed to the integrator
function set_stages {

	local g0 is 9.8067 / kgs_data:scale:acceleration.
	
	local stage_ptr is kgs_inputs:vehicle:iterator.
	
	until not stage_ptr:next {
	
		local vehicle_stage is stage_ptr:value.
	
		local stage_data is lexicon().
				
		// add payload mass
		if kgs_inputs:objective:haskey("payload") {
			set vehicle_stage:mass_total to vehicle_stage:mass_total + kgs_inputs:objective:payload.
			set vehicle_stage:mass_dry to vehicle_stage:mass_dry + kgs_inputs:objective:payload.
		}
		
		// add optional keys if they are excluded
		add_keys_if_else(stage_data, vehicle_stage, list("name", "throttle", "min_throttle"), list("guided stage: " + (stage_ptr:index + 1), 1, 0)).
		
		// scale thrust and isp and add to stage data
		set vehicle_stage:thrust to vehicle_stage:thrust / kgs_data:scale:acceleration.
		set vehicle_stage:isp to vehicle_stage:isp / kgs_data:scale:time.
		stage_data:add("thrust", vehicle_stage:thrust * stage_data:throttle).
		stage_data:add("vex", vehicle_stage:isp * g0).
		stage_data:add("index", stage_ptr:index).
		
		// if stage has g limit split it into two stages
		if vehicle_stage:haskey("g_limit") {
			local a_limit is vehicle_stage:g_limit * g0.
			local m_limit is stage_data:thrust / a_limit.
			
			if m_limit > vehicle_stage:mass_total {
				// if already in constant acceleration phase only add one stage
				add_keys_if(stage_data, vehicle_stage, list("mass_total", "mass_dry", "staging_sequence", "rcs_ullage")).
				stage_data:add("a_limit", a_limit).
				stage_data:add("mode", "c_accel").
				kgs_data:stages:add(stage_data).
			}
			else {
				// otherwise create a new stage for when acceleration limit is reached, both stages will have identical information
				// except the first part will operate under constant thrust and its final mass is set to when the acceleration would
				// exceed the limit and the second part will be under constant acceleration and have the staging sequence

				// set first part
				local stage_data_limit is stage_data:copy.
				stage_data:add("mass_total", vehicle_stage:mass_total).
				stage_data:add("mass_dry", m_limit).
				stage_data:add("mode", "c_thrust").
				
				// set second part
				stage_data_limit:add("mass_total", m_limit).
				add_keys_if(stage_data_limit, vehicle_stage, list("mass_dry", "staging_sequence", "rcs_ullage")).
				stage_data_limit:add("a_limit", a_limit).
				stage_data_limit:add("mode", "c_accel").
				set stage_data_limit:name to stage_data_limit:name + " (C-Accel)".
				
				// add stages
				kgs_data:stages:add(stage_data).
				kgs_data:stages:add(stage_data_limit).		
			}	
		}
		else {
			// if stage does not have a g limit add stage
			add_keys_if(stage_data, vehicle_stage, list("mass_total", "mass_dry", "staging_sequence", "rcs_ullage")).
			stage_data:add("mode", "c_thrust").
			kgs_data:stages:add(stage_data).
		}
	}
}

// copies stage information to data lexicon and sets event trigger
function set_olg_event_trigger {
	local event_time is kgs_data:olg:launch_time + olg_event_ptr:value:time.
	set kgs_data:ui:olg_event_time to event_time.
	set kgs_data:ui:olg_event_name to olg_event_ptr:value:description.
	set kgs_data:ui:olg_event_type to olg_event_ptr:value:type.
	when time:seconds > event_time then {
		olg_event_trigger().
	}
}.

// trigger specified event and set next if there is one
function olg_event_trigger {

    local event_type is olg_event_ptr:value:type.
	
	if olg_event_ptr:value:haskey("rename"){
		set kgs_data:olg:booster_name to olg_event_ptr:value:rename.
	}

	if event_type = "launch" or event_type = "stage" {
        stage.
    }
    else if event_type = "throttle" {
        set kgs_data:olg:throttle to olg_event_ptr:value:setting.
    }
	else if event_type = "action group" {
        trigger_action_group(olg_event_ptr:value:group).
    }
    else if event_type = "activate guidance" {
		// saves warp rate and cancels warp, then initializes guidance, once converged warp will be reset to original value
		kgs_data:clg:add("warp_rate", kuniverse:timewarp:rate).
		kuniverse:timewarp:cancelwarp.
        initialize_closed_loop_guidance().
    }
	
	if olg_event_ptr:next {
		set_olg_event_trigger().
	}
}

// set next stage trigger
function set_stage_trigger {

	// get stage data
    local ti is time:seconds.
	local met is ti - kgs_data:olg:launch_time.
    local mi is ship:mass * 1000.
	local tgo_stage is stage_burn_time(kgs_data:stages[0], met, mi) * kgs_data:scale:time.
	local tf is ti + tgo_stage.
	
	// set ui stage information
	set kgs_data:ui:clg_event_time to tf.

	if kgs_data:stages[1]:haskey("event_name") {
		set kgs_data:ui:clg_event_name to kgs_data:stages[1]:event_name.
	}
	else {
		set kgs_data:ui:clg_event_name to kgs_data:stages[1]:name.
	}

	// pause guidance two seconds before staging, prevents from updating guidance while stages are being removed
	if time:seconds > tf - 2 {
		set kgs_data:clg:staging to true.
	}
	else {
		set kgs_data:clg:staging to false.
	}
	
    when time:seconds > tf - 2 then {
        set kgs_data:clg:staging to true.
    }

	// set stage triggers depending on parameters
    if kgs_data:stages[0]:haskey("staging_sequence") {
	
		// staging sequence
		local n is kgs_data:stages[0]:staging_sequence:length.
		local stage_time is tf.
		
		for i in range(n - 1) {
			set stage_time to stage_time + kgs_data:stages[0]:staging_sequence[i].
			local trigger_time is stage_time.
			when time:seconds > trigger_time then {
				stage.
			} 
		}
		
		local trigger_time is stage_time + kgs_data:stages[0]:staging_sequence[n - 1].
		when time:seconds > trigger_time then {
			kgs_data:stages:remove(0).
			set guidance_command[1] to kgs_data:stages[0]:throttle.
			stage.
			if kgs_data:stages:length > 1 {
				set_stage_trigger().
			}
			else {
				set kgs_data:clg:staging to false.
			}
		}

		// rcs ullage trigger
		if kgs_data:stages[0]:haskey("rcs_ullage") {
		
			local rcs_start is tf + kgs_data:stages[0]:rcs_ullage[0].
			local rcs_end is rcs_start + kgs_data:stages[0]:rcs_ullage[1].
	
			when time:seconds > rcs_start then {
				rcs on.
				set ship:control:fore to 1.0.
			}

			when time:seconds > rcs_end then {
				set ship:control:fore to 0.0.
				rcs off.
			}
		}			
    }
    else {
		// if no staging sequence move to next stage without staging
        when time:seconds > tf then {
            kgs_data:stages:remove(0).
            if kgs_data:stages:length > 1 {
                set_stage_trigger().
            }
			else {
				set kgs_data:clg:staging to false.
			}
	    }    
    }
}

// triggers specified action group
function trigger_action_group {
	
	parameter group_number.	// action group number
	
	if group_number = 1 {
		ag1 on.
	}
	else if group_number = 2 {
		ag2 on.
	}
	else if group_number = 3 {
		ag3 on.
	}
	else if group_number = 4 {
		ag4 on.
	}
	else if group_number = 5 {
		ag5 on.
	}
	else if group_number = 6 {
		ag6 on.
	}
	else if group_number = 7 {
		ag7 on.
	}
	else if group_number = 8 {
		ag8 on.
	}
	else if group_number = 9 {
		ag9 on.
	}
	else if group_number = 10 {
		ag10 on.
	}
}

// calculate stage burn time
function stage_burn_time {

	parameter s.	// stage
	parameter met.	// mission elapsed time
	parameter mi.	// initial mass

	if s:mode = "c_thrust" {
		return (mi - s:mass_dry) / (s:thrust / s:vex).
	}
	else if s:mode = "c_accel" {	
		return s:vex / s:a_limit * ln(mi / s:mass_dry).
	}
	else if s:mode = "coast" {
		return (s:time_end - met) / kgs_data:scale:time.
	}
}

// warp to launch time
function warp_to_launch_time {
    kuniverse:timewarp:warpto(kgs_data:olg:launch_time - kgs_data:olg:launch_countdown).
}

// function called within newton algorithm to debug
function debug_function {
	
	parameter x.			// current solution vector
	parameter p.			// params
	parameter iteration.	// current iteration count
	parameter x_new.		// new solution vector
	parameter fdf.			// jacobian and error function
	parameter fmax.			// current max error

	local debug_line is 34.

	// display guidance vectors
	if kgs_settings:debug_level > 0 {

		if iteration = 1 {
			clearvecdraws().
			global lambda_vector is vecdraw(v(0, 0, 0), swap_yz(list_to_vector(x, 0)):normalized, rgba(1, 0, 0, 0.5), "Lambda", 10.0, true, 0.11, true, true).
			global lambda_dot_vector is vecdraw(v(0, 0, 0), swap_yz(list_to_vector(x, 3)):normalized, rgba(0, 1, 0, 0.5), "Lambda Dot", 10.0, true, 0.11, true, true).
		}

		global lambda_vector_new  is vecdraw(v(0, 0, 0), swap_yz(list_to_vector(x_new, 0)):normalized, rgba(0.5, 0.5, 0, 0.5), "", 11.0, true, 0.10, true, true).
		global lambda_dot_vector_new is vecdraw(v(0, 0, 0), swap_yz(list_to_vector(x_new, 3)):normalized, rgba(0.5, 0, 0.5, 0.5), "", 11.0, true, 0.10, true, true).
	}

	// print max error
	if kgs_settings:debug_level = 2 {
		if iteration = 1 {
			for i in range(6) {
				print_string("", 0, debug_line + i, 40).
			}
		}
		print_string("iteration: " + iteration +  "  max error: " + format_sci(fmax, 4), 0, debug_line + mod(iteration - 1, 6), 40, false).
	}

	// print numerical and analytic jacobian and their ratio
	if kgs_settings:debug_level > 3 {

		local fdf_num is numerical_jacobian(jacobian_function@, x, p).

		print_string("analytical jacobian", 0, debug_line, 130, false).
		set debug_line to debug_line + 1.
		for i in range(fdf:length) {
			local row_string is "c" + (i + 1) + ":".
			for j in range(fdf[0]:length - 1) {
				set row_string to row_string + "  " + ("" + fdf[i][j]):padleft(10):substring(0, 10).
			}
			print_string(row_string, 0, debug_line, 130, false).
			set debug_line to debug_line + 1.
		}
		print_string("numerical jacobian", 0, debug_line, 130, false).
		set debug_line to debug_line + 1.
		for i in range(fdf_num:length) {
			local row_string is "c" + (i + 1) + ":".
			for j in range(fdf_num[0]:length - 1) {
				set row_string to row_string + "  " + ("" + fdf_num[i][j]):padleft(10):substring(0, 10).
			}
			print_string(row_string, 0, debug_line, 130, false).
			set debug_line to debug_line + 1.
		}
		print_string("ratio", 0, debug_line, 130, false).
		set debug_line to debug_line + 1.
		for i in range(fdf_num:length) {
			local row_string is "c" + (i + 1) + ":".
			for j in range(fdf_num[0]:length - 1) {
				set row_string to row_string + "  " + ("" + (fdf[i][j] / fdf_num[i][j])):padleft(10):substring(0, 10).
			}
			print_string(row_string, 0, debug_line, 130, false).
			set debug_line to debug_line + 1.
		}

		set kgs_settings:debug_level to 3.
		wait 5.
		set debug_line to 34.
		for i in range(3 * (fdf:length + 1)) {
			print "":padleft(130) at (0, debug_line + 1).
		}
	}

	// print x values, max error, and error
	if kgs_settings:debug_level > 2 {

		if iteration = 1 {
			local string is "x   :".
			for i in range(x:length) {
				set string to string + " " + ("" + x[i]):padleft(5):substring(0, 5).
			}
			set string to string + "  f: " + ("" + format_sci(fmax)):padleft(9):substring(0, 9).
			for i in range(fdf:length) {
				set string to string + " " + ("" + format_sci(fdf[i][7])):padleft(9):substring(0, 9).
			}
			print string at (0, debug_line).
			for i in range(20) {
				print "":padleft(130) at (0, debug_line + i + 1).
			}
		}

		local string is "xnew:".
		for i in range(x:length) {
			set string to string + " " + ("" + x_new[i]):padleft(5):substring(0, 5).
		}
		set string to string + "  f: " + ("" + format_sci(fmax)):padleft(9):substring(0, 9).
		for i in range(fdf:length) {
			set string to string + " " + ("" + format_sci(fdf[i][7])):padleft(9):substring(0, 9).
		}

		print string at (0, debug_line + 1 + mod(iteration - 1, 20)).
	}
}

// print final information and exit
function exit_kgs {
	clearvecdraws().
	unlock steering.
	unlock throttle.
    print "Kerbal Guidance System is now exiting.".
    print "have a nice day!".
}

