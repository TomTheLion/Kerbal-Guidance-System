@lazyglobal off.

//
// user interface functions
//

// print static ui elements, set ui size, and set ui trigger
function create_ui {
	
	clearscreen.

	// set terminal size, increase size based on debug level
	if kgs_settings:debug_level > 2 {
		set terminal:width to 130.
		set terminal:height to 60.
	}
	else if kgs_settings:debug_level = 2 {
		set terminal:width to 50.
		set terminal:height to 40.
	}
	else {
		set terminal:width to 50.
		set terminal:height to 35.
	}

	// print base ui
	print ".------------------------------------------------.".
	print "| Kerbal Guidance System                    v1.0 |".
	print "|------------------------------------------------|".
	print "| VEHICLE STATUS                                 |".
	print "| Mode     :                Mass   :             |".
	print "| Tgo      :                Thrust :             |".
	print "| Throttle :                Accel  :             |".
	print "|------------------------------------------------|".
	print "| STAGE STATUS                                   |".
	print "| Current stage :                                |".
	print "| Next event    :                                |".
	print "| Event type    :                                |".
	print "| Tgo           :                                |".
	print "|------------------------------------------------|".
	print "| TRAJECTORY STATUS                              |".
	print "| Paramter      Current      Target       Error  |".
	print "| Altitude    |            |            |        |".
	print "| Velocity    |            |            |        |".
	print "| FP angle    |            |            |        |".
	print "| Inclination |            |            |        |".
	print "| LAN         |            |            |        |".
	print "|------------------------------------------------|".
	print "| GUIDANCE STATUS                                |".
	print "| No. guidance stages  :                         |".
	print "| Pred. final mass     :                         |".	
	print "| Last loop time       :                         |".
	print "| Last loop error      :                         |".
	print "| Last iteration count :                         |".
	print "'------------------------------------------------'".
	
	// print objectives
	print_string(format_number(kgs_inputs:objective:altitude), 29, 16, 10).
	print_string(format_number(sqrt(kgs_data:clg:velocity_constraint) * kgs_data:scale:velocity), 29, 17, 10).
	print_string(format_number(0), 29, 18, 10).
	
	if kgs_data:clg:mode > 0 {
		print_string(format_number(kgs_inputs:objective:inclination), 29, 19, 10).
	}
	else {
		print_string("N/A", 29, 19, 10).
		print_string("N/A", 42, 19, 6).
	}
	
	if kgs_data:clg:mode > 1 {
		print_string(format_number(kgs_inputs:objective:lan), 29, 20, 10).
	}
	else {
		print_string("N/A", 29, 20, 10).
		print_string("N/A", 42, 20, 6).
	}
	
	// print guidance status
	print_string("N/A", 25, 23, 10).
	print_string("N/A", 25, 24, 10).
	print_string("N/A", 25, 25, 10).
	print_string("N/A", 25, 26, 10).
	print_string("N/A", 25, 27, 10).

	// set ui trigger
	refresh_ui().
	set_ui_trigger().
}

// set ui to refresh based on refresh rate scaled by timewarp rate
function set_ui_trigger {
	local t0 is time:seconds.
	when time:seconds > t0 + kgs_settings:ui_refresh_rate * kuniverse:timewarp:rate then {
		if not kgs_data:clg:stop_ui {
			refresh_ui().
			set_ui_trigger().
		}
	}
}

// refresh dynamic ui elements
function refresh_ui {

	// get and print current state
	local r_ship is -body:position.
	local a_ship is r_ship:mag - body:radius.
	local v_ship is ship:orbit:velocity:orbit.
	local flight_path_angle is safe_arccos(vcrs(r_ship, v_ship):mag / r_ship:mag / v_ship:mag).

	print_string(format_number(a_ship), 16, 16, 10).
	print_string(format_number(v_ship:mag), 16, 17, 10).
	print_string(format_number(flight_path_angle), 16, 18, 10).
	print_string(format_number(ship:orbit:inclination), 16, 19, 10).
	print_string(format_number(ship:orbit:lan), 16, 20, 10).
	
	// calculate and print errors
	local altitude_error is abs(a_ship - kgs_inputs:objective:altitude) / kgs_inputs:objective:altitude.
	local velocity_error is abs(v_ship:mag / kgs_data:scale:velocity - sqrt(kgs_data:clg:velocity_constraint)).
	local flight_path_angle_error is abs(flight_path_angle) / 90.
	
	print_string(format_percent(altitude_error), 42, 16, 6).
	print_string(format_percent(velocity_error), 42, 17, 6).
	print_string(format_percent(flight_path_angle_error), 42, 18, 6).

	if kgs_data:clg:mode > 0 {
		local inclination_error is abs(ship:orbit:inclination - kgs_inputs:objective:inclination) / 180.
		print_string(format_percent(inclination_error), 42, 19, 6).
	}
	
	if kgs_data:clg:mode > 1 {
		local lan_error is abs(ship:orbit:lan - kgs_inputs:objective:lan) / 360.
		print_string(format_percent(lan_error), 42, 20, 6).
	}

	// print throttle, thrust, mass, and acceleration
	print_string(format_percent(throttle), 13, 6, 11	).
	print_string(format_number(ship:mass * 1000, 0), 37, 4, 10).
	print_string(format_number(ship:availablethrust * throttle), 37, 5, 10).
	print_string(format_number(ship:availablethrust * throttle / ship:mass), 37, 6, 10).

	// print guidance information
	if kgs_data:clg:active_guidance {

		// closed loop guidance information
		local tgo is get_tgo().
		print_string(format_number(tgo, 1), 13, 5, 11).

		// print stage status
		print_string(kgs_data:stages[0]:name, 18, 9, 30, false).
		
		// print next event information
		local tgo_clg is kgs_data:ui:clg_event_time - time:seconds.
		local tgo_olg is kgs_data:ui:olg_event_time - time:seconds.

		local clg_event_flag is false.
		local olg_event_flag is false.

		if kgs_data:stages:length > 1 and tgo_clg < tgo {
			set clg_event_flag to true.
		}

		if not olg_event_ptr:atend and tgo_olg < tgo {
			set olg_event_flag to true.
		}

		if clg_event_flag and olg_event_flag and tgo_clg > tgo_olg {
			set clg_event_flag to false.
		}

		if clg_event_flag {
			print_string(kgs_data:ui:clg_event_name, 18, 10, 30, false).
			print_string("CLG event", 18, 11, 30, false).
			print_string(format_number(tgo_clg, 1), 18, 12, 30, false).
		}
		else if olg_event_flag {
			print_string(kgs_data:ui:olg_event_name, 18, 10, 30, false).
			print_string(kgs_data:ui:olg_event_type, 18, 11, 30, false).
			print_string(format_number(tgo_olg, 1), 18, 12, 30, false).
		}
		else {
			print_string("N/A", 18, 10, 30, false).
			print_string("N/A", 18, 11, 30, false).
			print_string(format_number(tgo, 1), 18, 12, 30, false).
		}
		
		// print guidance status
		if kgs_data:clg:terminal_guidance {
			print_string(" ", 18, 22, 10, false).
			print_string("terminal", 13, 4, 11).
			print_string("N/A", 25, 25, 10).
			print_string("N/A", 25, 26, 10).
			print_string("N/A", 25, 27, 10).
		}
		else {
			print_string("closed loop", 13, 4, 11).
			if kgs_data:clg:first_pass {
				print_string(kgs_data:ui:status_icon[mod(kgs_data:ui:status_index, kgs_data:ui:status_icon:length)], 18, 22, 10, false).
				print_string(format_number(kgs_data:stages:length, 0), 25, 23, 10).
				print_string(format_number(kgs_data:clg:final_mass, 0), 25, 24, 10).
				print_string(format_number(kgs_data:clg:time - kgs_data:clg:time_prev), 25, 25, 10).
				print_string(format_sci(kgs_data:clg:last_err, 2), 25, 26, 10).
				print_string(format_number(kgs_data:clg:last_iter_count, 0), 25, 27, 10).
			}
		}
	}
	else {
		// print open loop guidance information
		print_string(kgs_data:olg:booster_name, 18, 9, 30, false).
		print_string("open loop", 13, 4, 11).
		print_string("N/A", 13, 5, 11).
		print_string(kgs_data:ui:olg_event_name, 18, 10, 30, false).
		print_string(kgs_data:ui:olg_event_type, 18, 11, 30, false).
		print_string(format_number(kgs_data:ui:olg_event_time - time:seconds, 1), 18, 12, 30, false).
	}
}