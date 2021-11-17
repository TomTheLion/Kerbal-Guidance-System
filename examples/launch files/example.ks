global kgs_inputs is lexicon(
	"objective", lexicon(
		"altitude", 80000,
		"apoapsis", 85000,
		"periapsis", 75000,
		"inclination", 10,
		"lan", 230,
		"load_initial_guidance", "0:/examples/guidance/example.json"
	),
	"open_loop_guidance", lexicon(
		"pitch_time", 20,
		"pitch_aoa", 7,
		"pitch_rate", 0.75,
		"pitch_duration", 30,
		"roll_time", 5,
		"azimuth_adjustment", -1.5
	),
	"timed_events", list(
		lexicon("time", 0, "type", "Launch", "description", "Lift Off!"),
		lexicon("time", 20, "type", "throttle", "setting", 0.60, "description", "Throttle to 60%"),
		lexicon("time", 60, "type", "Activate Guidance", "description", "Activating Guidance!"),
		lexicon("time", 100, "type", "Coast", "duration", 90, "description", "Coast to Apo!"),
		lexicon("time", 130, "type", "jettison", "mass", 174, "additional_stages", 0, "description", "Jettison Fairing!"),
		lexicon("time", 135, "type", "action group", "group", 1, "description", "Deploy Solar Panels"),
		lexicon("time", 140, "type", "action group", "group", 2, "description", "Deploy Antenna")
		
	),
	"vehicle", list(
		lexicon(
			"mass_total", 8197,
			"mass_dry", 4197,
			"isp", 320,
			"thrust", 215000,
			"g_limit", 2.5,
			"staging_sequence", list(1, 1)
		),
		lexicon(
			"mass_total", 2117,
			"mass_dry", 1617,
			"isp", 345,
			"thrust", 60000,
			"g_limit", 2.5,
			"staging_sequence", list(1, 1)
			
		),
		lexicon(
			"mass_total", 796,
			"mass_dry", 596,
			"isp", 320,
			"thrust", 20000,
			"g_limit", 1
		)
	)
).

clearscreen.
runpath("0:/kgs/kgs_main").