global kgs_inputs is lexicon(
	"objective", lexicon(
		"altitude", 200000,
		"inclination", 45,
		"lan", 45,
		"payload", 37500
	),
	"open_loop_guidance", lexicon(
		"booster_name", "RSS Example Booster",
		"pitch_time", 50,
		"pitch_aoa", 3,
		"pitch_duration", 40,
		"roll_time", 20,
		"azimuth_adjustment", 6
	),
	"timed_events", list(
		lexicon("time", 0, "type", "Launch", "description", "Lift Off!"),
		lexicon("time", 124, "type", "Stage", "description", "Jettison SRBs", "rename", "RSS Example Core Booster"),
		lexicon("time", 126, "type", "Activate Guidance", "description", "Activating Guidance!"),
		lexicon("time", 5 * 60, "type", "Jettison", "mass", 3000, "description", "Jettison Fairing!")
	),
	"vehicle", list(
		lexicon(
			"name", "RSS Example Core Booster",
			"mass_total", 766777,
			"mass_dry", 254777,
			"isp", 453.6,
			"thrust", 5997130,
			"staging_sequence", list(2, 2)	
		),
		lexicon(
			"name", "RSS Example Upper Stage",
			"mass_total", 158139,
			"mass_dry", 30139,
			"isp", 489.6,
			"thrust", 1571467
		)
	)
).

clearscreen.
runpath("0:/KGS/kgs_main").