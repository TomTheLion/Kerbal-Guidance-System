global kgs_inputs is lexicon(
	"objective", lexicon(
		"altitude", 80000
	),
	"open_loop_guidance", lexicon(
		"booster_name", "Simple Booster",
		"pitch_time", 8,
		"pitch_aoa", 5,
		"pitch_duration", 30
	),
	"timed_events", list(
		lexicon("time", 0, "type", "Launch", "description", "Lift Off!"),
		lexicon("time", 59, "type", "Stage", "description", "Upper Stage!", "rename", "Simple Upper Stage"),
		lexicon("time", 60, "type", "Activate Guidance", "description", "Activating Guidance!"),
		lexicon("time", 110, "type", "Coast", "duration", 110, "description", "Coast to Apo!")
	),
	"vehicle", list(
		lexicon(
			"name", "Simple Upper Stage",
			"mass_total", 3765,
			"mass_dry", 1765,
			"isp", 345,
			"thrust", 60000
		)
	)
).

clearscreen.
runpath("0:/kgs/kgs_main").