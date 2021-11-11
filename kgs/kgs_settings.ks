@lazyglobal off.

// global settings for kgs
global kgs_settings is lexicon(
    "ipu", 2000,						// increase kOS operation speed
	"integrator", rk4@,				// rk4 recommended (available: rk2, rk3, rk4, rkf45)
    "max_step", 0.25,					// max time step used for integration recommended 0.25 for rk4 (lower for rk2 or rk3)
	"eps_newton", 1e-5,					// error tolerance for newton's method (1e-5 recommened)
	"eps_rkf45", 1e-4,					// error tolerance if using adjustable time step rkf45 integrator
	"ui_refresh_rate", 0.25,			// interval beween ui updates, higher number will use less cpu time (0.25 @ 500 ipu takes ~25% of cpu time)
	"guidance_refresh_rate", 0.25,		// interval beween ship attitude updates, higher number will use less cpu time
	"debug_level", 2					// level 1 shows guidance vectors, level 2 shows basic iteration info, level 3 shows detailed interation info, level 4 shows jacobian info
).