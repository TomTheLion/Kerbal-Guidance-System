@lazyglobal off.

// global settings for kgs
global kgs_settings is lexicon(
	"ipu", 2000,					// controls the speed of the kOS processor
	"integrator", rk4@,				// integration method for the trajectory, 4 runge kutta methods are available (rk2, rk3, rk4, rkf45)
	"max_step", 0.25,				// limits the maximum step size the integrator will take
	"eps_newton", 1e-5,				// error tolerance for the Newton's method algorithm, higher numbers will run faster but accept worse solutions
	"ui_refresh_rate", 0.25,		// controls how often new ui information is displayed, lower numbers will require more cpu time
	"guidance_refresh_rate", 0.25,	// controls how often the ship attitude is adjusted
	"debug_level", 0,				// provides debug information: 0: off, 1: shows guidance vectors, 2: shows basic iteration info, 3: shows detailed interation info, 4: shows jacobian info
	"eps_rkf45", 1e-4				// error tolerance if using adjustable time step rkf45 integrator
).