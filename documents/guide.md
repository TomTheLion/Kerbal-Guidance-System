# KGS Guide

KGS requires two lexicons to be defined, kgs_settings and kgs_inputs, prior to calling the main function.

## Convergence

## Coast Phases

## KGS_Settings

The KGS_Settings lexicon provides several options that affect the operation of the algorithm:

Key                   | Recommended Value | Description
---                   | ---               | ---
ipu                   |              2000 | controls the speed of the kOS processor, the code will run better the higher ipu you use
integrator            |               rk4 | integration method for the trajectory 4 runge kutta methods are available (rk2, rk3, rk4, rkf45)
max_step              |              0.25 | limits the maximum step size the integrator will take, lower order integrators will require a lower max step
eps_newton            |              1e-5 | error tolerance for the Newton's method algorithm, higher numbers will run faster but accept worse solutions
ui_refresh_rate       |              0.25 | controls how often new ui information is displayed, lower numbers will require more cpu time
guidance_refresh_rate |              0.25 | controls how often the ship attitude is adjusted
debug_level           |                 0 | provides debug information: 0: off, 1: shows guidance vectors, 2: shows basic iteration info, 3: shows detailed interation info, 4: shows jacobian info
eps_rkf45             |              1e-5 | error tolerance if using adjustable time step rkf45 integrator

## KGS_Inputs

the KGS_Inputs lexicon is where the mission and vehicle parameters are defined, it is broken up into 2 sub-lexicons and 2 sub-lists:

## KGS_Inputs:Objective

The objective defines the desired orbit and payload and also provides the option to save and load initial guidance solutions.

Key                | Required? | Description
---                |---        | ---
altitude           | yes       | desired altitude at burn out
apoapsis           | no        | desired orbit apoapsis (if not set altitude will be used)
periapsis          | no        | desired orbit periapsis (if not set altitude will be used)
inclination        | no        | desired inclination
lan                | no        | desired longitude of the ascending node
payload            | no        | additional mass to add to each stage, useful for launch vehicles designed for different payloads
save_initial_guidance      | no        | if specified the converged guidance solution will be saved
load_initial_guidance      | no        | if a previous converged guidance solution was saved, it will be loaded

## KGS_Inputs:Open_loop_guidance

The open loop guidance parameters determine how the atmopsheric portion of the trajectory will be flown and allow for optimization of the launch time and azimuth.

Key                    | Required? | Description
---                    |---        | ---
pitch_time             | yes       | time to begin pitch over manuever
pitch_aoa              | yes       | maximum desired angle of attack during pitch over manuever
pitch_duration         | yes       | duration of pitch over manuever
pitch_rate             | no        | maximum angle rate for pitch over maneuver, defaul is 1 deg/sec
roll_time              | no        | time to begin roll maneuver
roll_angle             | no        | desired roll angle, defualt will align ship with launch azimuth
booster_name           | no        | desired booster display name
throttle               | no        | desired initial throttle
azimuth_adjustment     | no        | value that will be added to calculated launch azimuth for optimization
launch_direction       | no        | desired launch direction, options are north or south, default value is soonest
launch_time_adjustment | no        | value that will be added to calculated launch time for optimization
launch_countdown       | no        | how long to wait before lift off or if lan is set how long before lift off to stop time warp

## KGS_Inputs:Timed_staging

The timed staging parameters control predefined events, event types include:

Key               | Required? | Description
---               |---        | ---
launch            | yes       | stages vehicle (identical to stage event)
activate guidance | yes       | initializes guidance algorithm, this event should occur when through the thickest portion of the atmopshere
stage             | no        | stages vehicle
throttle          | no        | changes throttle value
action group      | no        | sets speicifed action group to on
jettison          | no        | stages vehicle and adjustes guidance algorithm for mass removed
coast             | no        | adds a timed stage with zero thrust

Key          | Required? | Description
---          |---        | ---
time         | yes       | time for event to occur
type         | yes       | type of event
description  | yes       | duration of pitch over manuever
setting      | if type = throttle | new throttle setting 
mass         | if type = jettison | mass to be jettisoned
duration     | if type = coast | duration of coast phase
group        | if type = action group | action group to set to on
rename       | no        | changes display name of booster

## KGS_Inputs:Vehicle



