# Kerbal-Guidance-System

Kerbal Guidance System or KGS is a program written for Kerbal Space Program in kerboscript that can guide a launch vehicle on an optimal trajectory to a desired orbit while maximizing payload. KGS offers the specification of three types of orbits:

* Unconstrainted orbital plane
* Constrained inclination
* Constrained inclination and longitude of the ascending node

KGS allows for the specification of various parameters to define the trajectory including:

* Burn out altitude
* Apoapsis
* Periapsis
* Inclination
* Longitude of the ascending node
* Payload
* Coast phases
* Jettison events
* Action group triggers
* Ullage

KGS can estimate optimal launch time and launch azimuth then fly a simple trajectory during the atmospheric phase of ascent. KGS provides several options to optimize the atmospheric ascent phase including:

* Pitch over time
* Pitch over angle
* Pitch over angle rate
* Roll time
* Roll angle
* Launch azimuth adjustment
* Launch time adjustment
* Throttle control

Please see the [guide](https://github.com/TomTheLion/Kerbal-Guidance-System/blob/main/documents/guide.md) and [examples](https://github.com/TomTheLion/Kerbal-Guidance-System/blob/main/examples/) to get a better understanding of how to use KGS.

If you are interested in understanding how the guidance algorithm works you can read about the theory [here](https://github.com/TomTheLion/Kerbal-Guidance-System/blob/main/documents/Optimal_Ascent_Guidance.pdf), though the math is quite difficult.

