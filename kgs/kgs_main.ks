@lazyglobal off.

// set kOS directory location
global kgs_dir is "0:/kgs".

// import settings and libraries
runoncepath(kgs_dir + "/lib/ksmanip").
runoncepath(kgs_dir + "/lib/ksmath.ks").
runoncepath(kgs_dir + "/kgs_settings.ks").
runoncepath(kgs_dir + "/lib/kgs_lib.ks").
runoncepath(kgs_dir + "/lib/kgs_ui.ks").
runoncepath(kgs_dir + "/lib/kgs_guidance.ks").

// set kOS ipu
set config:ipu to kgs_settings:ipu.

// initialize global parameters
initialize().

// create user interface
create_ui().

// warp to launch time
warp_to_launch_time().

// main guidance loop
until abort or kgs_data:clg:stop_guidance {
    guidance().	
    wait 0.
}

// wait for ship to settle, then exit
until abort or settle() {
	wait 0.
}
wait 2.
exit_kgs().