//	Load libraries
RUNPATH("0:/Libraries/misc_library").	
RUNPATH("0:/Libraries/maths_library").	
clearscreen.

parameter desired_body is "moon".

GLOBAL tgt_body is BODY(desired_body).

RUNPATH("0:/Libraries/navigation_library").	
RUNPATH("0:/Libraries/vehicle_library").	

RUNPATH("0:/UPFG_pdi/src/pdi_interface_library").
RUNPATH("0:/UPFG_pdi/src/pdi_targeting_library").
RUNPATH("0:/UPFG_pdi/src/pdi_upfg_library").
RUNPATH("0:/UPFG_pdi/src/pdi_vehicle_library").
RUNPATH("0:/UPFG_pdi/src/pdi_gui_library").

initialise_sites_file().

make_main_pdi_gui().

//make_pdi_braking_display().

until false {
	clearscreen.
	if (quit_program) {
		break.
	}
	
	print landing_state.
	
	
	wait 0.1.

}


close_all_GUIs().

clearvecdraws().
clearscreen.