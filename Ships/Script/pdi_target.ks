@LAZYGLOBAL OFF.

parameter desired_body is "moon".

GLOBAL tgt_body is BODY(desired_body).

CLEARSCREEN.
RUNPATH("0:/Libraries/misc_library").	
RUNPATH("0:/Libraries/navigation_library").
RUNPATH("0:/UPFG_pdi/src/pdi_gui_library.ks").


RUNPATH("0:/UPFG_pdi/src/pdi_tgt_editor_main.ks").