@LAZYGLOBAL OFF.
clearscreen.

parameter desired_body is "moon".

GLOBAL tgt_body is BODY(desired_body).

//Settings

GLOBAL vesselfilename is "Apollo LEM".     //this is the name of the vessel file to load


LOCAL tgtlat IS  4.296.
LOCAL tgtlong IS  19.992.

//apollo 11 - short
//LOCAL tgtlat IS  0.677575.
//LOCAL tgtlong IS  23.479351.



RUNPATH("0:/UPFG_pdi/src/pdi_main_executive").
