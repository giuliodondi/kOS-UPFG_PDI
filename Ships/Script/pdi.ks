@LAZYGLOBAL OFF.
clearscreen.

//Settings

GLOBAL vesselfilename is "Apollo LEM".     //this is the name of the vessel file to load


LOCAL tgtlat IS  4.9524.
LOCAL tgtlong IS  21.3164.

//apollo 11 - short
//LOCAL tgtlat IS  0.677575.
//LOCAL tgtlong IS  23.479351.


gLOBAL landing_state IS LEXICON(
						"tgtsite", LATLNG(tgtlat, tgtlong),
						"altitude",60,
						"velocity",1,
						"angle",-90,
						"altitude_bias", 1000,
						"range_bias",4000,
						"velocity_bias",0
						
).


GLOBAL log_data Is false.

GLOBAL debug Is true.

RUNPATH("0:/UPFG_pdi/src/pdi_main").
