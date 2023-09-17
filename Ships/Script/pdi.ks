@LAZYGLOBAL OFF.
clearscreen.

//Settings

GLOBAL vesselfilename is "Apollo LEM".     //this is the name of the vessel file to load


LOCAL tgtlat IS  4.296.
LOCAL tgtlong IS  19.992.

//apollo 11 - short
//LOCAL tgtlat IS  0.677575.
//LOCAL tgtlong IS  23.479351.


gLOBAL landing_state IS LEXICON(
						"tgtsite", LATLNG(tgtlat, tgtlong),
						"altitude",100,
						"velocity",1,
						"angle",-90,
						"altitude_bias", 0,
						"range_bias",1500,
						"velocity_bias",0,
						"tgt_redesig_t",100,
						"final_apch_t",45
						
).


GLOBAL log_data Is false.

GLOBAL debug Is false.

RUNPATH("0:/UPFG_pdi/src/pdi_main").
