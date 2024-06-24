@LAZYGLOBAL OFF.

//tgt_body needs to be defined outside

clearscreen.
close_all_GUIs().

initialise_sites_file().

GLOBAL tgtsite is get_default_site().

fix_sites_body_position().

make_target_editor_gui().

pdi_target_editor_main().


close_all_GUIs().

clearvecdraws().
clearscreen.




function pdi_target_editor_main {

	GLOBAL quit_program IS FALSE.


	until false {
		clearscreen.
		if (quit_program) {
			return.
		}
		
		update_target_editor_gui().
		
		WAIT 0.3.
	}
}


//necessary as the program can be called while the parent ship body is still earth
function fix_sites_body_position {
	
	for key_ in pdi_siteslex:keys {
	
		local s is pdi_siteslex[key_].
		
		local sp is s["position"].
		
		local body_pos is tgt_body:GEOPOSITIONLATLNG(sp:LAT, sp:LNG).
		
		set pdi_siteslex[key_] to LEXICON("name", key_, "position", body_pos).
	}
}


