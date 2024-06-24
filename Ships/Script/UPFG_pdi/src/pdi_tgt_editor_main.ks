clearscreen.
close_all_GUIs().

GLOBAL tgt_body is BODY("moon").
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
		
		print moonsiteslex.
		
		WAIT 0.3.

	}
}


//functions


//necessary as the program can be called while the parent ship body is still earth
function fix_sites_body_position {
	
	for key_ in moonsiteslex:keys {
	
		local s is moonsiteslex[key_].
		
		local sp is s["position"].
		
		local body_pos is tgt_body:GEOPOSITIONLATLNG(sp:LAT, sp:LNG).
		
		set moonsiteslex[key_] to LEXICON("name", key_, "position", body_pos).
	}
}


