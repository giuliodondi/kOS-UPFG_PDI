@LAZYGLOBAL OFF.
GLOBAL guitextwhite IS RGB(227/255,227/255,227/255).

GLOBAL target_editor_gui_width IS 300.
GLOBAL target_editor_gui_height IS 320.


FUNCTION make_target_editor_gui {
	//create the GUI.
	GLOBAL target_editor_gui is gui(target_editor_gui_width,target_editor_gui_height).
	SET target_editor_gui:X TO 200.
	SET target_editor_gui:Y TO 670.
	SET target_editor_gui:STYLe:WIDTH TO target_editor_gui_width.
	SET target_editor_gui:STYLe:HEIGHT TO target_editor_gui_height.
	SET target_editor_gui:STYLE:ALIGN TO "center".
	SET target_editor_gui:STYLE:HSTRETCH TO TRUE.

	set target_editor_gui:skin:LABEL:TEXTCOLOR to guitextwhite.
	
	// Add widgets to the GUI
	GLOBAL title_box is target_editor_gui:addhbox().
	set title_box:style:height to 60. 
	set title_box:style:margin:top to 0.


	GLOBAL text0 IS title_box:ADDLABEL("<b><size=20>PDI TARGET EDITOR</size></b>").
	SET text0:STYLE:ALIGN TO "center".


	
	GLOBAL quitb IS  title_box:ADDBUTTON("X").
	set quitb:style:margin:h to 7.
	set quitb:style:margin:v to 7.
	set quitb:style:width to 20.
	set quitb:style:height to 20.
	function quitcheck {
	  SET quit_program TO TRUE.
	}
	SET quitb:ONCLICK TO quitcheck@.
	
	
	GLOBAL togglesbox IS target_editor_gui:ADDVLAYOUT().
	SET togglesbox:STYLE:WIDTH TO 200.	
	SET togglesbox:STYLE:HEIGHT TO 60.	
	SET togglesbox:STYLE:margin:h TO 50.	

	GLOBAL select_sitebox IS togglesbox:ADDHLAYOUT().
	GLOBAL site_label IS select_sitebox:ADDLABEL("<size=15>Site : </size>").
	GLOBAL select_site IS select_sitebox:addpopupmenu().
	SET select_site:STYLE:WIDTH TO 120.
	SET select_site:STYLE:HEIGHT TO 25.
	SET select_site:STYLE:ALIGN TO "center".
	update_sites_menu().
		
	set select_site:index to -1.
	SET select_site:ONCHANGE to { 
		PARAMETER lex_key.	
		
		if (moonsiteslex:haskey(lex_key)) {
			set tgtsite to moonsiteslex[lex_key].
		} else {
			set tgtsite to get_default_moonsite().
		}
		
		set_gui_boxes().
	}.
	
	GLOBAL buttons_box IS togglesbox:ADDHLAYOUT().
	
	GLOBAL addsiteb is buttons_box:ADDBUTTON("ADD NEW").
	SET addsiteb:STYLE:WIDTH TO 55.
	SET addsiteb:STYLE:HEIGHT TO 25.
	
	set addsiteb:onclick to {
	
		local newsite is make_site_from_gui().
		
		if (moonsiteslex:haskey(newsite["name"])) {
			set moonsiteslex[newsite["name"]] to newsite.
		} else {
			moonsiteslex:add(newsite["name"], newsite).
		}
		
		update_sites_menu().
		
		local siteslist is select_site:options.
		set select_site:index to siteslist:indexof(newsite["name"]).
		
		set_gui_boxes().
	
	}.
	
	
	GLOBAL deletesiteb is buttons_box:ADDBUTTON("DELETE").
	SET deletesiteb:STYLE:WIDTH TO 55.
	SET deletesiteb:STYLE:HEIGHT TO 25.

	set deletesiteb:onclick to {
		
		if (moonsiteslex:haskey(tgtsite["name"])) {
			moonsiteslex:remove(tgtsite["name"]).
		}
		
		update_sites_menu().
		local siteslist is select_site:options.
		set select_site:index to siteslist:length - 1.
		
		set_gui_boxes().
	}.
	
	
	
	GLOBAL all_box IS target_editor_gui:ADDVLAYOUT().
	SET all_box:STYLE:WIDTH TO target_editor_gui_width.
	SET all_box:STYLE:HEIGHT TO 160.
	
	GLOBAL sitedata_textlabel IS all_box:ADDLABEL("<b>Site Data</b>").	
	SET sitedata_textlabel:STYLE:ALIGN TO "center".
	set sitedata_textlabel:style:margin:v to 5.
	
	GLOBAL site_databox IS all_box:ADDVBOX().
	SET site_databox:STYLE:ALIGN TO "center".
	SET site_databox:STYLE:WIDTH TO 230.
    SET site_databox:STYLE:HEIGHT TO 140.
	set site_databox:style:margin:h to 28.
	set site_databox:style:margin:v to 0.
	
	GLOBAL site_name_box IS site_databox:addhlayout().
	SET site_name_box:STYLE:ALIGN TO "center".
	GLOBAL site_name_text IS site_name_box:addlabel("Name : ").
	GLOBAL site_name is site_name_box:addtextfield("").
	set site_name:style:width to 110.
	set site_name:style:height to 22.
		
	set site_name:onconfirm to { 
		parameter val.
		
		SET tgtsite["name"] to val.
	}.
	
	GLOBAL site_lat_box IS site_databox:addhlayout().
	SET site_lat_box:STYLE:ALIGN TO "center".
	GLOBAL site_lat_text IS site_lat_box:addlabel("Latitude : ").
	GLOBAL site_lat is site_lat_box:addtextfield(0:tostring).
	set site_lat:style:width to 110.
	set site_lat:style:height to 22.
		
	set site_lat:onconfirm to { 
		parameter val.
		
		local sitepos is tgtsite["position"].
		local newlat is val:tonumber(0).
		set tgtsite["position"] to tgt_body:GEOPOSITIONLATLNG(newlat, sitepos:lng).
	}.
	
	GLOBAL site_lng_box IS site_databox:addhlayout().
	SET site_lng_box:STYLE:ALIGN TO "center".
	GLOBAL site_lng_text IS site_lng_box:addlabel("Longitude : ").
	GLOBAL site_lng is site_lng_box:addtextfield(0:tostring).
	set site_lng:style:width to 110.
	set site_lng:style:height to 22.
		
	set site_lng:onconfirm to { 
		parameter val.
		
		local sitepos is tgtsite["position"].
		local newlng is val:tonumber(0).
		set tgtsite["position"] to tgt_body:GEOPOSITIONLATLNG(sitepos:lat, newlng).
	}.
	
	GLOBAL site_elev_box IS site_databox:addhlayout().
	SET site_elev_box:STYLE:ALIGN TO "center".
	GLOBAL site_elev_text1 IS site_elev_box:addlabel("Elevation : ").
	set site_elev_text1:style:width to 110.
	
	GLOBAL site_elev_text2 IS site_elev_box:addlabel("xxx").
	set site_elev_text2:style:width to 50.
	
	set_gui_boxes().
	
	target_editor_gui:show.
	
	
}

function update_sites_menu {
	select_site:clear().
	
	FOR site IN moonsiteslex:KEYS {
		select_site:addoption(site).
	}
}

function set_gui_boxes {

	set site_name:text to tgtsite["name"].
	
	local sitepos is tgtsite["position"].
	
	set site_lat:text to sitepos:lat:tostring.
	set site_lng:text to sitepos:lng:tostring.
}



function update_target_editor_gui{
	clearvecdraws().
	
	local sitepos_body is tgtsite["position"]:position - tgt_body:position.
	arrow_foreignbody(tgt_body, sitepos_body, "site").
	
	set site_elev_text2:text to ROUND(tgtsite["position"]:terrainheight, 0) + " m".

}

function get_default_site {
	return LEXICON(
				"name", "site_name",
				"position",tgt_body:GEOPOSITIONLATLNG(0, 0)
	).
} 

function make_site_from_gui {
	local newname is tgtsite["name"].
	
	local sitepos is tgtsite["position"].
	
	local newlat is sitepos:lat.
	local newlng is sitepos:lng.

	return LEXICON(
				"name", newname,
				"position",tgt_body:GEOPOSITIONLATLNG(newlat, newlng)
	).
} 



FUNCTION close_all_GUIs{
	CLEARGUIS().
	if (defined(target_editor_gui)) {
		target_editor_gui:DISPOSE().
	}
}