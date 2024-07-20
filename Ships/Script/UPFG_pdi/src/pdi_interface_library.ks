GLOBAL terminalwidth IS 64.
GLOBAL terminalheight IS 47.

SET TERMINAL:WIDTH TO terminalwidth.
SET TERMINAL:HEIGHT TO terminalheight.

GLOBAL P_vizMsg IS LIST().

//wrapper 
function addMessage {
	PARAMETER msg.
	
	addGUIMessage(msg).
}

FUNCTION addGUIMessage {
	PARAMETER msg.
	
	LOCAL clear_ is false.
	
	local t_msg is TIME:SECONDS - vehicle["ign_t"].
	
	local t_str IS "".
	
	if (t_msg >= 0) {
		SET t_str TO t_str + "+".
	}
	
	pdi_add_scroll_msg(
						t_str + sectotime(TIME:SECONDS - vehicle["ign_t"],"") + ": " + msg,
						clear_
	).

}

FUNCTION addMessage_terminal {
	DECLARE PARAMETER msg.
	LOCAL tt IS TIME:SECONDS.
	LOCAL ttl IS 4.

	local rem_list IS LIST().
	FROM {LOCAL k IS 0.} UNTIL k = P_vizMsg:LENGTH  STEP { SET k TO k+1.} DO{
		IF tt >= P_vizMsg[k]["ttl"] {
			rem_list:ADD(k).
		}
	}
	FROM {LOCAL k IS rem_list:LENGTH-1.} UNTIL k <0  STEP { SET k TO k-1.} DO{
		P_vizMsg:REMOVE(rem_list[k]).
	}
	
	P_vizMsg:INSERT(
					0,
					LEXICON(
							"msg","T+" + sectotime(TIME:SECONDS - vehicle["ign_t"],"") + ": " + msg,
							"ttl",tt + ttl
					)
	) .
}

FUNCTION drawUI {

	IF NOT (DEFINED surfloc) {GLOBAL surfloc IS 17.}
	IF NOT (DEFINED vehloc) {GLOBAL vehloc IS 25.}
	IF NOT (DEFINED msgloc) {GLOBAL msgloc IS 35.}
	

	CLEARSCREEN.
	
	PRINT "|=============================================================|"  AT (0,1).
	PRINT "|           UPFG POWERED DESCENT GUIDANCE AGORITHM            |"  AT (0,2).
	PRINT "|                                                             |"  AT (0,3).
	PRINT "|                                                             |"  AT (0,4).
	PRINT "|                                                             |"  AT (0,5).
	PRINT "|                                                             |"  AT (0,6).
	PRINT "|                                                             |"  AT (0,7).
	PRINT "|=============================================================|"  AT (0,8).
	PRINT "|                                                             |"  AT (0,9).
	PRINT "|         POWERED DESCENT IGNITION TIMER :                    |"  AT (0,10).
	PRINT "|                                                             |"  AT (0,11).
	PRINT "|                                                             |"  AT (0,12).
	PRINT "|                                                             |"  AT (0,13).
	PRINT "|=============================================================|"  AT (0,14).
	PRINT "|                  SURFACE AND LANDING DATA                   |" AT (0,15).
	PRINT "|                                                             |" AT (0,16).
	PRINT "|  LDG SITE ALT   :               LDG SITE RANGE :            |" AT (0,17).
	PRINT "|  VERTICAL SPD   :               HORIZ SPD      :            |" AT (0,18).
	PRINT "|  SURF PITCH     :               INERT AZIMUTH  :            |" AT (0,19).
	PRINT "|  CMD DES RATE   :                                           |" AT (0,20).
	PRINT "|                                                             |" AT (0,21).
	PRINT "|=============================================================|" AT (0,22).
	PRINT "|         VEHICLE DATA         |                              |" AT (0,23).
	PRINT "|                              |                              |" AT (0,24).
	PRINT "|   TOT BURN TIME :            |                              |" AT (0,25).
	PRINT "|   CURRENT TWR   :            |                              |" AT (0,26).
	PRINT "|   CMD THROTTLE  :            |                              |" AT (0,27).
	PRINT "|   CURRENT STG   :            |                              |" AT (0,28).
	PRINT "|   STG TYPE      :            |                              |" AT (0,29).
	PRINT "|   STG REM TIME  :            |                              |" AT (0,30).
	PRINT "|                              |                              |" AT (0,31).
	PRINT "|=============================================================|" AT (0,32).
	PRINT "|                         MESSAGE BOX:                        |" AT (0,33).
	PRINT "|                                                             |" AT (0,34).
	PRINT "|                                                             |" AT (0,35).
	PRINT "|                                                             |" AT (0,36).
	PRINT "|                                                             |" AT (0,37).
	PRINT "|                                                             |" AT (0,38).
	PRINT "|                                                             |" AT (0,39).
	PRINT "|                                                             |" AT (0,40).
	PRINT "|                                                             |" AT (0,41).
	PRINT "|                                                             |" AT (0,42).
	PRINT "|                                                             |" AT (0,43).
	PRINT "|                                                             |" AT (0,44).
	PRINT "|                                                             |" AT (0,45).
	PRINT "|=============================================================|" AT (0,46).
	
	//IF (vehiclestate["ops_mode"] =2) OR (vehiclestate["ops_mode"] =1) OR (vehiclestate["ops_mode"] =3) {
	IF vehiclestate["ops_mode"] <4 {
		PRINT "     UPFG GUIDANCE DATA       "  AT (32,vehloc-2).
		PRINT "    S_MODE     : "	AT (32,vehloc).
		PRINT "    STATUS     : "	AT (32,vehloc+1).
		PRINT "   UPFG STG    : "	AT (32,vehloc+2).
		PRINT "     T_GO      : " AT (32,vehloc+3).	
		PRINT "     V_GO      : " AT (32,vehloc+4).
		PRINT "   RANGE ERR   : " AT (32,vehloc+5).
	}
	ELSE {
		PRINT "     FINAL DESCENT DATA       " AT (32,vehloc-2).
		PRINT "    RADAR ALT    : "	AT (32,vehloc).
		PRINT "    HORIZ SPD    : "	AT (32,vehloc+1).
		PRINT "    TGT RATE     : "	AT (32,vehloc+2).
	}
}


//				outputs all flight information to the screen
//				requires flight sequence given by P_seq defined as GLOBAL and of type LIST
//				capable of displaying custom messages, set up by addMessage as a GLOBAL STRING with a FLOAT time to live
FUNCTION dataViz {

	PRINTPLACE("VEHICLE : " + vehicle["name"],61,1,4).	
	
	//PRINTING VARIABLES IN THE CORRECT PLACE
	
	IF vehiclestate["ops_mode"] >0 {
		PRINTPLACE(sectotime(TIME:SECONDS - vehicle["ign_t"]),10,41,10).
	}
	
	//Status
	LOCAL vehstatus IS "CURRENT STATUS : ".
	IF vehiclestate["ops_mode"] =0 { SET vehstatus TO vehstatus + "PRE-CONVERGENCE".}
	ELSE IF vehiclestate["ops_mode"] =1 { SET vehstatus TO vehstatus + "COASTING TO PDI".}
	ELSE IF vehiclestate["ops_mode"] =2 { SET vehstatus TO vehstatus + "POWERED DESCENT".}
	ELSE IF vehiclestate["ops_mode"] =3 { SET vehstatus TO vehstatus + "POWERED DESCENT".}
	ELSE IF vehiclestate["ops_mode"] =4 { SET vehstatus TO vehstatus + "PITCHOVER".}
	ELSE IF vehiclestate["ops_mode"] =5 { SET vehstatus TO vehstatus + "ATTITUDE HOLD".}
	ELSE IF vehiclestate["ops_mode"] =-1 { SET vehstatus TO vehstatus + "SHUTDOWN".}
	PRINTPLACE(vehstatus ,61,1,12).
	

	//surface data
	LOCAL site_alt IS ALT:RADAR.
	
	IF (vehiclestate["ops_mode"] < 5) {
		SET site_alt TO cur_alt_above_target(landing_state).
	}
	
	PRINTPLACE(ROUND(site_alt, 2) + " m",12,19,surfloc).
	PRINTPLACE(ROUND(SHIP:VERTICALSPEED,1) + " m/s",12,19,surfloc+1).
	
	PRINTPLACE(ROUND(downrangedist(landing_state["tgtsite"],SHIP:GEOPOSITION),2) + " km",12,50,surfloc).
	PRINTPLACE(ROUND(SHIP:GROUNDSPEED,1) + " m/s",12,50,surfloc+1).
	
	local v_ is 0.
	
	IF (vehiclestate["ops_mode"] =0) OR (vehiclestate["ops_mode"]>=4) {set v_ to SHIP:SRFPROGRADE:VECTOR.}
	ELSE {set v_ to SHIP:PROGRADE:VECTOR.}

	PRINTPLACE(ROUND(90 - VANG(SHIP:FACING:VECTOR, SHIP:UP:VECTOR),2) + " deg",12,19,surfloc+2).
	PRINTPLACE(ROUND(compass_for(SHIP:FACING:VECTOR,SHIP:GEOPOSITION),2) + " deg",12,50,surfloc+2).
	PRINTPLACE(ROUND(tgt_rate,2) + " m/s",12,19,surfloc+3).
	
	
	
	
	//vehicle data
	
	LOCAL cur_stg_idx IS vehiclestate["cur_stg"].
	
	LOCAL total_stg_time IS 0.
	FOR s IN vehicle["stages"]:SUBLIST(cur_stg_idx,vehicle["stages"]:LENGTH - cur_stg_idx) {
		SET total_stg_time TO total_stg_time + s["Tstage"].
	}
	
	PRINTPLACE(sectotime(total_stg_time),12,19,vehloc).
	PRINTPLACE(ROUND(get_TWR(),2) + " ",12,19,vehloc+1).
	PRINTPLACE(ROUND(THROTTLE*100,1) + " %",12,19,vehloc+2).
	
	PRINTPLACE(" " + cur_stg_idx + " ",12,19,vehloc + 3).
	
	LOCAL stg IS get_stage().
	
	PRINTPLACE(stg["staging"]["type"],12,19,vehloc+4).
	PRINTPLACE(sectotime(stg["Tstage"]),12,19,vehloc+5).
	
	
	//upfg data
	//IF (vehiclestate["ops_mode"] =2) OR (vehiclestate["ops_mode"] =1) OR (vehiclestate["ops_mode"] =3) {
	IF (vehiclestate["ops_mode"]<4) {
		
		PRINTPLACE(current_orbit["mode"],12,50,vehloc).
	
		IF usc["conv"]=1 { PRINTPLACE("CONVERGED",12,50,vehloc+1). }
		ELSE IF usc["conv"]>-4 { PRINTPLACE("CONVERGING",12,50,vehloc+1). }
		ELSE { PRINTPLACE("NOT CONVERGED",12,50,vehloc+1). }
		
		IF DEFINED (upfgInternal) {
			PRINTPLACE(sectotime(upfgInternal["Tgo"]),12,50,vehloc+3). 
			PRINTPLACE(ROUND(upfgInternal["vgo"]:MAG,0),12,50,vehloc+4). 
			PRINTPLACE(ROUND(upfgInternal["z_error"],0)+ " m",12,50,vehloc+5). 
		}
		
	} ELSE {		
		PRINTPLACE(ROUND(ALT:RADAR, 2) + " m",12,50,vehloc).
		PRINTPLACE(ROUND(SHIP:GROUNDSPEED,1) + " m/s",12,50,vehloc+1).
		PRINTPLACE(ROUND(tgt_rate,2) + " m/s",12,50,vehloc+2).
	}

	
	//messages
	 message_Viz().
	
	

}

