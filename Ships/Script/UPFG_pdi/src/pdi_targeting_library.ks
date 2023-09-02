

GLOBAL current_orbit IS LEXICON (	
								"periapsis",ORBIT:PERIAPSIS,
								"apoapsis",ORBIT:APOAPSIS,
								"inclination",ORBIT:INCLINATION,
								"SMA",ORBIT:SEMIMAJORAXIS,
								"ecc",ORBIT:ECCENTRICITY,
								"LAN",ORBIT:LAN,
								"periarg",ORBIT:ARGUMENTOFPERIAPSIS
).


GLOBAL surfacestate IS  LEXICON("MET",0,"az",0,"pitch",0,"alt",0,"vs",0,"hs",0,"vdir",0,"hdir",0,"q",0).
GLOBAL orbitstate IS  LEXICON("radius",0,"velocity",0). 


								//INITIALISATION FUNCTION 
								
// need to work in upfg frame								
FUNCTION setup {
	
	landing_state:ADD("radius", v(0,0,0)).
	landing_state:ADD("velvec", v(0,0,0)).
	landing_state:ADD("normal", v(0,0,0)).
	landing_state:ADD("fpa", 0) .
	landing_state:ADD("mode", 1).

	//this is the state at powered descent ignition position
	current_orbit:ADD("radius", 0) .	
	current_orbit:ADD("velocity", 0) .
	current_orbit:ADD("normal", V(0,0,0)) .
	current_orbit:ADD("fpa", 0) .
	current_orbit:ADD("eta", 0) .
	current_orbit:ADD("mode", 2) .
	current_orbit:ADD("pdi_time_ahead", 0) .	//wil be relative to vehicle_ignition which is the "T=0" for all calculations


	SET current_orbit["normal"] TO upfg_normal(current_orbit["inclination"], current_orbit["LAN"]).
	current_orbit:ADD(
					"perivec", 
					vecYZ(targetPerivec(
								current_orbit["inclination"],
								current_orbit["LAN"],
								current_orbit["periarg"]
					))
	) .
	
	PRINT "RUNNING UPFG PRE-CONVERGENCE" AT (0,5).
	
	//initialise pdi time ahead 
	//if the current position is less than 60° from the landing site right now add one orbit
	shift_landing_posvec(current_orbit["pdi_time_ahead"]).
	
	LOCAL tgtvec_proj IS VXCL(current_orbit["normal"], landing_state["radius"]):NORMALIZED.
	
	LOCAL pdi_guess_vec IS rodrigues(tgtvec_proj, current_orbit["normal"], -30):NORMALIZED.
	LOCAL min_standoff_initial_vec IS rodrigues(tgtvec_proj, current_orbit["normal"], -60):NORMALIZED.
	LOCAL veh_pos IS vecYZ(-SHIP:ORBIT:BODY:POSITION).
	
	//choose the maximum bw the guess pdi position and the min standoff
	//if the vehicle is past the standoff this will force one extra rotation

	//while the standoff angle must be positive if we're past it 
	LOCAL standoff_angle IS signed_angle(veh_pos, min_standoff_initial_vec, current_orbit["normal"], 1).
	
	set current_orbit["radius"] TO pdi_guess_vec.
	
	LOCAL alpha_landing_error_old IS 0.
	UNTIL FALSE {
	
		//calculate angle bw the standoff and the pdi guess
		LOCAL standoff_angle_to_pdi IS signed_angle(min_standoff_initial_vec, current_orbit["radius"], current_orbit["normal"], 0).
		
		//add 30° to the standoff angle to find the time to pdi_guess
		SET current_orbit["pdi_time_ahead"] TO eta_to_dt(standoff_angle + standoff_angle_to_pdi, current_orbit["SMA"], current_orbit["ecc"]).

		print "standoff_angle " + standoff_angle at (0,1).
		print "standoff_angle_to_pdi " + standoff_angle_to_pdi at (0,2).
		print "pdi_time " + current_orbit["pdi_time_ahead"] at (0,3).
		
		
		SET landing_state tO update_landing_state(landing_state, current_orbit["radius"], current_orbit["pdi_time_ahead"]).
		
		calculate_veh_state_at_pdi().
		
		//converge upfg
		LOCAL x IS setupUPFG(landing_state).
		SET upfgInternal TO x[0].
		SET usc TO x[1].

		UNTIL FALSE {
			IF usc["conv"]=1 {
				PRINT "                                                         " AT (0,7).
				PRINT "GUIDANCE CONVERGED IN " + usc["itercount"] + " ITERATIONS" AT (0,7).
				BREAK.
			}
			SET upfgInternal TO upfg_wrapper(upfgInternal).

			WAIT 0.
		}
		
		LOCAL alpha_landing_error IS signed_angle(upfgInternal["rp"], landing_state["radius"], landing_state["normal"]:NORMALIZED, 0).
		
		PRINTPLACE(sectotime(upfgInternal["Tgo"]),12,50,10). 
		PRINTPLACE(ROUND(upfgInternal["vgo"]:MAG,0),12,50,11). 
		PRINTPLACE(ROUND(alpha_landing_error,5),12,50,12). 
		
		clearvecdraws().
	
		arrow_body(vecyz(current_orbit["normal"]) * SHIP:ORBIT:BODY:RADIUS, "normal").
		arrow_body(vecyz(landing_state["radius"]),"tgt").
	
	
		arrow_body(vecyz(upfgInternal["rp"]),"rp").
		arrow_body(vecyz(orbitstate["radius"]),"initial_r").
		
		IF (ABS(alpha_landing_error - alpha_landing_error_old) < 0.01) {BREAK.}
		
		SET alpha_landing_error_old TO alpha_landing_error.
		
		SET current_orbit["radius"] TO rodrigues(current_orbit["radius"], current_orbit["normal"],alpha_landing_error).
		
		WAIT 0.8.
	}
	
	
	until false{}
	

	
	
	
	
	
	
	
	SET current_orbit["mode"] TO 1.


	
}	


// given pdi position do orbital mechanics and work out initial parameters and velocity
FUNCTION calculate_veh_state_at_pdi {

	//radius, velocity, fpa
	
	set current_orbit["eta"] to signed_angle(current_orbit["perivec"],current_orbit["radius"],current_orbit["normal"]:NORMALIZED,1).
	set current_orbit["fpa"] to orbit_eta_fpa(current_orbit["eta"], current_orbit["SMA"], current_orbit["ecc"]).
	SET current_orbit["radius"] TO current_orbit["radius"]:NORMALIZED * orbit_eta_alt(current_orbit["eta"], current_orbit["SMA"], current_orbit["ecc"]).
	set current_orbit["velocity"] to orbit_alt_vel(current_orbit["radius"]:MAG, current_orbit["SMA"]).

	//prepare fictitious initial conditions at the arbitrary position
	SET orbitstate["radius"] TO current_orbit["radius"].
	
	LOCAL iz IS VCRS(current_orbit["normal"], current_orbit["radius"]):NORMALIZED.
	
	SET orbitstate["velocity"] TO rodrigues(iz, current_orbit["normal"], current_orbit["fpa"]):NORMALIZED * current_orbit["velocity"].
	
}


FUNCTION warp_to_pdi{

	parameter pdi_time.
	
	LOCAL t2pdi IS pdi_time - TIME:SECONDS.

		
	IF t2pdi > 3600 or t2pdi < -60    {set warp to 4.}
	ELSE IF t2pdi > 1000     {set warp to 3.}
	ELSE IF t2pdi > 180     {set warp to 2.}
	ELSE IF t2pdi > 35  {set warp to 1.}
	ELSE IF t2pdi > 27  {set warp to 0.}						
}








//		NAVIGATION FUNCTIONS 



FUNCTION update_navigation {
	
	SET surfacestate["MET"] TO TIME:SECONDS. 
	
	
	//measure position and orbit parameters
	
	LOCAL progv IS v(0,0,0).
	
	IF vehiclestate["ops_mode"] >1 {set progv to SHIP:PROGRADE:VECTOR.}
	ELSE {set progv to SHIP:SRFPROGRADE:VECTOR.}
	
	SET surfacestate["hdir"] TO compass_for(progv,SHIP:GEOPOSITION ).
	SET surfacestate["vdir"] TO 90 - VANG(progv, SHIP:UP:VECTOR).
	SET surfacestate["pitch"] TO 90 - VANG(SHIP:FACING:VECTOR, SHIP:UP:VECTOR).	
	SET surfacestate["az"] TO compass_for(SHIP:FACING:VECTOR,SHIP:GEOPOSITION ).
	SET surfacestate["alt"] TO SHIP:ALTITUDE.
	SET surfacestate["vs"] TO SHIP:VERTICALSPEED.
	SET surfacestate["hs"] TO SHIP:VELOCITY:SURFACE:MAG.
	
	SET orbitstate["velocity"] TO vecYZ(SHIP:ORBIT:VELOCITY:ORBIT).
	SET orbitstate["radius"] TO vecYZ(SHIP:ORBIT:BODY:POSITION)*-1.

}

//will move the landing site according to body rotation and time parameter.
//time parameter is positive in the future i.e. move the target "ahead"
FUNCTION shift_landing_posvec {
	PARAMETER dt.

	LOCAL mode IS landing_state["mode"].
	//if pre-convergence expect mode to not be zero
	//if landing mode it should be zero
	//switches the sign of the body rotation vector
	
	LOCAL rotvec IS vecYZ(BODY:ANGULARVEL:NORMALIZED).
	
	LOCAL site IS vecYZ(landing_state["tgtsite"]:POSITION - SHIP:ORBIT:BODY:POSITION).
	
	SET landing_state["radius"] TO  site:NORMALIZED*(BODY:RADIUS + landing_state["tgtsite"]:TERRAINHEIGHT).
	IF (NOT dt=0) {
		SET landing_state["radius"] TO  rodrigues(landing_state["radius"], rotvec:NORMALIZED, 360*dt/BODY:ROTATIONPERIOD).
	}
}



FUNCTION update_landing_state {
	PARAMETER ldg_state.
	PARAMETER cur_pos.
	PARAMETER dt.
	
	shift_landing_posvec(dt).
	
	LOCAL dr IS ldg_state["radius"] - cur_pos.
	SET ldg_state["normal"] TO VCRS(ldg_state["radius"],dr):NORMALIZED.
	
	//add altitude biases
	SET ldg_state["radius"] TO ldg_state["radius"]:NORMALIZED*(ldg_state["radius"]:MAG + ldg_state["altitude"] +  ldg_state["altitude_bias"]).
	
	//positive range bias means we shift the target behind i.e. towards us 
	LOCAL angle_bias IS -(ldg_state["range_bias"]/BODY:RADIUS)*(180/CONSTANT:PI).
	
	//clearvecdraws().
	//arrow_body(vecyz(ldg_state["radius"]) * SHIP:ORBIT:BODY:RADIUS, "tgt").
	
	SET ldg_state["radius"] TO  rodrigues(ldg_state["radius"], ldg_state["normal"], angle_bias).
	
	//arrow_body(vecyz(ldg_state["radius"]) * SHIP:ORBIT:BODY:RADIUS, "bias_tgt").
	
	//points towards the landing  site
	LOCAL iz IS -VCRS(ldg_state["radius"], ldg_state["normal"]):NORMALIZED.
	LOCAL vel IS rodrigues(iz, ldg_state["normal"], ldg_state["fpa"]):NORMALIZED*ldg_state["velocity"].	
	
	//correct with the velocity of the ground plus the bias
	LOCAL velcorr IS ldg_state["tgtsite"]:VELOCITY:ORBIT:MAG + ldg_state["velocity_bias"].
	SET ldg_state["velvec"] TO vel + velcorr*VCRS(vecYZ(BODY:ANGULARVEL:NORMALIZED), ldg_state["radius"]):NORMALIZED.
	
	//arrow_body(vecyz(vel) * SHIP:ORBIT:BODY:RADIUS, "vel").
	//arrow_body(vecyz(ldg_state["velvec"]) * SHIP:ORBIT:BODY:RADIUS, "velvec").
	
	return ldg_state.
}



//redesignate landing target during P64
FUNCTION redesig_landing_tgt {
	PARAMETER refvec.
	PARAMETER enabled_.
	
	IF (enabled_) {
		local controlvec IS v(SHIP:CONTROL:PILOTYAW,-SHIP:CONTROL:PILOTPITCH,0).
		SET controlvec TO get_shifts(controlvec,refvec).
		
		LOCAL tgtlong IS landing_state["tgtsite"]:LNG.
		LOCAL tgtlat IS landing_state["tgtsite"]:LAT.
		set tgtlong TO tgtlong + delta_shift*controlvec:X.
		set tgtlat TO tgtlat + delta_shift*controlvec:Y.
		
		SET landing_state["tgtsite"] TO LATLNG(tgtlat,tgtlong).
	}

	clearvecdraws().
	LOCAL arrow_scalefac IS 1 + landing_state["tgtsite"]:DISTANCE/150.
	
	pos_arrow(
		landing_state["tgtsite"]:POSITION,
		"target",
		10*arrow_scalefac,
		0.05*arrow_scalefac
	).
}



//given target translation shifts in a given frame
//rotates them to the north-east frame.
FUNCTION get_shifts {
	parameter controlvec.
	parameter refvec.

	LOCAL northvec IS (LATLNG(90,0):POSITION - SHIP:ORBIT:BODY:POSITION):NORMALIZED.
	
	SET refvec TO VXCL(SHIP:ORBIT:BODY:POSITION:NORMALIZED,refvec).
	SET northvec TO VXCL(SHIP:ORBIT:BODY:POSITION:NORMALIZED,northvec).

	local alpha is signed_angle(northvec,refvec,- SHIP:ORBIT:BODY:POSITION,1).
	local out IS v(0,0,0).
	
	SET out:X TO controlvec:X*COS(alpha) + controlvec:Y*SIN(alpha).
	SET out:Y TO - controlvec:X*SIN(alpha) + controlvec:Y*COS(alpha).
	
	return out.
}
