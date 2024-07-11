GLOBAL pdi_angle_guess IS 20.

GLOBAL current_orbit IS LEXICON (	
								"periapsis",ORBIT:PERIAPSIS,
								"apoapsis",ORBIT:APOAPSIS,
								"inclination",ORBIT:INCLINATION,
								"SMA",ORBIT:SEMIMAJORAXIS,
								"ecc",ORBIT:ECCENTRICITY,
								"LAN",ORBIT:LAN,
								"periarg",ORBIT:ARGUMENTOFPERIAPSIS
).

GLOBAL landing_state IS LEXICON(
						"body", tgt_body,
						"tgtsite_name", "",
						"tgtsite", LATLNG(0, 0),
						"altitude",100,
						"velocity",1,
						"angle",-90,
						"altitude_bias", 0,
						"range_bias",1500,
						"velocity_bias",0,
						"tgt_redesig_t",100,
						"final_apch_t",45	
).


GLOBAL surfacestate IS  LEXICON("MET",0,"az",0,"pitch",0,"alt",0,"vs",0,"hs",0,"vdir",0,"hdir",0,"q",0).
GLOBAL orbitstate IS  LEXICON("radius",0,"velocity",0). 


								//INITIALISATION FUNCTION 
								
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
}

								
// need to work in upfg frame								
FUNCTION pre_converge_guidance {
	
	PRINT " RUNNING UPFG PRE-CONVERGENCE" AT (0,5).
	
	//initialise pdi time ahead 
	//if the current position is less than 60° from the landing site right now add one orbit
	SET landing_state["radius"] TO shift_landing_posvec(landing_state, current_orbit["pdi_time_ahead"]).
	
	LOCAL tgtvec_proj IS VXCL(current_orbit["normal"], landing_state["radius"]):NORMALIZED.
	
	LOCAL pdi_guess_vec IS rodrigues(tgtvec_proj, current_orbit["normal"], -pdi_angle_guess):NORMALIZED.
	LOCAL min_standoff_initial_vec IS rodrigues(tgtvec_proj, current_orbit["normal"], -2*pdi_angle_guess):NORMALIZED.
	LOCAL veh_pos IS vecYZ(-SHIP:ORBIT:BODY:POSITION).
	
	//choose the maximum bw the guess pdi position and the min standoff
	//if the vehicle is past the standoff this will force one extra rotation

	//while the standoff angle must be positive if we're past it 
	LOCAL standoff_angle IS signed_angle(veh_pos, min_standoff_initial_vec, current_orbit["normal"], 1).
	
	set current_orbit["radius"] TO pdi_guess_vec.
	
	LOCAL alpha_landing_error_old IS 0.
	LOCAL iter_count IS 0.
	UNTIL FALSE {
		SET iter_count TO iter_count + 1.
	
		PRINT " ITERATION " +  iter_count AT (0,7).
	
		//calculate angle bw the standoff and the pdi guess
		LOCAL standoff_angle_to_pdi IS signed_angle(min_standoff_initial_vec, current_orbit["radius"], current_orbit["normal"], 0).
		
		//add 30° to the standoff angle to find the time to pdi_guess
		SET current_orbit["pdi_time_ahead"] TO eta_to_dt(standoff_angle + standoff_angle_to_pdi, current_orbit["SMA"], current_orbit["ecc"]).

		if (debug) {
			print "standoff_angle " + standoff_angle at (0,10).
			print "standoff_angle_to_pdi " + standoff_angle_to_pdi at (0,11).
			print "pdi_time " + sectotime(current_orbit["pdi_time_ahead"]) at (0,12).
			
			print "radius " + (landing_state["radius"]:mag - body:radius) at (0,12).
			
		}
		
		
		SET landing_state tO update_landing_state(landing_state, current_orbit["radius"], current_orbit["pdi_time_ahead"]).
		
		calculate_veh_state_at_pdi().
		
		//converge upfg
		LOCAL x IS setupUPFG(landing_state).
		SET upfgInternal TO x[0].
		SET usc TO x[1].

		UNTIL FALSE {
			IF usc["conv"]=1 {
				PRINT "                                                          " AT (0,9).
				PRINT " GUIDANCE CONVERGED IN " + usc["itercount"] + " ITERATIONS" AT (0,9).
				BREAK.
			}
			SET upfgInternal TO upfg_wrapper(upfgInternal).

			WAIT 0.
		}
		
		//ad tgo to the time ahead prediction
		LOCAL shifted_tgt_tgo IS shift_landing_posvec(landing_state, current_orbit["pdi_time_ahead"] + upfgInternal["tgo"]).
		
		LOCAL alpha_landing_error IS signed_angle(upfgInternal["rp"], shifted_tgt_tgo, landing_state["normal"]:NORMALIZED, 0).
		
		if (debug) {
			PRINTPLACE(sectotime(upfgInternal["Tgo"]),12,50,10). 
			PRINTPLACE(ROUND(upfgInternal["vgo"]:MAG,0),12,50,11). 
			PRINTPLACE(ROUND(landing_state["delta_v_estimate"],0),12,50,12). 
			PRINTPLACE(ROUND(alpha_landing_error,5),12,50,13). 
			
			clearvecdraws().
		
			arrow_body(vecyz(current_orbit["normal"]) * SHIP:ORBIT:BODY:RADIUS, "normal").
			arrow_body(vecyz(landing_state["radius"]),"tgt").
		
		
			arrow_body(vecyz(upfgInternal["rp"]),"rp").
			arrow(vecyz(orbitstate["radius"]),"initial_r", SHIP:ORBIT:BODY:POSITION, 1.2).
			arrow(vecyz(orbitstate["velocity"]*1000),"initial_v", SHIP:ORBIT:BODY:POSITION + vecyz(orbitstate["radius"])*1.2, 1.2).
		}
		
		IF (ABS(alpha_landing_error - alpha_landing_error_old) < 0.01) {BREAK.}
		
		SET alpha_landing_error_old TO alpha_landing_error.
		
		SET current_orbit["radius"] TO rodrigues(current_orbit["radius"], current_orbit["normal"],alpha_landing_error).
		
		if (debug) {
			WAIT 0.8.
		}
	}
	
	if (debug) {
		until false{}
	}
	
	
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
	PARAMETER ldg_state.
	PARAMETER dt.
	
	LOCAL site IS vecYZ(ldg_state["tgtsite"]:POSITION - SHIP:ORBIT:BODY:POSITION).
	
	SET ldg_state["radius"] TO  site:NORMALIZED*(BODY:RADIUS + ldg_state["tgtsite"]:TERRAINHEIGHT).
	IF (NOT dt=0) {
		LOCAL mode IS ldg_state["mode"].
		//if pre-convergence expect mode to not be zero
		//if landing mode it should be zero
		//switches the sign of the body rotation vector
		
		LOCAL rotvec IS vecYZ(BODY:ANGULARVEL:NORMALIZED).
		SET ldg_state["radius"] TO  rodrigues(ldg_state["radius"], rotvec:NORMALIZED, 360*dt/BODY:ROTATIONPERIOD).
	}
	
	RETURN ldg_state["radius"].
}



FUNCTION update_landing_state {
	PARAMETER ldg_state.
	PARAMETER cur_pos.
	PARAMETER dt.
	
	SET ldg_state["radius"] TO shift_landing_posvec(ldg_state, dt).
	
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

FUNCTION cur_alt_above_target {
	PARAMETER ldg_state.
	
	LOCAL local_datum_alt IS bodyalt(-SHIP:ORBIT:BODY:POSITION).
	
	LOCAL tgt_datum_alt IS bodyalt(pos2vec(ldg_state["tgtsite"])).
	
	print "local_datum_alt " + local_datum_alt at (2,40).
	print "tgt_datum_alt " + tgt_datum_alt at (2,41).
	
	RETURN local_datum_alt - tgt_datum_alt.
}


//redesignate landing target during P64
FUNCTION redesig_landing_tgt {
	PARAMETER delta_shift.
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

FUNCTION redraw_target_arrow {
	clearvecdraws().
	LOCAL arrow_scalefac IS 15 + landing_state["tgtsite"]:DISTANCE/10.
	
	pos_arrow(
		landing_state["tgtsite"],
		"target",
		arrow_scalefac,
		0.1
	).
}
