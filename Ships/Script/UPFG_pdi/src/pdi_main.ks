function init{
	CLEARSCREEN.
	clearvecdraws().
	
	//relative to tgo in the biased target phase
	GLOBAL tgt_redesig_T IS 60.//80
	//relative to tgo in the actual target phase
	GLOBAL final_apch_T IS 45.//60
	//tilt angle limit from vertical during pitchdown and att hold modes
	gLOBAL anglelim IS 20.
	
	//	Load libraries
	RUNPATH("0:/Libraries/misc_library").	
	RUNPATH("0:/Libraries/maths_library").	
	RUNPATH("0:/Libraries/navigation_library").	
	RUNPATH("0:/Libraries/vehicle_library").	
	
	RUNPATH("0:/UPFG_pdi/src/pdi_interface_library").
	RUNPATH("0:/UPFG_pdi/src/pdi_targeting_library").
	RUNPATH("0:/UPFG_pdi/src/pdi_upfg_library").
	RUNPATH("0:/UPFG_pdi/src/pdi_vehicle_library").
	
	//	Load vessel file
	IF (vesselfilename:ENDSWITH(".ks")=TRUE) {
		SET vesselfilename TO vesselfilename:REMOVE( vesselfilename:FIND(".ks") ,3 ).
	}
	RUNPATH("0:/UPFG_pdi/VESSELS/" + vesselfilename + ".ks").
	
	
	wait until ship:unpacked and ship:loaded.
	
	GLOBAL tgt_rate IS 0.
	GLOBAL rate_dt IS 0.5.
	GLOBAL delta_rate IS 0.25.
	GLOBAL delta_shift IS 5000.
	GLOBAL null_velocity_gain IS 4.
	SET delta_shift TO delta_shift/SHIP:BODY:RADIUS.
	
	
	initialise_vehicle().
	
	setup_steering_magaer().
	
	//set the ignition time into the future to bias the mission closk
	//and prevent events from triggering before PDI
	set vehicle["stages"][1]["ign_t"] to TIME:SECONDS + 100000.
	
	main_loop().
}


FUNCTION main_loop {

setup().

return.
	
	drawUI().
	//get_mass_bias().
	getState().
	
	
	
	addMessage("SETTING UP PRE-CONVERGENCE").
	
	
	
	SET vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"] TO 0.8.

	
	LOCAL x IS setupUPFG(target_orbit).
	GLOBAL upfgInternal IS x[0].
	GLOBAL usc IS x[1].
	
	
	addMessage("RUNNING PRE-CONVERGENCE").
	dataViz().
	
	//to bypass iteration time of UPFG
	SET vehiclestate["staging_in_progress"] TO TRUE.
	
	
	//attemp to run many pre-convergence iterations of UPPFG altering the initial state
	//until the predicted final posiion converges to the frozen target
	
	LOCAL pd_tgo IS upfgInternal["tgo"].
	LOCAL d_alpha_old IS 0.
	UNTIL FALSE {
		
		//converge upfg 
		UNTIL FALSE {
			IF usc["itercount"]=0 { //detects first pass or convergence lost
				WHEN usc["conv"]=1 THEN {
					addMessage("GUIDANCE CONVERGED IN " + usc["itercount"] + " ITERATIONS").
				}
			}
			IF usc["conv"]=1 {BREAK.}
			SET upfgInternal TO upfg_wrapper(upfgInternal).

			dataViz().
			WAIT 0.
		}
		SET pd_tgo TO upfgInternal["tgo"].
		
		if (debug) {
			clearvecdraws().
			arrow_body(vecyz(upfgInternal["rp"]),"rp").
			arrow_body(vecyz(target_orbit["radius"]),"tgt").
			arrow_body(vecyz(current_orbit["radius"]),"ign").
		}
		
		//error angle
		LOCAL d_alpha IS signed_angle(target_orbit["radius"],upfgInternal["rp"],target_orbit["normal"]:NORMALIZED,1).
		SET d_alpha TO unfixangle(d_alpha).
		
		//rotate the pdi state backwards by the error angle
		SET current_orbit["radius"] TO rodrigues(current_orbit["radius"],target_orbit["normal"],-d_alpha).
		
		//redefine the orbital state at pdi
		SET current_orbit["eta"] TO signed_angle(current_orbit["perivec"],current_orbit["radius"],current_orbit["normal"]:NORMALIZED,1).	
		set current_orbit["velocity"] to orbit_alt_vel(current_orbit["radius"]:MAG, current_orbit["SMA"]).
		set current_orbit["fpa"] to orbit_eta_fpa(current_orbit["eta"], current_orbit["SMA"], current_orbit["ecc"]).

		//prepare fictitious initial conditions at the arbitrary position
		SET orbitstate["radius"] TO current_orbit["radius"].
		LOCAL iz IS VCRS(current_orbit["radius"],current_orbit["normal"]):NORMALIZED.
		SET orbitstate["velocity"] TO rodrigues(iz,current_orbit["normal"], current_orbit["fpa"]):NORMALIZED*current_orbit["velocity"].
		
		//after the state has been corrected check if we should break out 
		IF ( ( ABS (d_alpha - d_alpha_old)  < 0.01 ) OR ( ABS (d_alpha) - ABS(d_alpha_old) < 0.01 ) ) {BREAK.}

		SET d_alpha_old TO d_alpha.
		LOCAL x IS setupUPFG(target_orbit).
		SET upfgInternal[0] TO x[0].
		SET usc TO x[1].
		SET upfgInternal["tgo"] TO pd_tgo.
		
		if (debug) {
			wait 1.
		}
	}
	
	return.
	
	SET vehiclestate["staging_in_progress"] TO FALSE.
	
	//this is the prediction for the angle traversed during descent
	LOCAL pd_angle IS vang(upfgInternal["rp"],current_orbit["radius"]).
	
	SET current_orbit["normal"] TO upfg_normal(ABS(current_orbit["inclination"]), current_orbit["LAN"]).
	SET current_orbit["mode"] TO 2.
	SET landing_state["mode"] TO 2.
	update_landing_tgt(0).
	
	//given the actual target position, iterate to find ignition position
	LOCAL shipvec IS -vecYZ(SHIP:ORBIT:BODY:POSITION:NORMALIZED).
	SET shipvec TO VXCL(current_orbit["normal"],shipvec).
	LOCAL sitevec IS v(0,0,0).
	LOCAL ignitionvec IS v(0,0,0).
	LOCAL eta1 IS signed_angle(current_orbit["perivec"],shipvec,current_orbit["normal"],1).	
	SET t1 TO SHIP:ORBIT:PERIOD - eta_to_dt(eta1,current_orbit["SMA"],current_orbit["ecc"]).
	LOCAL epsilon IS 0.001.
	LOCAL pdi_time IS 0.
	
	UNTIL FALSE {
		LOCAL ignitionvec_old IS ignitionvec.
		
		SET sitevec TO VXCL(current_orbit["normal"],landing_state["radius"]).
		SET ignitionvec TO rodrigues(sitevec,-current_orbit["normal"],-pd_angle).
		
		LOCAL eta2 IS signed_angle(current_orbit["perivec"],ignitionvec,-current_orbit["normal"],1).		
		SET t2 TO SHIP:ORBIT:PERIOD - eta_to_dt(eta2,current_orbit["SMA"],current_orbit["ecc"]).
		SET pdi_time TO t2 - t1 .
		update_landing_tgt(pdi_time + pd_tgo).
		
		
		
		IF VANG(ignitionvec_old,ignitionvec) <= epsilon {BREAK.}
		
		WAIT 0.
	}
	
	//initialise ignition timer
	SET vehicle["ign_t"] TO TIME:SECONDS + pdi_time.
	
	addMessage("POWERED DESCENT INITIATION POINT DETERMINED").
	
	IF (debug) {
		
		print "ignition phase angle " + signed_angle(shipvec,ignitionvec,-current_orbit["normal"],1) at (1,40).
		print "site phase angle " + signed_angle(shipvec,VXCL(current_orbit["normal"],landing_state["radius"]),-current_orbit["normal"],1) at (1,41).
		print "true anomaly " + signed_angle(shipvec,current_orbit["perivec"],-current_orbit["normal"],1) at (1,42).
		print "pdi eta " + pdi_time at (1,44).
		print "burn angle " + pdi_time at (1,44).
		print "tgo " + upfgInternal["tgo"] at (1,45).
		return.

	}
	
		//arrow(vecYZ( CROSS(ignitionvec:NORMALIZED,current_orbit["normal"]) ), "pdi_retro",v(0,0,0),10,0.05).
		//arrow(vecYZ(usc["lastvec"]), "pdivec",v(0,0,0),10,0.05).
		
	
	//initialise initial orientation
	
	SET shipvec TO rodrigues(shipvec,-current_orbit["normal"],vang(shipvec,ignitionvec)).
	SET control["refvec"] TO vecYZ(shipvec).
	set control["steerdir"] TO aimAndRoll(vecYZ(usc["lastvec"]), control["refvec"], control["roll_angle"]). 
	
	SET vehiclestate["ops_mode"] TO 1.
	
	
	//SET STEERINGMANAGER:MAXSTOPPINGTIME TO 1.
	SET target_orbit TO landing_state.
	
	GLOBAL redesig_flag IS FALSE.
	
	GLOBAL att_hold_swch IS FALSE.
	
	
	//initialise PDI cues
	//warp halt and attitude cue
	WHEN TIME:SECONDS > vehicle["ign_t"] - (60*warp) THEN {
		set warp to 0.
		SAS OFF.
		RCS ON.
		SET control["refvec"] TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
		LOCK STEERING TO control["steerdir"].
		
		WHEN TIME:SECONDS > vehicle["ign_t"] - 15 THEN {
			addMessage("PREPARE FOR POWERED DESCENT").
		}
	}
	
	//WHEN TIME:SECONDS > vehicle["ign_t"] - (60*warp) THEN {
	//	set warp to 0.
	//	SAS OFF.
	//	SET control["refvec"] TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	//	LOCK STEERING TO control["steerdir"].
	//	
	//	//ullage cue
	//	WHEN TIME:SECONDS > vehicle["ign_t"] - 15 THEN {
	//		addMessage("PREPARE FOR POWERED DESCENT").
	//		//freeze attitude
	//		LOCK STEERING TO LOOKDIRUP(SHIP:FACING:FOREVECTOR, SHIP:FACING:TOPVECTOR).
	//		
	//		//setup PDI cue
	//		WHEN  TIME:SECONDS >= vehicle["ign_t"] - 0.05 THEN {
	//			addMessage("INITIATING POWERED DESCENT").
	//			update_landing_tgt(pd_tgo).
	//			SET target_orbit TO landing_state.
	//			LOCAL x IS setupUPFG(target_orbit).
	//			SET upfgInternal[0] TO x[0].
	//			SET usc TO x[1].
	//			SET usc["lastvec"] TO vecYZ(SHIP:retrograde:FOREVECTOR).
	//			SET upfgInternal["tgo"] TO pd_tgo.
	//			SET control["refvec"] TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	//			set control["steerdir"] TO aimAndRoll(vecYZ(usc["lastvec"]), control["refvec"], control["roll_angle"]). 
	//			SAS OFF.
	//			LOCK STEERING TO control["steerdir"].
	//			LOCK THROTTLE to throttleControl().
	//			SET SHIP:CONTROL:PILOTMAINTHROTTLE TO vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"].
	//			SET vehiclestate["ops_mode"] TO 2.
	//			
	//			
	//			//landing target redesignation cues
	//			WHEN (vehiclestate["ops_mode"] = 2) AND (usc["conv"]=1) AND (upfgInternal["tgo"]<tgt_redesig_T) THEN {
	//				SET upfgInternal TO resetUPFG(upfgInternal).
	//				GLOBAL redesig_flag IS TRUE.
	//				SET vehiclestate["ops_mode"] TO 3.
	//				SET landing_state["mode"] TO 2.
	//				SET landing_state["altitude_bias"] TO 0.
	//				set landing_state["range_bias"] to 750.
	//				set landing_state["velocity_bias"] to 0.
	//				addMessage("TARGET SITE REDESIGNATION ENABLED").
	//				
	//				WHEN (upfgInternal["tgo"]<(final_apch_T+1)) THEN {
	//					SET redesig_flag TO FALSE.
	//				}
	//				
	//				//addMessage("PRESS AG8 TO ENABLE REDESIGNATION").
	//				//ON AG8 {
	//				//	SET redesig_flag TO NOT(redesig_flag).
	//				//	IF redesig_flag {
	//				//		addMessage("TARGET SITE REDESIGNATION ENABLED").
	//				//	} ELSE {
	//				//		addMessage("TARGET SITE REDESIGNATION DISABLED").
	//				//	}
	//				//	IF (upfgInternal["tgo"]<(final_apch_T+1)) {
	//				//		RETURN TRUE.
	//				//		addMessage("TARGET SITE FROZEN").
	//				//	} ELSE {
	//				//		RETURN FALSE.
	//				//	}
	//				//	
	//				//}
	//			}
	//			
	//			//cue to switch to att hold
	//			WHEN (vehiclestate["ops_mode"] = 3) AND (usc["conv"]=1) AND (upfgInternal["tgo"]<final_apch_T) THEN {
	//				addMessage("PRESS AG9 TO SWITCH TO ATT HOLD").
	//				SET redesig_flag TO FALSE.
	//				ON AG9 {
	//					SET vehiclestate["ops_mode"] TO 4.
	//					SET control["refvec"] TO SHIP:FACING:TOPVECTOR.
	//					set vehicle["roll"] TO 0.
	//					SET tgt_rate TO -ABS(landing_state["velocity"]).
	//					
	//					drawUI().
	//					clearvecdraws().
	//					
	//					WHEN (vehiclestate["ops_mode"] = 4) AND (ALT:RADAR <= landing_state["altitude"]) THEN {
	//						UNLOCK STEERING.
	//						SAS ON.
	//						SET vehiclestate["ops_mode"] TO 5.
	//						SET usc["lastthrot"] TO tgt_throt(0).
	//						
	//						drawUI().
	//						
	//						//trigger to engage/disengage velocity nulling
	//						ON SAS {
	//							IF SAS {
	//								UNLOCK STEERING. 
	//							}
	//							ELSE {
	//								LOCK STEERING TO control["steerdir"].
	//							}
	//							RETURN TRUE.
	//						}
	//
	//					}
	//					
	//				}
	//			}
	//			
	//		}
	//	}
	//
	//}
	//
	
	

	//control loops
	
	UNTIL FALSE {
	
		LOCAL iterationT IS surfacestate["MET"].
		getState().
		
		IF (vehiclestate["ops_mode"] = 1) {
				// we're waiting for pdi
				set warp to 0.
				SAS OFF.
				
				IF (TIME:SECONDS >= vehicle["ign_t"] - 0.05) {
					addMessage("INITIATING POWERED DESCENT").
					
					LOCAL x IS setupUPFG(landing_state).
					SET upfgInternal[0] TO x[0].
					SET usc TO x[1].
					SET usc["lastvec"] TO vecYZ(SHIP:retrograde:FOREVECTOR).
					SET upfgInternal["tgo"] TO pd_tgo.
					
					SET control["refvec"] TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
					set control["steerdir"] TO aimAndRoll(vecYZ(usc["lastvec"]), control["refvec"], control["roll_angle"]). 

					SAS OFF.
					LOCK STEERING TO control["steerdir"].
					LOCK THROTTLE to throttleControl().
					SET SHIP:CONTROL:PILOTMAINTHROTTLE TO vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"].
					SET vehiclestate["ops_mode"] TO 2.
				}

		} ELSE IF ((vehiclestate["ops_mode"] = 2) OR (vehiclestate["ops_mode"] = 3))  {
			//upfg running mode
			IF usc["itercount"]=0 { //detects first pass or convergence lost
				WHEN usc["conv"]=1 THEN {
					addMessage("GUIDANCE CONVERGED IN " + usc["itercount"] + " ITERATIONS").
				}
			}
			
			//landing target redesignation
			IF (vehiclestate["ops_mode"] = 3) {
				IF (redesig_flag) {
					LOCAL fore_vec IS SHIP:VELOCITY:SURFACE:NORMALIZED.
					redesig_landing_tgt(fore_vec,redesig_flag).
					
					IF (upfgInternal["tgo"]<(final_apch_T+1)) {
						clearvecdraws().
						addMessage("PRESS AG9 TO SWITCH TO ATT HOLD").
						SET redesig_flag TO FALSE.
						//manual switch trigger
						ON AG9 {
							addMessage("ATT HOLD ENGAGED").
							SET att_hold_swch TO TRUE.
						}
					}
				}
				
				IF (att_hold_swch OR (usc["conv"]=1 AND upfgInternal["tgo"] < upfgFinalizationTime) OR SHIP:VELOCITY:SURFACE:MAG< 80) {
					SET vehiclestate["ops_mode"] TO 4.
					SET control["refvec"] TO SHIP:FACING:TOPVECTOR.
					set vehicle["roll"] TO 0.
					SET tgt_rate TO -ABS(landing_state["velocity"]).
				}
				
			} else {
				IF (usc["conv"]=1 AND upfgInternal["tgo"] < tgt_redesig_T) {
					SET upfgInternal TO resetUPFG(upfgInternal).
					
					SET vehiclestate["ops_mode"] TO 3.
					
					SET landing_state["mode"] TO 2.
					SET landing_state["altitude_bias"] TO 0.
					set landing_state["range_bias"] to 750.
					set landing_state["velocity_bias"] to 0.
					
					addMessage("TARGET SITE REDESIGNATION ENABLED").
					
				}
			}	
			
			SET upfgInternal TO upfg_wrapper(upfgInternal).
			
		} ELSE IF (vehiclestate["ops_mode"] = 4) {
			//pitchover mode
			
			IF (ALT:RADAR <= landing_state["altitude"]) {
				UNLOCK STEERING.
				SAS ON.
				SET vehiclestate["ops_mode"] TO 5.
				SET usc["lastthrot"] TO tgt_throt(0).
				
				drawUI().
				
				//trigger to engage/disengage velocity nulling
				ON SAS {
					IF SAS {
						UNLOCK STEERING. 
					}
					ELSE {
						LOCK STEERING TO control["steerdir"].
					}
					RETURN TRUE.
				}

			}
		
			LOCAL deltaT IS surfacestate["MET"] - iterationT.
		
			SET control["refvec"] TO SHIP:FACING:TOPVECTOR.
		
			LOCAL out IS pitchover(
				usc["lastthrot"],
				landing_state["altitude"]*0.95,
				tgt_rate,
				deltaT
			).
			
			SET usc["lastvec"] TO out[0].
			SET usc["lastthrot"] TO out[1].
			
			//progressively reduce throttle
			//SET usc["lastthrot"] TO usc["lastthrot"] - 0.0025.
		
		} ELSE IF vehiclestate["ops_mode"] >= 5 {
			//att hold mode
			
			SET control["refvec"] TO SHIP:FACING:TOPVECTOR.
			SET tgt_rate TO tgt_rate + delta_rate*SHIP:CONTROL:PILOTTOP.
			LOCAL d_a IS delta_accel(surfacestate["vs"],tgt_rate,rate_dt).
			SET usc["lastthrot"] TO tgt_throt(d_a).
			
			//if SAS is off, null surface velocity
			IF (NOT SAS) {
				SET usc["lastvec"] TO null_velocity(null_velocity_gain).
				
				//switch to ATT HOLD if control inputs are detected
				IF NOT ( SHIP:CONTROL:PILOTROTATION:MAG=0 ) {
					SAS ON.
				}
			} ELSE {
				//invert roll input 
				SET SHIP:CONTROL:ROLL TO -SHIP:CONTROL:PILOTROLL.
			}
			
			//break out
			IF (SHIP:STATUS="LANDED") {
				SET vehiclestate["ops_mode"] TO -1.
				BREAK.
			}
			
		}
		
		
		//steering and throttle control
		IF NOT vehiclestate["staging_in_progress"] {
			set control["steerdir"] TO aimAndRoll(vecYZ(usc["lastvec"]), control["refvec"], control["roll_angle"]). 
		}

		SET vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"] TO usc["lastthrot"].		
		
		dataViz().
		WAIT 0.
	}
	
	//shutdown
	
	SAS OFF.
	LOCK THROTTLE to 0.
	LIST ENGINES IN Eng.
	FOR E IN Eng {IF e:ISTYPE("engine") {E:SHUTDOWN.}}
	
	UNLOCK STEERING.
	UNLOCK THROTTLE.
	SET SHIP:CONTROL:PILOTMAINTHROTTLE TO 0.
	clearscreen.

}




init().