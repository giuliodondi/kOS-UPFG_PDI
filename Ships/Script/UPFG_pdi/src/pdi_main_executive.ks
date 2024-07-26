@LAZYGLOBAL OFF.
CLEARSCREEN.
SET CONFIG:IPU TO 1200.	
global debug_mode is true.
global dap_debug is true.

GLOBAL log_data Is false.
GLOBAL debug Is false.

function pdi_main_exec {
	CLEARSCREEN.
	clearvecdraws().
	
	STEERINGMANAGER:RESETPIDS().
	STEERINGMANAGER:RESETTODEFAULT().
	SET STEERINGMANAGER:ROLLTS TO 1.2.
	SET SteeringManager:MAXSTOPPINGTIME TO 0.75.
	//setup_steering_manager().

	//	Load libraries
	RUNPATH("0:/Libraries/misc_library").	
	RUNPATH("0:/Libraries/maths_library").	
	RUNPATH("0:/Libraries/navigation_library").	
	RUNPATH("0:/Libraries/vehicle_library").	
	RUNPATH("0:/Libraries/aerosim_library").	
	
	RUNPATH("0:/UPFG_pdi/src/pdi_interface_library").
	RUNPATH("0:/UPFG_pdi/src/pdi_gui_library").
	RUNPATH("0:/UPFG_pdi/src/pdi_targeting_library").
	RUNPATH("0:/UPFG_pdi/src/pdi_upfg_library").
	RUNPATH("0:/UPFG_pdi/src/pdi_vehicle_library").
	
	initialise_sites_file().
	
	make_main_pdi_gui().
	
	//	Load vessel file
	IF (vesselfilename:ENDSWITH(".ks")=TRUE) {
		SET vesselfilename TO vesselfilename:REMOVE( vesselfilename:FIND(".ks") ,3 ).
	}
	RUNPATH("0:/UPFG_pdi/VESSELS/" + vesselfilename + ".ks").
	
	
	wait until ship:unpacked and ship:loaded.
	
	GLOBAL tgt_rate IS 0.
	GLOBAL rate_dt IS 0.5.
	GLOBAL delta_rate IS 0.25.
	//GLOBAL delta_shift IS 5000.
	GLOBAL null_velocity_gain IS 4.
	//SET delta_shift TO delta_shift/SHIP:BODY:RADIUS.
	
	initialise_vehicle().
	
	GLOBAL dap IS pdi_dap_factory().
	
	GLOBAL dap_gui_executor IS loop_executor_factory(
												0.15,
												{
													//protection
													if (SAS) {
														SAS OFF.
													}
													
													if (is_dap_auto()) {
														dap:steer_auto_thrvec().
														dap:thr_control_auto().
													} else if (is_dap_css()) {
														dap:steer_css().
														dap:thr_control_css().
													}
													
																										
													if (dap_debug) {
														//clearscreen.
														clearvecdraws().
														
														dap:print_debug(2).
														
														arrow_ship(3 * dap:steer_thrvec,"steer_thrvec").
														arrow_ship(2 * dap:steer_dir:forevector,"forevec").
														arrow_ship(2 * dap:steer_dir:topvector,"topvec").
													
													}
													
													//dataViz().
												}
	).
	
	pdi_countdown_loop().
	
	//pdi_main_loop().
	
	clearscreen.
	clearvecdraws().
	close_all_GUIs().
}

function pdi_countdown_loop {

	//to bypass iteration time of UPFG during pre-convergence
	SET vehiclestate["staging_in_progress"] TO TRUE.

	
	LOCK STEERING TO dap:steer_dir.
	
	SET vehiclestate["ops_mode"] TO 1.

	
	GLOBAL att_hold_swch IS FALSE.
	
	
	UNTIL FALSE {
		if (quit_program) {
			return.
		}

		getState().
	
		if (not landing_state["pre_converged"]) {
		
			addMessage("TARGET SELECTED : " + landing_state["tgtsite_name"]).
			addMessage("RE-CONVERGING POWERED DESCENT").
		
			pre_converge_guidance().
			
			if (landing_state["pre_conv_interrupt"]) {
				set landing_state["pre_converged"] to false.
				set landing_state["pre_conv_interrupt"] to false.
				
			} else {


				dap:set_steer_tgt(vecYZ(upfgInternal["steering"])).
				set dap:thr_rpl_tgt to 0.
				
				
				//save steering info at pdi 
				//SET usc["lastvec"] TO - orbitstate["velocity"]:NORMALIZED.
				//SET control["refvec"] TO vecYZ(orbitstate["radius"]).


				//drawUI().
				
				//dataViz().

				//initialise initial orientation
				
				//set control["steerdir"] TO aimAndRoll(vecYZ(usc["lastvec"]), control["refvec"], control["roll_angle"]). 
				//LOCK STEERING TO control["steerdir"].
				
				//WHEN (surfacestate["time"] > vehicle["ign_t"] - 20) THEN {
				//	addMessage("PREPARE FOR POWERED DESCENT").
				//}
				
				set landing_state["pre_converged"] to true.
			}

		}
			
	
		//IF (surfacestate["time"] > vehicle["ign_t"] - 65) {
		//	set warp to 0.
		//	SAS OFF.
		//	RCS ON.
		//	SET control["refvec"] TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
		//	LOCK STEERING TO control["steerdir"].
		//}
	
	}

}


FUNCTION pdi_main_loop {

	
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
				warp_controller(vehicle["ign_t"] - TIME:SECONDS, FALSE, 60).
				
				IF TIME:SECONDS > vehicle["ign_t"] - (60*warp) {
					set warp to 0.
					SAS OFF.
					RCS ON.
				
					IF (TIME:SECONDS >= vehicle["ign_t"] - 0.05) {
						addMessage("INITIATING POWERED DESCENT").
						
						SET vehiclestate["staging_in_progress"] TO FALSE.
						
						resetUPFG(upfgInternal).
						SET landing_state["mode"] TO 0.
						
						SET control["refvec"] TO -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
						set control["steerdir"] TO aimAndRoll(vecYZ(usc["lastvec"]), control["refvec"], control["roll_angle"]). 

						SAS OFF.
						LOCK STEERING TO control["steerdir"].
						LOCK THROTTLE to throttleControl().
						SET SHIP:CONTROL:PILOTMAINTHROTTLE TO vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"].
						SET vehiclestate["ops_mode"] TO 2.
					}
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
					
					LOCAL delta_shift IS 0.3 * landing_state["tgtsite"]:DISTANCE / SHIP:BODY:RADIUS.
					
					redesig_landing_tgt(delta_shift, fore_vec,redesig_flag).
					
					//IF (upfgInternal["tgo"]<(landing_state["final_apch_t"] + 1)) {
					//	clearvecdraws().
					//	addMessage("PRESS AG9 TO SWITCH TO ATT HOLD").
					//	SET redesig_flag TO FALSE.
					//	//manual switch trigger
					//	ON AG9 {
					//		addMessage("ATT HOLD ENGAGED").
					//		SET att_hold_swch TO TRUE.
					//	}
					//}
				}
				
				IF (att_hold_swch OR SHIP:VELOCITY:SURFACE:MAG < 60) {	// OR (usc["conv"]=1 AND upfgInternal["tgo"] < upfgFinalizationTime)
					SET vehiclestate["ops_mode"] TO 4.
					SET control["refvec"] TO SHIP:FACING:TOPVECTOR.
					SET tgt_rate TO -ABS(landing_state["velocity"]).
					drawUI().
				}
				
			} else {
				IF (usc["conv"]=1 AND upfgInternal["tgo"] < landing_state["tgt_redesig_t"]) {
					SET upfgInternal TO resetUPFG(upfgInternal).
					
					SET vehiclestate["ops_mode"] TO 3.
					
					//SET landing_state["mode"] TO 2.
					SET landing_state["altitude_bias"] TO 0.
					set landing_state["range_bias"] to 1000.
					set landing_state["velocity_bias"] to 0.
					
					SET redesig_flag TO TRUE.
					
					addMessage("TARGET SITE REDESIGNATION ENABLED").
					
				}
			}	
			
			SET upfgInternal TO upfg_wrapper(upfgInternal).
			
		} ELSE IF (vehiclestate["ops_mode"] = 4) {
			//pitchover mode
			SET control["roll_angle"] TO 0.
			
			LOCAL tgt_rel_alt IS cur_alt_above_target(landing_state).
			
			IF (tgt_rel_alt <= landing_state["altitude"]) {
				UNLOCK STEERING.
				SAS ON.
				SET vehiclestate["ops_mode"] TO 5.
				SET usc["lastthrot"] TO tgt_throt(0).
				
				SET tgt_rate TO -0.5.
				
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
				tgt_rel_alt,
				landing_state["altitude"],
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
			SET SHIP:CONTROL:PILOTMAINTHROTTLE TO usc["lastthrot"].
						
			
			//if SAS is off, null surface velocity
			IF (NOT SAS) {
				SET usc["lastvec"] TO null_velocity(null_velocity_gain).
				SET control["roll_angle"] TO 0.
				
				//switch to ATT HOLD if control inputs are detected
				IF NOT ( SHIP:CONTROL:PILOTROTATION:MAG=0 ) {
					SAS ON.
				}
			} ELSE {
				//invert roll input 
				LOCAL delta_roll IS -SHIP:CONTROL:PILOTROLL.
				SET control["roll_angle"] TO control["roll_angle"] + delta_roll.
				SET SHIP:CONTROL:ROLL TO delta_roll.
			}
			
			//break out
			IF (SHIP:STATUS="LANDED") {
				SET vehiclestate["ops_mode"] TO -1.
				BREAK.
			}
			
		}
		
		if (redesig_flag) {
			redraw_target_arrow().
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
	

}




pdi_main_exec().