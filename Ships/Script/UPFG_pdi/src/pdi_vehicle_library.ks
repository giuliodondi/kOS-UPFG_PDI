//Global vars

GLOBAL g0 IS 9.80665. 
GLOBAL vehicle_pre_staging_t IS 5.
GLOBAL vehicle_const_f_dt IS 600.
GLOBAL vehicle_const_a_glim IS 0.5.	//has to be in earth gs even around the moon
GLOBAL vehicle_pre_conv_throt IS 0.8.
GLOBAL vehicle_constg_initial_throt IS 0.35.
//tilt angle limit from vertical during pitchdown and att hold modes
gLOBAL vehicle__atthold_anglelim IS 20.


GLOBAL vehiclestate IS LEXICON(
	"ops_mode",0,
	"cur_stg", 1,
	"staging_time", 0,
	"staging_in_progress", FALSE,
	"m_burn_left", 0,
	"avg_thr", average_value_factory(6)
).


GLOBAL control Is LEXICON(
	"steerdir", LOOKDIRUP(SHIP:FACING:FOREVECTOR, SHIP:FACING:TOPVECTOR),
	"roll_angle",0,
	"refvec", v(0,0,0)
).

//instead of reading the vehicle config from file, build it at runtime
function setup_standard_vehicle {
	
	GLOBAL vehicle IS LEXICON().
	
	vehicle:add("roll", 0).
	
	local full_englex is get_running_englex().
	
	local englex is LEXICON(
				"thrust", full_englex["thrust"], 
				"isp", full_englex["isp"], 
				"flow",full_englex["flow"],
				"minThrottle", full_englex["minThrottle"]
	).
	
	local stg1_m_initial is SHIP:mass.
	
	local stageslex is LIST(0).
	
	local stg1 is LEXICON(
			"m_initial",stg1_m_initial,
			"m_final",	0,
			"m_burn",0
			"staging", LEXICON (
				"type","time",
				"ignition",	TRUE,
				"ullage", "none",
				"ullage_t",	0	
			),
			"mode", 1,
			"throttle", vehicle_pre_conv_throt,
			"Tstage",vehicle_const_f_dt,
			"engines",	LIST(englex)					
	).
	
	set stg1["m_final"] to const_f_dt_mfinal(stg1).
	set stg1["m_burn"] to stg1["m_initial"] - stg1["m_final"].
	
	IF NOT stg1:HASKEY("tankparts") {get_stg_tanks_res(stg1).}
	
	local res_left IS get_prop_mass(stg1).	
	set res_left to res_left - stg1["m_burn"].
	
	local stg2 is LEXICON(
			"m_initial",stg1["m_final"],
			"m_final",	0,
			"m_burn",0
			"staging", LEXICON (
				"type","depletion",
				"ignition",	FALSE,
				"ullage", "none",
				"ullage_t",	0	
			),
			"mode", 2,
			"glim", vehicle_const_a_glim,
			"throttle", vehicle_pre_conv_throt,
			"Tstage",0,
			"engines",	LIST(englex)	,
			"tankparts", stg1["tankparts"]		
	).
	
	SET stg2["m_final"] TO stg2["m_initial"] - res_left.
	LOCAL y IS const_G_t_m(stg2).
	SET stg2["Tstage"] TO y[0].
	SET stg2["m_final"] TO y[1].
	SET stg2["m_burn"] TO stg2["m_initial"] - y[1].
	
	stageslex:add(stg1).
	stageslex:add(stg2).
	
	vehicle:add("stages", stageslex).

	//debug_vehicle().
}


// taken from the launch script, except we only allow for constant thrust depletion stages
declare function initialise_vehicle{
	
	
	FUNCTION fix_mass_params {
		parameter stg.
		
		IF stg:HASKEY("m_burn") {
			IF stg:HASKEY("m_initial") {
				IF NOT stg:HASKEY("m_final") {
					stg:ADD("m_final",0).
					SET stg["m_final"] TO stg["m_initial"] - stg["m_burn"].
				}
			}
			ELSE IF stg:HASKEY("m_final") {
				IF NOT stg:HASKEY("m_initial") {
					stg:ADD("m_initial",0).
					SET stg["m_initial"] TO stg["m_final"] + stg["m_burn"].
				}
			}
		}
		ELSE IF stg:HASKEY("m_initial") AND stg:HASKEY("m_final") {
			IF NOT stg:HASKEY("m_burn") {
				stg:ADD("m_burn",0).
				SET stg["m_burn"] TO stg["m_initial"] - stg["m_final"].
			}
		}
		ELSE {
			PRINT ("ERROR! VEHICLE MASS PARAMETERS ILL-DEFINED") AT (1,40).
			LOCAL X IS 1/0.
		}
	}
	
	
	LOCAL vehlen IS vehicle["stages"]:LENGTH.

	FROM {LOCAL k IS 1.} UNTIL k > (vehlen - 1) STEP { SET k TO k+1.} DO{
	
		local stg IS vehicle["stages"][k].
	
	
		LOCAL iisspp IS 0.
		LOCAL tthrust IS 0.
		LOCAL minthrust IS 0.
		local can_throttle IS FALSE.
		LOCAL fflow IS 0.
		
		IF (stg["engines"]:ISTYPE("LIST")) {
			//trigger that we should do the initial stage parsing 
			FOR v in stg["engines"] {
				SET tthrust TO tthrust + v["thrust"].
				SET iisspp TO iisspp + v["isp"]*v["thrust"].
				
				IF (v:HASKEY("flow")) {
					SET fflow TO fflow + v["flow"].
				} ELSE {
					SET fflow TO fflow + v["thrust"] * 1000 / (v["isp"] * g0).
				}
				
				IF (v:HASKEY("minThrust")) {
					SET can_throttle TO TRUE.
					SET minthrust TO minthrust + v["minThrust"].
				} else {
					SET minthrust TO minthrust + v["thrust"].
				}
			}
			
			SET stg["engines"] TO LEXICON("thrust", tthrust, "isp", iisspp/tthrust, "flow", fflow).
			
			if (can_throttle) {
				stg["engines"]:ADD("minThrottle", minthrust/tthrust).
			}
			
			fix_mass_params(stg).
		
			SET stg["m_initial"] 			TO stg["m_initial"]*1000.
			SET stg["m_final"] 			TO stg["m_final"]*1000.
			SET stg["m_burn"] 				TO stg["m_burn"]*1000.
			SET stg["engines"]["thrust"] 	TO stg["engines"]["thrust"]*1000.
		}
		
		IF NOT (stg:HASKEY("Throttle")) {stg:ADD("Throttle", vehicle_pre_conv_throt).}
	
		IF NOT (stg:HASKEY("Tstage")) {stg:ADD("Tstage",0).}
		
		IF NOT stg:HASKEY("mode") {	
			stg:ADD("mode", 1).	
		}
		
		IF stg["staging"]["type"]="time" {
			SET stg["Tstage"] TO stg["Tstage"].
			
			
		} ELSE IF (stg["staging"]["type"]="depletion") {
		
			IF (stg["mode"] = 1) {
				//regular constant thrust depletion stage 
				SET stg["Tstage"] TO  const_f_t(stg).
			} ELSE IF (stg["mode"] = 2) {
				//g-limited stage, figure out if we need to add a minimum constant throttle depletion stage
				
				LOCAL y IS const_G_t_m(stg).
				SET stg["Tstage"] TO y[0].
				
				LOCAL newstg_m_initial IS y[1].
				LOCAL newstg_m_final IS stg["m_final"].
				
				//only add a new stage if it burns for at least 3 seconds 
				LOCAL min_newstg_m_initial IS newstg_m_final + 3 * stg["engines"]["minThrottle"] * stg["engines"]["flow"].
				
				//print "newstg_m_initial " + newstg_m_initial/1000 at (0,29).
				//print "min_newstg_m_initial " + min_newstg_m_initial/1000 at (0,30).
				//print "newstg_m_final " + newstg_m_final/1000 at (0,31).
				
				IF (newstg_m_initial > min_newstg_m_initial) {
					
					SET stg["m_final"] TO newstg_m_initial.
					SET stg["m_burn"] TO stg["m_initial"] - newstg_m_initial.
					SET stg["staging"]["type"] TO "minthrot".
					SET stg["staging"]["stg_action"] TO {}.
					
					//create the new stage 
					
					LOCAL new_stg  IS LEXICON(
												"m_initial",	newstg_m_initial,
												"m_final",	newstg_m_final,
												"m_burn", newstg_m_initial - newstg_m_final,
												"staging", LEXICON (
															"stg_action",{},
															"type","depletion",
															"ignition",	FALSE,
															"ullage", "none",
															"ullage_t",	0
												),
												"engines",	stg["engines"],
												"Tstage",0,
												"mode", 1,
												"glim",stg["glim"],
												"Throttle",stg["engines"]["minThrottle"]
										).
										
					vehicle["stages"]:INSERT(k+1, new_stg).
					SET vehlen TO vehicle["stages"]:LENGTH.
				}
			}
		}
		
		
		
	}
	SET vehiclestate["m_burn_left"] TO vehicle["stages"][1]["m_burn"].
	
	vehicle:ADD("ign_t", 0).
	
	//required to be facing ahead at pitchover
	SET vehicle["roll"] TO 180.

	SET control["roll_angle"] TO vehicle["roll"].
	
	SET vehicle["ign_t"] TO TIME:SECONDS.
	
	set_staging_trigger().
	
	PRINT " INITIALISATION COMPLETE" AT (0,3).
	
	//debug_vehicle().
}


FUNCTION debug_vehicle {
	
	dump_vehicle().
	
	until false{
		print "vehicle debug, press crtl-c to quit" at (0,2).
		wait 0.1.
	}
}


FUNCTION dump_vehicle {
	IF EXISTS("0:/vehicledump.txt") {
		DELETEPATH("0:/vehicledump.txt").
	}
	
	log vehicle:dump() to "0:/vehicledump.txt".
}






									//VEHICLE PERFORMANCE & STAGING FUNCTIONS


FUNCTION get_stage {
	RETURN vehicle["stages"][vehiclestate["cur_stg"]].
}


FUNCTION get_TWR {
	RETURN vehiclestate["avg_thr"]:average()/(1000*SHIP:MASS*bodygravacc()).
}

//measures everything about the current state of the vehicle, including instantaneous thrust
//thrust only averaged over if staging is not in progress
FUNCTION getState {
	
	update_navigation().
	
	IF DEFINED events {	events_handler().}
	
	//measure and compute vehicle performance parameters
	
	local stg IS get_stage().
	
	LOCAL x IS get_current_thrust_isp().
	
	SET vehiclestate["thr_vec"] TO x[0].
	
	vehiclestate["avg_thr"]:update(vehiclestate["thr_vec"]:MAG).
	
	LOCAL avg_thrust is vehiclestate["avg_thr"]:average().
	LOCAL avg_isp is x[1].
		
	LOCAL m_old IS stg["m_initial"].

	SET stg["m_initial"] TO SHIP:MASS*1000.
	
	LOCAL deltam IS m_old - stg["m_initial"].
	
	IF NOT (vehiclestate["staging_in_progress"]) {
	
		IF NOT stg:HASKEY("tankparts") {get_stg_tanks_res(stg).}
		local res_left IS get_prop_mass(stg).
		SET vehiclestate["m_burn_left"] to res_left.
								 
		IF (stg["staging"]["type"]="m_burn") {
		
			SET stg["m_burn"] TO stg["m_burn"] - deltam.
			SET stg["m_final"] TO stg["m_initial"] -  stg["m_burn"].
			
		} ELSE IF (stg["staging"]["type"]="time") {
		
		    	SET stg["Tstage"] TO stg["Tstage"] - surfacestate["deltat"].
				
		} ELSE IF (stg["mode"]=2 AND stg["staging"]["type"]="depletion"){
			//both depletion and minthrot stages
			
			SET stg["m_final"] TO stg["m_initial"] - res_left.
			LOCAL y IS const_G_t_m(stg).
			SET stg["Tstage"] TO y[0].
			SET stg["m_final"] TO y[1].
			SET stg["m_burn"] TO stg["m_initial"] - y[1].
			
			//LOCAL nextstg IS vehicle["stages"][vehiclestate["cur_stg"]+1].
			//
			//SET nextstg["m_initial"] TO y[1].
			//SET nextstg["Tstage"] TO const_f_t(nextstg).
			
		}ELSE IF (stg["mode"]=1 AND stg["staging"]["type"]="depletion") {

			SET stg["m_burn"] TO res_left.
			SET stg["m_final"] TO stg["m_initial"] -  res_left.
			SET stg["Tstage"] TO const_f_t(stg).
			 
		}
	}	
	
	if (engine_flameout()) {
		//this should force staging if we detect flameout and it hasn't been triggered yet 
		SET stg["Tstage"] TO 0.
	}
	
	if (debug_mode) {
		dump_vehicle().
	}
}





//Staging function.
FUNCTION STAGING{

	LOCAL depletion_ is (vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["type"]="depletion").
	
	IF depletion_ {
		SET vehiclestate["staging_in_progress"] TO TRUE.
	}
	
	addMessage("CLOSE TO STAGING").
	SET vehiclestate["staging_time"] TO surfacestate["time"]+100.		//bias of 100 seconds to avoid premature triggering of the staging actions

	WHEN (vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <= 0.01) THEN {	
		SET vehiclestate["staging_in_progress"] TO TRUE.
		addMessage("STAGING").
		SET vehiclestate["staging_time"] TO surfacestate["time"].
		IF (vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["type"]="depletion") {set vehiclestate["staging_time"] to vehiclestate["staging_time"] + 1.5 .}
	}
	
	
	local stagingaction IS {STAGE.}.
	IF vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]:HASKEY("stg_action") {
		SET stagingaction TO vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]["stg_action"].
	}
	
	WHEN (surfacestate["time"] > vehiclestate["staging_time"]) THEN {
		staging_reset(stagingaction).	
	}
	
	IF vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]["ignition"]=TRUE {
		WHEN (surfacestate["time"] > (vehiclestate["staging_time"] + vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["ullage_t"])) THEN { 
			wait until stage:ready.
			
			STAGE.
			
			addMessage("SPOOLING UP").
			WHEN vehiclestate["avg_thr"]["latest_value"]>= 0.95*vehicle["stages"][vehiclestate["cur_stg"]]["engines"]["thrust"] THEN {
				addMessage("STAGING SEQUENCE COMPLETE").
				SET vehiclestate["staging_in_progress"] TO FALSE.
			}
		}
	}
	ELSE {
		WHEN (surfacestate["time"] > vehiclestate["staging_time"] + 0.5) THEN {
			addMessage("STAGING SEQUENCE COMPLETE").
			SET vehiclestate["staging_in_progress"] TO FALSE.
		}
	}
}


FUNCTION staging_reset {

	FUNCTION handle_ullage {
		PARAMETER stg.
	
		IF stg["staging"]["ullage"]="none" OR stg["staging"]["ullage_t"]=0 {
			RETURN.
		}
		addMessage("ULLAGE THRUST").
		IF stg["staging"]["ullage"]="srb"{
			RETURN.
		}
		ELSE IF stg["staging"]["ullage"]="rcs"{
			RCS ON. 
			SET SHIP:CONTROL:FORE TO 1.0.
			WHEN TIME:SECONDS > (vehiclestate["staging_time"] + stg["staging"]["ullage_t"]+1) THEN {
				SET SHIP:CONTROL:FORE TO 0.0.
			}
		}
	}

	PARAMETER stagingaction.
	wait until stage:ready.
	
	stagingaction:call().
		
	SET vehiclestate["cur_stg"] TO vehiclestate["cur_stg"]+1.
	local stg is get_stage().
	SET stg["ign_t"] TO surfacestate["time"].
	
	vehiclestate["avg_thr"]:reset().
	
	SET vehiclestate["m_burn_left"] TO stg["m_burn"].
	handle_ullage(stg).
	set_staging_trigger().
	//this is necessary to reset throttle if we have g-throttling before another stage
	set upfgInternal["throtset"] to stg["Throttle"].
}

FUNCTION set_staging_trigger {

	WHEN (vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <= vehicle_pre_staging_t) THEN {
		if (vehiclestate["cur_stg"]< (vehicle["stages"]:LENGTH - 1)) {
			STAGING().
		} else {
			set vehicle["low_level"] to TRUE.
		}
	}
}


									//VEHICLE CONTROL FUNCTIONS


FUNCTION setup_steering_manager {
	STEERINGMANAGER:RESETTODEFAULT().
	SET SteeringManager:MAXSTOPPINGTIME TO 4.
	SET STEERINGMANAGER:ROLLCONTROLANGLERANGE TO 4.0.
	SET STEERINGMANAGER:PITCHTS TO 4.0.
	SET STEERINGMANAGER:YAWTS TO 4.0.
	SET STEERINGMANAGER:ROLLTS TO 3.0.
	SET STEERINGMANAGER:PITCHPID:KD TO 0.2.
	SET STEERINGMANAGER:YAWPID:KD TO 0.2.
	SET STEERINGMANAGER:ROLLPID:KD TO 0.2.
}


//	Throttle controller for UPFG
FUNCTION throttleControl {

	local stg IS get_stage().
	local throtval is stg["Throttle"].

	LOCAL minthrot IS 0.
	IF stg:HASKEY("minThrottle") {
		SET minthrot TO stg["minThrottle"].
	}
	
	RETURN throtteValueConverter(stg["Throttle"], minthrot).
}

//used in transition between UPFG and hover phases
//calculates required acceleration to achieve target vertical speed by a target altitude
//acceleration is a function of pitch and throttle setting together
//calculates throttling rate to achieve the hovering pitch limit by the end of the braking
//calculates current pitch angle and steering vector

FUNCTION pitchover {
	PARAMETER k0.
	PARAMETER tgt_h.
	PARAMETER h0.
	parameter hdot0.
	parameter dt.
	
	//prevent the final latitude from being too close to the ground
	SET h0 TO MAX(60,h0).
	
	//prevents positive vertical velocity
	SET hdot0 TO MIN(0,hdot0).
	
	
	LOCAL hdot IS SHIP:VERTICALSPEED.
	LOCAL dh IS tgt_h - h0.
	
	//get the vehicle's thrust
	local stg IS get_stage().
	
	LOCAL g_acc IS SHIP:BODY:MU/(SHIP:ORBIT:BODY:POSITION:MAG)^2.
	LOCAL m IS stg["m_initial"].
	LOCAL T IS stg["engines"]["thrust"].
	
	LOCAL total_acc IS ( hdot^2 - hdot0^2 )/(2*dh).

	
	LOCAL q_ IS ( total_acc + g_acc)*m/T.
	
	LOCAL kf IS q_/COS(vehicle__atthold_anglelim).
	
	LOCAL kdot IS (kf - k0)*total_acc/ABS( hdot - hdot0) .
	
	SET k0 TO k0 + CLAMP(kdot*dt,-0.1,0.1).
	
	LOCAL aalpha IS ABS(ARCCOS(limitarg(q_/k0))).
	
	//IF (aalpha > vehicle__atthold_anglelim) {
	//	SET aalpha TO vehicle__atthold_anglelim.
	//	SET throtval TO q_/COS(aalpha).
	//}
	
	LOCAL leftvec IS -VCRS(-SHIP:ORBIT:BODY:POSITION:NORMALIZED, SHIP:VELOCITY:SURFACE:NORMALIZED ).
	LOCAL out_f IS rodrigues(-SHIP:ORBIT:BODY:POSITION:NORMALIZED, leftvec, aalpha).

	//return the unit thrust direction in UPFG coordinates
	RETURN LIST(vecYZ(out_f), k0).
}


// feedback acceleration to reach target descent rate (for att hold)
FUNCTION delta_accel {
	PARAMETER rate.
	PARAMETER tgt_rate.
	PARAMETER dt.
	
	RETURN (tgt_rate - rate)/dt.

}
// computes target acceleration (gravity + correction)
// and throttle precentage that achieves it.
FUNCTION tgt_throt {
	PARAMETER delta_a.
	
	local stg IS get_stage().
	
	LOCAL tgt_F IS delta_a + SHIP:BODY:MU/(SHIP:ORBIT:BODY:POSITION:MAG)^2.
	
	SET tgt_F TO tgt_F*stg["m_initial"].
	
	//figure out the total thrust required given vehicle inclination wrt the up direction
	LOCAL upv IS -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	LOCAL thrdir IS SHIP:FACING:FOREVECTOR.
	LOCAL alpha_ IS VANG(upv,thrdir).
	
	SET tgt_F TO tgt_F/COS(alpha_).
	
	RETURN tgt_F/stg["engines"]["thrust"].
}



//computes a unit thrust direction with the vertical component along gravity 
//and the horizontal component opposite to the horiz velocity.
FUNCTION null_velocity {
	PARAMETER dT.

	//find the horizontal velocity to cancel
	LOCAL srf_v IS SHIP:VELOCITY:SURFACE.
	LOCAL up_v IS -SHIP:ORBIT:BODY:POSITION:NORMALIZED.
	LOCAL horiz_v IS VXCL(up_v,srf_v).
	
	//vertical component of force is assumed to balance gravity
	LOCAL vert_F_mod IS SHIP:BODY:MU/(SHIP:ORBIT:BODY:POSITION:MAG)^2.
	LOCAL vert_F IS up_v*vert_F_mod.
	
	//horizontal component is opposite to the horizontal velocity and depends on a gain
	LOCAL horiz_F IS  -horiz_v/dT.
	
	//limit deviation from the vertical to anglelim
	SET horiz_F TO horiz_F:NORMALIZED*MIN(horiz_F:MAG,vert_F_mod*TAN(vehicle__atthold_anglelim)).
	
	//return the unit thrust direction in UPFG coordinates
	LOCAL out_f IS (horiz_F + vert_F):NORMALIZED.
	RETURN vecYZ(out_f).
}


