//Global vars

GLOBAL g0 IS 9.80665. 
GLOBAL g_body IS BODY:MU/(BODY:RADIUS^2).
GLOBAL vehicle_pre_staging_t IS 5.
GLOBAL vehicle_pre_conv_throt IS 0.8.
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



// taken from the launch script, except we only allow for constant thrust depletion stages
declare function initialise_vehicle{

	RUNPATH("0:/Libraries/resources_library").	

	
	FUNCTION add_resource {
		parameter lexx.
		parameter reslist.
		
		
		for res in reslist {
			IF NOT lexx:HASKEY(res) {lexx:ADD(res,0).}
		}
		RETURN lexx.
	}
	
	
	
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
	
	
	
	
	//local m_bias IS ship:mass.
	//SET m_bias TO m_bias - vehicle["stages"][1]["m_initial"].
	//IF m_bias<0.5 {
	
	//set m_bias to 0.
	//}

	//IF NOT vehicle:HASKEY("offaxis_thrust") {vehicle:ADD("offaxis_thrust",v(0,0,0)).}

	LOCAL vehlen IS vehicle["stages"]:LENGTH.

	FROM {LOCAL k IS 1.} UNTIL k > (vehlen - 1) STEP { SET k TO k+1.} DO{
	
		local stg IS vehicle["stages"][k].
	
	
		LOCAL iisspp IS 0.
		LOCAL tthrust IS 0.
		LOCAL fflow IS 0.
		local stage_res IS LEXICON().
		
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
				
				
				SET stage_res TO add_resource(stage_res,v["resources"]).
			}
			SET stg["engines"] TO LEXICON("thrust", tthrust, "isp", iisspp/tthrust, "flow", fflow).
			
			SET stage_res TO res_dens_init(stage_res).
		
			stg:ADD("resources",stage_res).
			
			fix_mass_params(stg).
		
			SET stg["m_initial"] 			TO stg["m_initial"]*1000.
			SET stg["m_final"] 			TO stg["m_final"]*1000.
			SET stg["m_burn"] 				TO stg["m_burn"]*1000.
			SET stg["engines"]["thrust"] 	TO stg["engines"]["thrust"]*1000.
		}
		
		IF NOT (stg:HASKEY("Throttle")) {stg:ADD("Throttle",1).}
	
		IF NOT (stg:HASKEY("Tstage")) {stg:ADD("Tstage",0).}
		//stg:ADD("ign_t", 0).
		
				
		//SET stg["m_initial"] TO stg["m_initial"] + m_bias.
		//SET stg["m_final"] TO stg["m_final"] + m_bias.

		IF NOT stg:HASKEY("mode") {	
			stg:ADD("mode", 1).	
		}
		
		
		IF (stg["staging"]["type"]="depletion") {
			SET stg["Tstage"] TO  const_f_t(stg).
		
		} else {
			clearscreen.
			PRINT ("ERROR! VEHICLE IS ONLY ALLOWED TO HAVE DEPLETON-TYPE STAGES") AT (1,5).
			LOCAL X IS 1/0.
		}
		
		
		
	}
	SET vehiclestate["m_burn_left"] TO vehicle["stages"][1]["m_burn"].
	
	vehicle:ADD("ign_t", 0).
	
	//required to be facing ahead at pitchover
	SET vehicle["roll"] TO 180.

	SET control["roll_angle"] TO vehicle["roll"].
	
	SET vehicle["ign_t"] TO TIME:SECONDS.
	
	SET vehicle["stages"][vehiclestate["cur_stg"]]["Throttle"] TO 0.8.
	
	set_staging_trigger().
	
	PRINT " INITIALISATION COMPLETE" AT (0,3).
	
	//debug_vehicle().
}


FUNCTION debug_vehicle {
	IF EXISTS("0:/vehicledump.txt") {
		DELETEPATH("0:/vehicledump.txt").
	}
	
	log vehicle:dump() to "0:/vehicledump.txt".
	
	until false{wait 0.1.}
}






									//VEHICLE PERFORMANCE & STAGING FUNCTIONS

FUNCTION get_stage {
	RETURN vehicle["stages"][vehiclestate["cur_stg"]].
}


FUNCTION get_TWR {
	RETURN vehiclestate["avg_thr"]:average()/(1000*SHIP:MASS*g_body).
}


FUNCTION get_stg_tanks {

	FUNCTION parts_tree {
		parameter part0.
		parameter partlist.
		parameter reslist.
	
		
		
		//UNTIL FALSE {
		//	wait 0.
		//	FOR partres IN parentpart:RESOURCES {
		//		IF reslist:KEYS:CONTAINS(partres:NAME) { SET breakflag TO TRUE.}
		//	}
		//	IF parentpart=CORE:PART { SET breakflag TO TRUE.}
		//	IF breakflag { BREAK.}
		//	SET parentpart TO parentpart:PARENT.
		//}
		
		FOR res IN reslist:KEYS {
			LOCAL  parentpart IS part0.
			local breakflag IS FALSE.
			UNTIL FALSE {
				wait 0.
				
				FOR partres IN parentpart:RESOURCES {
					IF res=partres:NAME { 
						IF NOT partlist:CONTAINS(parentpart) {partlist:ADD( parentpart ).}
						SET breakflag TO TRUE.
					}
				}
				
				
				IF parentpart=CORE:PART OR parentpart=SHIP:ROOTPART { SET breakflag TO TRUE.}
				IF breakflag { BREAK.}
				SET parentpart TO parentpart:PARENT.
			}
			
		}
		
		return partlist.
	}


	PARAMETER stg.

	local tanklist IS LIST().
	local reslist is stg["resources"].
	
	list ENGINES in all_eng.
	LOCAL parentpart IS 0.
	FOR e IN all_eng {
		IF e:ISTYPE("engine") {
			IF e:IGNITION {
				SET tanklist TO parts_tree(e:PARENT,tanklist,reslist).
			}
		}
	}
	
	//ignore fuel ducts if already found parts
	IF tanklist:LENGTH=0 {
		LOCAL duct_list IS SHIP:PARTSDUBBED("fuelLine").
		FOR d IN duct_list {
			SET tanklist TO parts_tree(d:PARENT,tanklist,reslist).
		}
	}
	stg:ADD("tankparts", tanklist).	
	
}

FUNCTION get_prop_mass {
	PARAMETER stg.
	
	local tanklist is stg["tankparts"].
	local reslist is stg["resources"].
	local prop_mass IS 0.
	
	FOR tk IN tanklist {
		FOR tkres In tk:RESOURCES {
			FOR res IN reslist:KEYS {
				IF tkres:NAME = res {
					set prop_mass TO prop_mass + tkres:amount*reslist[res].
				}
		
			}
		}
	}
	set prop_mass to prop_mass*1000.
    RETURN prop_mass.
}

//measures everything about the current state of the vehicle, including instantaneous thrust
//thrust only averaged over if staging is not in progress
FUNCTION getState {

	LOCAL deltat IS surfacestate["MET"].
	
	update_navigation().
	
	SET deltat TO surfacestate["MET"] - deltat.
	
	//measure and compute vehicle performance parameters
	
	local stg IS get_stage().
	
	LOCAL x IS get_current_thrust_isp().
	
	SET vehiclestate["thr_vec"] TO x[0].
	
	vehiclestate["avg_thr"]:update(vehiclestate["thr_vec"]:MAG).
	
	LOCAL avg_thrust is vehiclestate["avg_thr"]:average().
	LOCAL avg_isp is x[1].

	IF NOT stg:HASKEY("tankparts") {get_stg_tanks(stg).}
		
	LOCAL m_old IS stg["m_initial"].

	SET stg["m_initial"] TO SHIP:MASS*1000.
	
	LOCAL deltam IS m_old - stg["m_initial"].
	
	local res_left IS get_prop_mass(stg).
	
	SET vehiclestate["m_burn_left"] to res_left.
	
	IF NOT (vehiclestate["staging_in_progress"]) {
		
								 
		IF (stg["staging"]["type"]="m_burn") {
		
			SET stg["m_burn"] TO stg["m_burn"] - deltam.
			SET stg["m_final"] TO stg["m_initial"] -  stg["m_burn"].
			
		} ELSE IF (stg["staging"]["type"]="time") {
		
		    	SET stg["Tstage"] TO stg["Tstage"] - deltat.
				
		} ELSE IF (stg["staging"]["type"]="glim"){	
			
			SET stg["m_final"] TO stg["m_initial"] - res_left.
			
			LOCAL y IS glim_t_m(stg).
			
			SET stg["Tstage"] TO y[0].
			SET stg["m_final"] TO y[1].
			SET stg["m_burn"] TO stg["m_initial"] - y[1].
			
			//LOCAL nextstg IS vehicle["stages"][vehiclestate["cur_stg"]+1].
			//
			//SET nextstg["m_initial"] TO y[1].
			//
			//IF nextstg["mode"]=1 {
			//	SET nextstg["Tstage"] TO const_f_t(nextstg).
			//	SET nextstg["m_burn"] TO res_left - stg["m_burn"].
			//}
			//ELSE IF nextstg["mode"]=2 {
			//	
			//	SET nextstg["m_final"] TO z[1].
			//	
			//	LOCAL z IS const_G_t_m(nextstg).
			//	
			//	SET nextstg["Tstage"] TO z[0].
			//	SET nextstg["m_final"] TO z[1].
			//	SET nextstg["m_final"] TO z[1].
			//}
			
		}  ELSE IF (stg["mode"]=2){
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
}





//Staging function.
FUNCTION STAGING{
	
	local stg_staginginfo IS vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["type"].
	
	local flameout IS (stg_staginginfo="depletion").
	
	IF flameout {
		SET vehiclestate["staging_in_progress"] TO TRUE.
		SET P_steer TO "kill".
	}
	
	addMessage("CLOSE TO STAGING").
	SET vehiclestate["staging_time"] TO TIME:SECONDS+100.		//bias of 100 seconds to avoid premature triggering of the staging actions
	
	
	

	WHEN (flameout AND maxthrust=0) or ((NOT flameout) AND vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <=0.0005 ) THEN {	
		SET vehiclestate["staging_in_progress"] TO TRUE.
		addMessage("STAGING").
		SET vehiclestate["staging_time"] TO TIME:SECONDS.
		IF (vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["type"]="depletion") {set vehiclestate["staging_time"] to vehiclestate["staging_time"] + 1.5 .}
	}
	
	
	local stagingaction IS {STAGE.}.
	IF vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]:HASKEY("stg_action") {
		SET stagingaction TO vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]["stg_action"].
	}
	
	WHEN TIME:SECONDS > vehiclestate["staging_time"] THEN {
		staging_reset(stagingaction).	
	}
	
	IF vehicle["stages"][vehiclestate["cur_stg"]+1]["staging"]["ignition"]=TRUE {
		WHEN TIME:SECONDS > (vehiclestate["staging_time"] + vehicle["stages"][vehiclestate["cur_stg"]]["staging"]["ullage_t"]) THEN { 
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
		WHEN TIME:SECONDS > vehiclestate["staging_time"] + 0.5 THEN {
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
	SET stg["ign_t"] TO TIME:SECONDS.
	
	vehiclestate["avg_thr"]:reset().
	
	SET vehiclestate["m_burn_left"] TO stg["m_burn"].
	handle_ullage(stg).
	set_staging_trigger().
	IF vehiclestate["ops_mode"]=2 {SET usc["lastthrot"] TO stg["Throttle"].	}
}

FUNCTION set_staging_trigger {
	WHEN ( vehicle["stages"][vehiclestate["cur_stg"]]["Tstage"] <= vehicle_pre_staging_t AND vehiclestate["cur_stg"]< (vehicle["stages"]:LENGTH - 1) ) THEN {
		STAGING().
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
	PARAMETER h0.
	parameter hdot0.
	parameter dt.
	
	//prevent the final latitude from being too close to the ground
	SET h0 TO MAX(60,h0).
	
	//prevents positive vertical velocity
	SET hdot0 TO MIN(0,hdot0).
	
	
	LOCAL hdot IS SHIP:VERTICALSPEED.
	LOCAL dh IS ALT:RADAR - h0.
	
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


