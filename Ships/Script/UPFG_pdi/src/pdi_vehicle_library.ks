//Global vars

GLOBAL g0 IS 9.80665. 
GLOBAL vehicle_pre_staging_t IS 5.
GLOBAL vehicle_const_f_dt IS 520.
GLOBAL vehicle_const_a_glim IS 0.18.	//has to be in earth gs even around the moon
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
	
	local stg1_m_initial is SHIP:mass*1000.
	
	local stageslex is LIST(0).
	
	vehicle:add("stages", stageslex).
	
	local stg1 is LEXICON(
			"m_initial",stg1_m_initial,
			"m_final",	0,
			"m_burn",0,
			"staging", LEXICON (
				"type","time",
				"ignition",	TRUE,
				"ullage", "none",
				"ullage_t",	0	
			),
			"mode", 1,
			"throttle", vehicle_pre_conv_throt,
			"Tstage",vehicle_const_f_dt,
			"engines",	englex				
	).
	
	set stg1["m_final"] to const_f_dt_mfinal(stg1).
	set stg1["m_burn"] to stg1["m_initial"] - stg1["m_final"].
	
	IF NOT stg1:HASKEY("tankparts") {get_stg_tanks_res(stg1).}
	
	local res_left IS get_prop_mass(stg1).	
	set res_left to res_left - stg1["m_burn"].
	
	stageslex:add(stg1).
	
	local stg2 is LEXICON(
			"m_initial",stg1["m_final"],
			"m_final",	0,
			"m_burn",0,
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
			"engines",	englex	,
			"tankparts", stg1["tankparts"]		
	).
	
	SET stg2["m_final"] TO stg2["m_initial"] - res_left.
	LOCAL y IS const_G_t_m(stg2).
	SET stg2["Tstage"] TO y[0].
	SET stg2["m_final"] TO y[1].
	SET stg2["m_burn"] TO stg2["m_initial"] - y[1].
	
	
	stageslex:add(stg2).
	
	//required to be facing ahead at pitchover
	SET vehicle["roll"] TO 180.

	SET control["roll_angle"] TO vehicle["roll"].
	
	//prevent events from triggering before PDI
	set vehicle["ign_t"] to TIME:SECONDS + 100000.
	
	set_staging_trigger().

	debug_vehicle().
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


//thr_rpl will refer to the full rated power level of the engine where 0 throttle is 0% force
//unlike the ksp throttle where 0% is the minimum
function pdi_dap_factory {

	STEERINGMANAGER:RESETTODEFAULT().
	SET STEERINGMANAGER:ROLLCONTROLANGLERANGE TO 4.0.


	LOCAL this IS lexicon().
	
	this:add("steer_mode", "").
	this:add("thr_mode", "").		//ksp 0-1 throttle value
	
	this:add("thr_cmd", 1).
	this:add("thr_cmd_rpl", 1).
	
	this:add("last_time", TIME:SECONDS).
	
	this:add("iteration_dt", 0).
	
	this:add("update_time",{
		LOCAL old_t IS this:last_time.
		SET this:last_time TO TIME:SECONDS.
		SET this:iteration_dt TO this:last_time - old_t.
	}).
	
	this:add("cur_dir", SHIP:FACINg).
	this:add("cur_thrvec", v(0,0,0)).
	
	this:add("cur_steer_pitch", 0).
	this:add("cur_steer_az", 0).
	this:add("cur_steer_roll", 0).
	
	this:add("steer_pitch_delta", 0).
	this:add("steer_yaw_delta", 0).
	this:add("steer_roll_delta", 0).
	this:add("throt_delta", 0).
	
	this:add("steer_dir", SHIP:FACINg).
	
	this:add("measure_refv_roll", {
		LOCAL refv IS VXCL(this:steer_thrvec, this:steer_refv):NORMALIZED.
		LOCAL topv IS VXCL(this:steer_thrvec, this:cur_dir:TOPVECTOR):NORMALIZED.
		
		
		set this:cur_steer_roll to signed_angle(refv, topv, this:steer_thrvec, 1).
	}).
	
	this:add("measure_cur_state", {
		this:update_time().
		
		set this:cur_dir to ship:facing.
		set this:cur_thrvec to thrust_vec().
		
		set this:cur_steer_az to get_az_lvlh(this:steer_dir).
		set this:cur_steer_pitch to get_pitch_lvlh(this:steer_dir).
		
		this:measure_refv_roll().
		
		local tgtv_h is vxcl(this:steer_dir:topvector, this:steer_tgtdir:forevector):normalized.
		local tgtv_v is vxcl(this:steer_dir:starvector, this:steer_tgtdir:forevector):normalized.
		local tgttv_p is vxcl(this:steer_dir:forevector, this:steer_tgtdir:topvector):normalized.
		
		
		set this:steer_pitch_delta to signed_angle(tgtv_v, this:steer_dir:forevector, this:steer_dir:starvector, 0).
		set this:steer_yaw_delta to -signed_angle(tgtv_h, this:steer_dir:forevector, this:steer_dir:topvector, 0).
		set this:steer_roll_delta to signed_angle(tgttv_p, this:steer_dir:topvector, this:steer_dir:forevector, 0).
		
		set this:throt_delta to this:thr_rpl_tgt - this:thr_cmd_rpl.
	}).
	
	
	
	this:add("max_steervec_corr", 5).
	this:add("steer_refv", SHIP:FACINg:topvector).
	this:add("steer_thrvec", SHIP:FACINg:forevector).
	this:add("steer_roll", 0).
	this:add("steer_cmd_roll", 0).
	
	this:add("steer_tgtdir", SHIP:FACINg).
	
	

	
	this:add("set_steer_tgt", {
		parameter new_thrvec.
		
		set this:steer_thrvec to new_thrvec.
		
		//required for continuous pilot input across several funcion calls
		LOCAL time_gain IS ABS(this:iteration_dt/0.2).
		
		local max_roll_corr is 13 * time_gain * STEERINGMANAGER:MAXSTOPPINGTIME.
		
		local roll_delta is unfixangle(this:cur_steer_roll - this:steer_roll).
		set roll_delta to sign(roll_delta) * min(abs(roll_delta) ,max_roll_corr).
		
		set this:steer_cmd_roll to this:cur_steer_roll - roll_delta.
		
		set this:steer_tgtdir to aimAndRoll(this:steer_thrvec, this:steer_refv, this:steer_cmd_roll).
	}).
	
	this:add("steer_auto_thrvec", {
		set this:steer_mode to "auto_thrvec".
		
		this:measure_cur_state().
	
		local steer_err_tol is 0.5.
	
		local max_roll_corr is 20.
		
		local cur_steervec is this:cur_dir:forevector.
		local tgt_steervec is this:steer_tgtdir:forevector.
		
		local steer_err is vang(cur_steervec, tgt_steervec).
		
		if (steer_err > steer_err_tol) {
			local steerv_norm is vcrs(cur_steervec, tgt_steervec).
			local steerv_corr is min(this:max_steervec_corr, steer_err).
			
			set tgt_steervec to rodrigues(cur_steervec, steerv_norm, steerv_corr).
		} else {
			set tgt_steervec to tgt_steervec.
		}
		
		local cur_topvec is vxcl(tgt_steervec, this:cur_dir:topvector).
		local tgt_topvec is vxcl(tgt_steervec, this:steer_tgtdir:topvector).
		
		//local roll_err is signed_angle(tgt_topvec, cur_topvec, tgt_steervec, 0).
		//local roll_corr is sign(roll_err) * min(abs(roll_err) ,max_roll_corr).
		//
		//print "roll_corr " + roll_corr + " " at (0,20).
		//
		//set tgt_topvec to rodrigues(cur_topvec, tgt_steervec, -roll_corr).
	
		set this:steer_dir to LOOKDIRUP(tgt_steervec, tgt_topvec ).
	}).


	this:add("steer_css", {
		set this:steer_mode to "css".
		
		this:set_steering_css().
		
		this:measure_cur_state().
	
		//required for continuous pilot input across several funcion calls
		LOCAL time_gain IS ABS(this:iteration_dt/0.2).
		
		//gains suitable for manoeivrable steerign in atmosphere
		LOCAL pitchgain IS 0.7.
		LOCAL rollgain IS 1.
		LOCAL yawgain IS 0.7.
		
		LOCAL steer_tol IS 0.1.
		
		local input_pitch is SHIP:CONTROL:PILOTPITCH.
		local input_roll is SHIP:CONTROL:PILOTROLL.
		local input_yaw is SHIP:CONTROL:PILOTYAW.
		
		if (abs(input_pitch) < steer_tol) {
			set input_pitch to 0. 
		}
		if (abs(input_roll) < steer_tol) {
			set input_roll to 0. 
		}
		if (abs(input_yaw) < steer_tol) {
			set input_yaw to 0. 
		}

		//measure input minus the trim settings
		LOCAL deltaroll IS time_gain * rollgain * input_roll.
		LOCAL deltapitch IS time_gain * pitchgain * input_pitch.
		LOCAL deltayaw IS time_gain * yawgain * input_yaw.
		
		print input_pitch + "  " at (0,20).
		print input_roll + "  " at (0,21).
		print input_yaw + "  " at (0,22).
		
		local steer_fore is this:steer_dir:forevector.
		local steer_top is this:steer_dir:topvector.
		local steer_star is this:steer_dir:starvector.
		
		local new_steerfore is rodrigues(steer_fore, steer_top, deltayaw).
		set new_steerfore to rodrigues(new_steerfore, steer_star, - deltapitch).
		
		local cur_fore is this:cur_dir:forevector.
		
		local cur_new_norm is vcrs(cur_fore, new_steerfore).
		
		LOCAL ang_dev IS MIN(this:max_steervec_corr, vang(cur_fore, new_steerfore) ).
		
		set new_steerfore to rodrigues(cur_fore, cur_new_norm, ang_dev).
		
		local new_steertop is vxcl(new_steerfore, steer_top).   
		
		set new_steertop to rodrigues(new_steertop, new_steerfore, - deltaroll).
		
		set this:steer_dir to LOOKDIRUP(new_steerfore, new_steertop ).
	}).
	
	
	this:add("thr_rpl_tgt", 0).
	this:add("thr_cmd_tgt", 0).
	this:add("thr_max", 1).	
	this:add("thr_min", 0).	
	
	this:add("update_thr_cmd", {
		local new_thr_rpl is CLAMP(this:thr_rpl_tgt, this:thr_min, this:thr_max).
		set this:thr_cmd_tgt to throtteValueConverter(new_thr_rpl, this:thr_min).
	}).
	
	this:add("thr_control_auto", {
		set this:thr_mode to "thr_auto".
	
		this:update_thr_cmd().
		
		set this:thr_cmd to this:thr_cmd_tgt.
		set this:thr_cmd_rpl to throtteValueConverter(this:thr_cmd, this:thr_min, TRUE).
	}).
	
	
	this:add("thr_control_css", {
		set this:thr_mode to "thr_css".
		
		this:update_thr_cmd().
	
		local thr_input is  SHIP:CONTROL:PILOTMAINTHROTTLE.
		set this:thr_cmd to thr_input.
		set this:thr_cmd_rpl to throtteValueConverter(this:thr_cmd, this:thr_min, TRUE).
	}).
	
	
	this:add("print_debug",{
		PARAMETER line.
		
		print "steer_mode : " + this:steer_mode + "    " at (0,line).
		print "thr_mode : " + this:thr_mode + "    " at (0,line + 1).
		
		print "loop dt : " + round(this:iteration_dt(),3) + "    " at (0,line + 3).
		
		print "cur_steer_pitch : " + round(this:cur_steer_pitch,3) + "    " at (0,line + 5).
		print "cur_steer_roll : " + round(this:cur_steer_roll,3) + "    " at (0,line + 6).
		print "steer_cmd_roll : " + round(this:steer_cmd_roll,3) + "    " at (0,line + 7).
		print "thr_rpl_tgt : " + round(this:thr_rpl_tgt,3) + "    " at (0,line + 8).
		print "thr_cmd_tgt : " + round(this:thr_cmd_tgt,3) + "    " at (0,line + 9).
		
		print "steer_pitch_delta : " + round(this:steer_pitch_delta,3) + "    " at (0,line + 11).
		print "steer_roll_delta : " + round(this:steer_roll_delta,3) + "    " at (0,line + 12).
		print "steer_yaw_delta : " + round(this:steer_yaw_delta,3) + "    " at (0,line + 13).
		print "throt_delta : " + round(this:throt_delta,3) + "    " at (0,line + 14).
		
	}).
	
	this:add("set_steering_auto", {
		SET STEERINGMANAGER:MAXSTOPPINGTIME TO 2.
	}).
	
	this:add("set_steering_css", {
		SET STEERINGMANAGER:MAXSTOPPINGTIME TO 4.
	}).
	
	this:add("set_rcs", {
		PARAMETER on_.
		
		if (on_) {
			RCS ON.
		} else {
			RCS OFF.
		}
	}).
	
	this:measure_cur_state().

	return this.
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


