// 			UPFG implementation lifted form the Shuttle OPS1 script - less the RTLS stuff

//global UPFG variables 

// dummy lexicon for gui printout before mode 103
GLOBAL upfgInternal IS LEXICON(
							"tgo", 1,
							"vgo", v(0,0,0),
							"s_meco", FALSE,
							"s_init", FALSE,
							"s_conv", FALSE,
							"iter_conv", 0,
							"throtset", 1
).
	
	


//	empty lexicon
FUNCTION setupUPFG {
	LOCAL init_steervec IS vecYZ(thrust_vec()).
	LOCAL stg IS get_stage().
	local init_throt is stg["Throttle"].
	LOCAL min_throt IS 0.
	
	if (stg["engines"]:HASKEY("minThrottle")) {
		set min_throt to stg["engines"]["minThrottle"].
	}

	SET upfgInternal TO  LEXICON(
		"r_cur", V(0, 0, 0),
		"v_cur", V(0, 0, 0),
		"ve_cur", V(0, 0, 0),
		"t_cur", 0,
		"m_cur", 0,
		"tb_cur", 0,
		"s_meco", FALSE,
		"s_init", FALSE,
		"s_conv", FALSE,
		"iter_conv", 0,
		"iter_unconv", 0,
		"itercount", 0,
		"itercount_reset", 40,
		"terminal_time", 10,
		"tgo_conv", 1,
		"steer_conv", 20,
		"constraint_release_t", 40,
		"rbias", V(0, 0, 0),
		"rd", V(0, 0, 0),
		"rdmag", 0,
		"vd", V(0, 0, 0),
		"v", V(0, 0, 0),
		"ix", V(0, 0, 0),
		"iy", V(0, 0, 0),
		"iy_tvr", V(0, 0, 0),
		"iz", V(0, 0, 0),
		"rgrav", V(0, 0, 0),
		"time", 0,
		"dt", 0,
		"tgo", 1,
		"vgo", v(0,0,0),
		"vgodmag", 0,
		"lambda", V(1,0,0),
		"lambdadot", V(0,0,0),
		"lambdy", 0,
		"omega", V(0,0,0),
		"steering", init_steervec,
		"s_alt", FALSE,
		"s_plane", FALSE,
		"s_throt", FALSE,
		"throtset", init_throt
	).
}	


FUNCTION upfg_normal {
	PARAMETER tgtIncl.
	PARAMETER tgtLAN.

	RETURN vecYZ(targetNormal(tgtIncl, tgtLAN)). 
}

FUNCTION resetUPFG {
	addMessage("RESETTING UPFG").
	setupUPFG().
}


FUNCTION upfg_standard_initialise {
	PARAMETER tgt_orb.
	PARAMETER internal.
	
	LOCAL curT IS surfacestate["time"].
	LOCAL curR IS orbitstate["radius"].
	LOCAL curV IS orbitstate["velocity"].
	
	local cutvec is VXCL(tgt_orb["normal"], curR):NORMALIZED.
	
	//arbitrarily set cutoff point at 10 degrees ahead of the current position.
	set cutvec to rodrigues(cutvec, tgt_orb["normal"], 10).
	
	LOCAL rdmag IS tgt_orb["cutoff alt"]*1000 + SHIP:BODY:RADIUS.
	
	set tgt_orb["radius"] TO cutvec:NORMALIZED * rdmag.	
	
	local rgrav IS -SHIP:ORBIT:BODY:MU * curR / curR:MAG^3.
	
	SET internal["time"] tO curT.
	SET internal["rd"] tO tgt_orb["radius"].
	SET internal["rdmag"] tO rdmag.
	SET internal["v"] tO curV.
	SET internal["rgrav"] tO 0.5 * rgrav.
	
	
	SET internal["s_plane"] TO TRUE.
	SET internal["s_alt"] TO TRUE.
	
	SET internal["ix"] tO tgt_orb["radius"]:NORMALIZED.
	SET internal["iy"] tO -tgt_orb["normal"].
	
	SET internal["vd"] tO cutoff_velocity_vector(
		internal["ix"],
		internal["iy"],
		tgt_orb["velocity"],
		tgt_orb["fpa"]
	).
	
	SET internal["vgo"] tO internal["vd"] - internal["v"].
}

FUNCTION upfg_sense_current_state {
	PARAMETER internal.
	
	LOCAL stg IS get_stage().
	
	SET internal["t_cur"] TO surfacestate["time"].
	SET internal["r_cur"] TO orbitstate["radius"].
	SET internal["v_cur"] TO orbitstate["velocity"].
	SET internal["ve_cur"] TO vecYZ(surfacestate["surfv"]).
	SET internal["m_cur"] TO stg["m_initial"].
	SET internal["tb_cur"] TO stg["Tstage"].

	IF (target_orbit["mode"] = 5) {
		SET internal["mbod"] tO vehicle["rtls_mbod"].
	}
}



//		UPFG MAIN ROUTINE

FUNCTION upfg_landing {
	PARAMETER vehicle.
	PARAMETER tgt_orb.
	PARAMETER internal.
	
	LOCAL s_mode Is tgt_orb["mode"].
	
	LOCAL g0 IS 9.80665. 
	
	//	measure vehicle parameters
	LOCAL n IS vehicle:LENGTH.
	LOCAL SM IS LIST().
	LOCAL aL IS LIST().
	LOCAL mT IS LIST().
	LOCAL md IS LIST().
	LOCAL ve IS LIST().
	LOCAL fT IS LIST().
	LOCAL aT IS LIST().
	LOCAL tu IS LIST().
	LOCAL tb IS LIST().
	LOCAL kklist IS LIST().
	
	local kk_cmd is internal["throtset"].
	local kk_gl is kk_cmd.
  
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		SM:ADD(vehicle[i]["mode"]).
		IF vehicle[i]:HASKEY("glim") {
			aL:ADD(vehicle[i]["glim"]*g0).
		} ELSE {
			aL:ADD(0).
		}
		fT:ADD(vehicle[i]["engines"]["thrust"]).
		md:ADD(vehicle[i]["engines"]["flow"]).
		ve:ADD(vehicle[i]["engines"]["isp"]*g0).
		tb:ADD(vehicle[i]["Tstage"]).
		kklist:ADD(vehicle[i]["Throttle"]).
		mT:ADD(vehicle[i]["m_initial"]).

		IF (i=0) {
			SET kklist[i] TO internal["throtset"].	
			SET mT[i] TO internal["m_cur"].	
			SET tb[i] TO internal["tb_cur"].	
			
			if (SM[i]=2) {
				set kk_gl to vehicle[i]["glim"] * g0 * mT[i] / fT[i].
			}
		}
		SET fT[i] TO fT[i]*kklist[i].
		SET md[i] TO md[i]*kklist[i].
		aT:ADD(fT[i]/mT[i]).
		tu:ADD(ve[i]/aT[i]).
	}

	
	//initialise or update
	LOCAL dt IS 0.
	IF (internal["s_init"]) {
		SET dt TO internal["t_cur"] - internal["time"].	
		SET internal["time"] TO internal["t_cur"].
		SET internal["dt"] TO dt.
		
		//vgo update subtask
		SET internal["vgo"] TO internal["vgo"] - (internal["v_cur"] - internal["v"]).
		SET internal["v"] TO internal["v_cur"].
	} ELSE {
			
		upfg_standard_initialise(tgt_orb, internal).
		
		SET internal["s_init"] tO TRUE.
		
		//modification - reset convergence 
		SET internal["iter_conv"] TO 0.
		SET internal["s_conv"] tO FALSE.
	}
	
	
	//	tgo subtask
	IF SM[0]=1 {
		SET aT[0] TO fT[0] / internal["m_cur"].
	} ELSE IF SM[0]=2 {
		SET aT[0] TO aL[0].
	}
	SET tu[0] TO ve[0] / aT[0].
	
	LOCAL Li IS LIST().
	LOCAL Lsum IS 0.
	FROM { LOCAL i IS 0. } UNTIL i>=(n-1) STEP { SET i TO i+1. } DO {
		
		LOCAL Li_ IS 0.
		
		IF SM[i]=1 {
			SET Li_ TO ve[i] * LN(tu[i]/(tu[i]-tb[i])).
		} ELSE IF SM[i]=2 {
			SET Li_ TO  aL[i]*tb[i].
		} ELSE  {
			SET Li_ TO 0.
		}
		
		IF ((Lsum + Li_) > internal["vgo"]:MAG) {
			SET n to (i + 1).
			BREAK.
		} ELSE {
			Li:ADD(Li_).
			SET Lsum TO Lsum + Li_.
		}
	}
	
	Li:ADD(internal["vgo"]:MAG - Lsum).
	
	
	LOCAL tgoi IS LIST().
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		IF SM[i]=1 {
			SET tb[i] TO tu[i] * (1-CONSTANT:E^(-Li[i]/ve[i])).
		} ELSE IF SM[i]=2 {
			SET tb[i] TO Li[i] / aL[i].
		}
		IF i=0 {
			tgoi:ADD(tb[i]).
		} ELSE {
			tgoi:ADD(tgoi[i-1] + tb[i]).
		}
	}
	
	LOCAL tgop IS internal["tgo"].
	SET internal["tgo"] TO tgoi[n-1].
	
	//	thrust integrals subtask
	LOCAL L_ IS 0.
	LOCAL J_ IS 0.
	LOCAL S_ IS 0.
	LOCAL Q_ IS 0.
	LOCAL H_ IS 0.
	LOCAL P_ IS 0.
	LOCAL Ji IS LIST().
	LOCAL Si IS LIST().
	LOCAL Qi IS LIST().
	LOCAL Pi IS LIST().
	LOCAL tgoi1 IS 0.
	
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		IF i>0 {
			SET tgoi1 TO tgoi[i-1].
		}
		IF SM[i]=1 {
			Ji:ADD( tu[i]*Li[i] - ve[i]*tb[i] ).
			Si:ADD( -Ji[i] + tb[i]*Li[i] ).
			Qi:ADD( Si[i]*(tu[i]+tgoi1) - 0.5*ve[i]*tb[i]^2 ).
			Pi:ADD( Qi[i]*(tu[i]+tgoi1) - 0.5*ve[i]*tb[i]^2 * (tb[i]/3+tgoi1) ).
		} ELSE IF SM[i]=2 {
			Ji:ADD( 0.5*Li[i]*tb[i] ).
			Si:ADD( Ji[i] ).
			Qi:ADD( Si[i]*(tb[i]/3+tgoi1) ).
			Pi:ADD( (1/6)*Si[i]*(tgoi[i]^2 + 2*tgoi[i]*tgoi1 + 3*tgoi1^2) ).
		}
		
		SET Ji[i] TO Ji[i] + Li[i]*tgoi1.
		SET Si[i] TO Si[i] + L_*tb[i].
		SET Qi[i] TO Qi[i] + J_*tb[i].
		SET Pi[i] TO Pi[i] + H_*tb[i].
		
		SET L_ TO L_+Li[i].
		SET J_ TO J_+Ji[i].
		SET S_ TO S_+Si[i].
		SET Q_ TO Q_+Qi[i].
		SET P_ TO P_+Pi[i].
		SET H_ TO J_*tgoi[i] - Q_.
	}
	LOCAL JOL IS J_/L_.
	LOCAL Qprime IS Q_ - S_*JOL.
	
	
	//	reference thrust vector subtask
	IF internal["vgo"]:MAG <>0 { SET internal["lambda"] TO internal["vgo"]:NORMALIZED.}
	
	// range-to-go subtask
	
	IF (tgop>0) {
		SET internal["rgrav"] TO (internal["tgo"]/tgop)^2 * internal["rgrav"].
	}
	LOCAL rgo IS internal["rd"] - (internal["r_cur"] + internal["v_cur"]*internal["tgo"] + internal["rgrav"]) + internal["rbias"].
	
	LOCAL rgoprime IS Qprime * VCRS(internal["omega"], internal["lambda"]) + S_ * internal["lambda"].
	
	LOCAL rgox IS 0.
	IF (internal["s_alt"]) {
		SET rgox TO VDOT(internal["ix"], rgo).
	} ELSE {
		SET rgox tO VDOT(internal["ix"], rgoprime).
	}
	
	LOCAL rgoy IS 0.
	IF (internal["s_plane"]) {
		SET rgoy TO VDOT(internal["iy"],rgo).
	} ELSE {
		SET rgoy tO VDOT(internal["iy"],rgoprime).
	}
	
	LOCAL rgoxy IS rgox * internal["ix"] + rgoy * internal["iy"].
	
	SET internal["iz"] TO VCRS(internal["ix"], internal["iy"]).
	LOCAL rgoz IS (S_ - VDOT(internal["lambda"], rgoxy)) / VDOT(internal["lambda"], internal["iz"]).
	SET rgo TO rgoxy + rgoz * internal["iz"].
	
	//turning rate vector subtask
	
	SET internal["lambdadot"] TO (rgo - S_ * internal["lambda"]) / Qprime.
	
	//steering inputs update subtask
	LOCAL steering_prev IS internal["steering"].
	SET internal["steering"] TO  internal["lambda"] - internal["lambdadot"]*JOL.
	
	//burnout state vector prediction + thrust time integrals
	LOCAL phi IS VANG(internal["steering"], internal["lambda"]) * CONSTANT:DEGTORAD.
	LOCAL phidot IS - phi / JOL.
	LOCAL vthrust IS (L_ - 0.5 * L_ * phi^2 - J_ * phi * phidot - 0.5 * H_ * phidot^2).
	SET vthrust TO vthrust * internal["lambda"] - (L_ * phi + J_ * phidot) * internal["lambdadot"]:NORMALIZED.
	LOCAL rthrust IS S_ - 0.5 * S_ * phi^2 - Q_ * phi * phidot - 0.5 * P_ * phidot^2.
	SET rthrust TO rthrust * internal["lambda"] - (S_ * phi + Q_ * phidot) * internal["lambdadot"]:NORMALIZED.
	SET internal["rbias"] TO rgo - rthrust.
	
	
	LOCAL rc1 IS internal["r_cur"] - 0.1 * rthrust - (internal["tgo"] / 30) * vthrust.
	LOCAL vc1 IS internal["v_cur"] + 1.2 * rthrust / internal["tgo"] - 0.1 * vthrust.
	
	LOCAL pack IS cse_rk3(rc1, vc1, internal["tgo"]).
	SET internal["rgrav"] TO pack[0] - rc1 - vc1 * internal["tgo"].
	LOCAL vgrav IS pack[1] - vc1.

	LOCAL rp IS internal["r_cur"] + internal["v_cur"]*internal["tgo"] + internal["rgrav"] + rthrust.
	LOCAL vp IS internal["v_cur"] + vgrav + vthrust.
	
	//desired orbit plane correction subtask
	IF (internal["s_plane"]) {
		SET internal["rd"] TO VXCL(internal["iy"],rp).	
	} ELSE {
		//always land in this for rtls
		set internal["rd"] to rp.
	}
	
	//desired position subtask
	SET internal["ix"] TO internal["rd"]:NORMALIZED.
	
	IF (internal["s_alt"]) {
		SET internal["rd"] TO tgt_orb["radius"]:MAG * internal["ix"].	
	}
	
	//corrector portion
	LOCAL dvgo IS 0.

	//desired velocity
	SET tgt_orb TO cutoff_params(tgt_orb, internal["rd"]).
	
	IF (internal["s_plane"]) {
		SET internal["iy"] TO -tgt_orb["normal"].
	} ELSE {
		SET internal["iy"] TO VCRS(internal["vd"], internal["rd"]):NORMALIZED.
		SET tgt_orb["normal"] TO -internal["iy"].
	}
	
	SET internal["iz"] TO VCRS(internal["ix"], internal["iy"]):NORMALIZED.
	
	SET internal["vd"] TO cutoff_velocity_vector(
		internal["ix"],
		internal["iy"],
		tgt_orb["velocity"],
		tgt_orb["fpa"]
	).

	SET dvgo TO (internal["vd"] - vp).
	
	//vgo correction subtask
	SET internal["vgo"] TO internal["vgo"] + dvgo.
	
	//my addition: throttle controller for constant g in any case 
	if (SM[0]=2) {
		SET kk_cmd TO CLAMP(kk_gl, 0, 1).
	}
	
	//convergence check subtask 
	LOCAL wasconv IS internal["s_conv"].
	LOCAL tgo_expected IS tgop - dt.
	
	IF (NOT internal["s_conv"]) {
		SET internal["itercount"] TO internal["itercount"]+1.
	}
		
	IF (ABS(tgo_expected - internal["tgo"]) < internal["tgo_conv"]) { //first criterion for convergence
		IF (VANG(steering_prev, internal["steering"]) < internal["steer_conv"]) { //second criterion for convergence
			SET internal["iter_unconv"] TO 0.
			IF (internal["iter_conv"] < 3) {
				SET internal["iter_conv"] TO internal["iter_conv"] + 1.
			} ELSE {
				if (NOT internal["s_conv"]) {
					//moved here from main executive
					addMessage("GUIDANCE CONVERGED IN " + internal["itercount"] + " ITERATIONS").
					SET internal["s_conv"] tO TRUE.
					SET internal["iter_conv"] TO 0.
				}
			}
		} ELSE { //something is wrong, reset
			resetUPFG().
			RETURN.
		}
	} ELSE {
		//if we were converged and twice in a row we break the convergence criterion, go unconverged
		IF (wasconv AND internal["iter_unconv"] < 2) {
			SET internal["iter_unconv"] TO internal["iter_unconv"] + 1.
		} ELSE {
			SET internal["iter_conv"] TO 0.
			SET internal["s_conv"] tO FALSE.
		}
	}	

	IF wasconv AND (NOT internal["s_conv"]) {
		//in this case we had convergence and we lost it, reset itercount
		SET internal["itercount"] TO 0.
	}
	
	//constraint release
	If (internal["s_conv"] AND internal["tgo"] < internal["constraint_release_t"]) OR 
		((NOT internal["s_init"]) AND (internal["tgo"] < (internal["constraint_release_t"] - 10))) {
		//release cutoff position
		IF (internal["s_alt"] OR internal["s_plane"]) {
			SET internal["omega"] TO VCRS(internal["lambda"], internal["lambdadot"]).
			SET internal["s_plane"] TO FALSE.
			SET internal["s_alt"] TO FALSE.
		}
	}
	
	//protection
	if (internal["itercount"] > internal["itercount_reset"]) {
		resetUPFG().
		RETURN.
	}
	
	//throttle command 
	SET internal["throtset"] TO CLAMP(kk_cmd, 0, 1).
	
	//terminal count 		
	local tgo_terminal_flag IS (internal["tgo"] <= internal["terminal_time"]).
	
	local guided_meco_flag is internal["s_conv"] AND tgo_terminal_flag.
	
	local delta_vd is (internal["vd"] - internal["v_cur"]):mag.
	
	local unguided_meco_flag is false.
	
	local tgt_v_close_flag is (0.015 * internal["vd"]:mag >= delta_vd).
	set unguided_meco_flag to (NOT internal["s_conv"]) AND tgt_v_close_flag.

	set internal["s_meco"] TO (guided_meco_flag OR unguided_meco_flag).
	
}




//	for reference and posterity
FUNCTION upfg_landing_old {

	DECLARE FUNCTION compute_iF {
		PARAMETER time_.
		LOCAL out IS  lambda + lambdadot*time_.
		RETURN out:NORMALIZED.
	}

	PARAMETER t.
	PARAMETER vehicle.
	PARAMETER ldg_state.
	PARAMETER previous.
	

	LOCAL dt IS t - previous["time"].
	LOCAL v_cur IS orbitstate["velocity"].
	LOCAL vgo IS previous["vgo"] - (v_cur - previous["v"]).
	LOCAL tgo IS previous["tgo"].
	LOCAL lambda IS previous["lambda"].
	LOCAL lambdadot IS previous["lambdadot"].
		
	LOCAL r_cur IS orbitstate["radius"].
	LOCAL cser IS previous["cser"].
	LOCAL rd IS previous["rd"].
	LOCAL rbias IS previous["rbias"].
	LOCAL rgrav IS previous["rgrav"].
	LOCAL iy IS -ldg_state["normal"]:NORMALIZED.
	LOCAL iz IS VCRS(rd,iy):NORMALIZED.
	LOCAL m IS vehicle[0]["m_initial"].
	LOCAL Kk IS previous["throtset"].
	LOCAL drz IS 0.
	
	LOCAL g0 IS 9.80665. 
	
	//	1
	LOCAL n IS vehicle:LENGTH.
	LOCAL SM IS LIST().
	LOCAL aL IS LIST().
	LOCAL md IS LIST().
	LOCAL ve IS LIST().
	LOCAL fT IS LIST().
	LOCAL aT IS LIST().
	LOCAL tu IS LIST().
	LOCAL tb IS LIST().
	LOCAL kklist IS LIST().
  
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		SM:ADD(vehicle[i]["mode"]).
		IF vehicle[i]:HASKEY("glim") {
			aL:ADD(vehicle[i]["glim"]*g0).
		} ELSE {
			aL:ADD(0).
		}
		fT:ADD(vehicle[i]["engines"]["thrust"]).
		md:ADD(vehicle[i]["engines"]["flow"]).
		kklist:ADD(vehicle[i]["Throttle"]).
		IF (i=0) {
			SET kklist[i] TO Kk.	
		}
		SET fT[i] TO fT[i]*kklist[i].
		SET md[i] TO md[i]*kklist[i].
		ve:ADD(vehicle[i]["engines"]["isp"]*g0).
		aT:ADD(fT[i] / vehicle[i]["m_initial"]).
		tu:ADD(ve[i]/aT[i]).
		tb:ADD(vehicle[i]["Tstage"]).
	}
	
	
	//	3
	IF SM[0]=1 {
		SET aT[0] TO fT[0] / m.
	} ELSE IF SM[0]=2 {
		SET aT[0] TO aL[0].
	}
	SET tu[0] TO ve[0] / aT[0].
	
	LOCAL Li IS LIST().
	LOCAL Lsum IS 0.
	FROM { LOCAL i IS 0. } UNTIL i>=n-1 STEP { SET i TO i+1. } DO {
		IF SM[i]=1 {
			Li:ADD( ve[i]*LN(tu[i]/(tu[i]-tb[i])) ).
		} ELSE IF SM[i]=2 {
			Li:ADD( aL[i]*tb[i] ).
		} ELSE Li:ADD( 0 ).
		SET Lsum TO Lsum + Li[i].
		
		IF Lsum>vgo:MAG {
			RETURN upfg_landing(
				t,
				vehicle:SUBLIST(0,vehicle:LENGTH-1),
				ldg_state,
				previous
			).
		}
	}
	Li:ADD(vgo:MAG - Lsum).
	
	
	LOCAL tgoi IS LIST().
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		IF SM[i]=1 {
			SET tb[i] TO tu[i] * (1-CONSTANT:E^(-Li[i]/ve[i])).
		} ELSE IF SM[i]=2 {
			SET tb[i] TO Li[i] / aL[i].
		}
		IF i=0 {
			tgoi:ADD(tb[i]).
		} ELSE {
			tgoi:ADD(tgoi[i-1] + tb[i]).
		}
	}
	
	SET tgo TO tgoi[n-1].
	
	//	4
	LOCAL L_ IS 0.
	LOCAL J_ IS 0.
	LOCAL S_ IS 0.
	LOCAL Q_ IS 0.
	LOCAL H_ IS 0.
	LOCAL P_ IS 0.
	LOCAL Ji IS LIST().
	LOCAL Si IS LIST().
	LOCAL Qi IS LIST().
	LOCAL Pi IS LIST().
	LOCAL tgoi1 IS 0.
	
	FROM { LOCAL i IS 0. } UNTIL i>=n STEP { SET i TO i+1. } DO {
		IF i>0 {
			SET tgoi1 TO tgoi[i-1].
		}
		IF SM[i]=1 {
			Ji:ADD( tu[i]*Li[i] - ve[i]*tb[i] ).
			Si:ADD( -Ji[i] + tb[i]*Li[i] ).
			Qi:ADD( Si[i]*(tu[i]+tgoi1) - 0.5*ve[i]*tb[i]^2 ).
			Pi:ADD( Qi[i]*(tu[i]+tgoi1) - 0.5*ve[i]*tb[i]^2 * (tb[i]/3+tgoi1) ).
		} ELSE IF SM[i]=2 {
			Ji:ADD( 0.5*Li[i]*tb[i] ).
			Si:ADD( Ji[i] ).
			Qi:ADD( Si[i]*(tb[i]/3+tgoi1) ).
			Pi:ADD( (1/6)*Si[i]*(tgoi[i]^2 + 2*tgoi[i]*tgoi1 + 3*tgoi1^2) ).
		}
		
		SET Ji[i] TO Ji[i] + Li[i]*tgoi1.
		SET Si[i] TO Si[i] + L_*tb[i].
		SET Qi[i] TO Qi[i] + J_*tb[i].
		SET Pi[i] TO Pi[i] + H_*tb[i].
		
		SET L_ TO L_+Li[i].
		SET J_ TO J_+Ji[i].
		SET S_ TO S_+Si[i].
		SET Q_ TO Q_+Qi[i].
		SET P_ TO P_+Pi[i].
		SET H_ TO J_*tgoi[i] - Q_.
	}
	LOCAL K_ IS J_/L_.
	
	
	//	5
	IF vgo:MAG <>0 { SET lambda TO vgo:NORMALIZED.}
	IF previous["tgo"]>0 {
		SET rgrav TO (tgo/previous["tgo"])^2 * rgrav.
	}
	
	LOCAL rgo IS rd - (r_cur + v_cur*tgo + rgrav).
	LOCAL iz IS VCRS(rd,iy):NORMALIZED.
	LOCAL rgoxy IS rgo - VDOT(iz,rgo)*iz.
	LOCAL rgoz IS (S_ - VDOT(lambda,rgoxy)) / VDOT(lambda,iz).
	SET rgo TO rgoxy + rgoz*iz + rbias.
	LOCAL lambdade IS Q_ - S_*K_.
	SET lambdadot TO (rgo - S_*lambda) / lambdade.
	LOCAL iF_ IS compute_iF(-K_).
	LOCAL phi IS VANG(iF_,lambda)*CONSTANT:DEGTORAD.
	LOCAL phidot IS -phi/K_.
	LOCAL vthrust IS (L_ - 0.5*L_*phi^2 - J_*phi*phidot - 0.5*H_*phidot^2).
	SET vthrust TO vthrust*lambda - (L_*phi + J_*phidot)*lambdadot:NORMALIZED.
	LOCAL rthrust IS S_ - 0.5*S_*phi^2 - Q_*phi*phidot - 0.5*P_*phidot^2.
	SET rthrust TO rthrust*lambda - (S_*phi + Q_*phidot)*lambdadot:NORMALIZED.
	SET vbias TO vgo - vthrust.
	SET rbias TO rgo - rthrust.
	
	
	//	7
	
	
	LOCAL rc1 IS r_cur - 0.1*rthrust - (tgo/30)*vthrust.
	LOCAL vc1 IS v_cur + 1.2*rthrust/tgo - 0.1*vthrust.
	LOCAL pack IS cse(rc1, vc1, tgo, cser).
	SET cser TO pack[2].
	SET rgrav TO pack[0] - rc1 - vc1*tgo.
	LOCAL vgrav IS pack[1] - vc1.
	
	
	//	8
	LOCAL rp IS r_cur + v_cur*tgo + rgrav + rthrust.
	
	SET rp TO VXCL(iy,rp).
	
	LOCAL vd IS v(0,0,0).
	
	LOCAL ix IS rp:NORMALIZED.
	SET iz TO VCRS(ix,iy):NORMALIZED.
	
	IF (ldg_state["mode"]=1) {
		SET rd TO ldg_state["radius"]:MAG*ix.	
		SET vd TO ldg_state["velvec"].
	}
	ELSE IF (ldg_state["mode"]=0) {	
	
		SET ldg_state TO update_landing_state(ldg_state, r_cur, tgo).
		SET rd TO ldg_state["radius"].
		SET vd TO ldg_state["velvec"].
	
		//range throttling
		SET drz TO VDOT(iz,(rd - rp)).
		
		LOCAL vgoz IS VDOT(iz,vgo).
		LOCAL dtgo IS -2*drz/vgoz.
		
		LOCAL K_gain IS tb[0]/(tb[0] + dtgo).
		
		//SET K_gain TO 1.1*K_gain.
		SET Kk TO Kk*K_gain.
		
		//clamp throttle
		SET Kk TO MAX(0.001,MIN(1,Kk)).
	
		
	}

	SET vgo TO vd - v_cur - vgrav + vbias.
	
	//	RETURN - build new internal state instead of overwriting the old one
	LOCAL current IS LEXICON(
		"cser", cser,
		"rbias", rbias,
		"rd", rd,
		"rp", rp,
		"rgrav", rgrav,
		"time", t,
		"tgo", tgo,
		"v", v_cur,
		"vgo", vgo,
		"lambda", lambda,
		"lambdadot", lambdadot,
		"t_lambda",(t + K_),
		"steering",iF_,
		"throtset", Kk,
		"z_error", drz
	).
	
	
	
	RETURN LIST(current, ldg_state).
}
