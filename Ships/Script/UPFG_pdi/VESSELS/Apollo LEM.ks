
GLOBAL vehicle IS LEXICON(
					"name","Apollo LEM", 
					"roll",0,
					"stages",LIST(0,		//never remove this zero		
					
								LEXICON(
										"m_initial",	15.142,
										"m_final",	6.380,
										"staging", LEXICON (
											"type","depletion",
											"ignition",	TRUE,
											"ullage", "none",
											"ullage_t",	0	
										),
										"minThrottle",0.101,
										"engines",	LIST(
														LEXICON("thrust", 46.7, "isp", 305, "resources",LIST("Aerozine50","MON1"))	//1xLMDE-H
										)					
								)
							)
).






