
GLOBAL vehicle IS LEXICON(
					"name","Apollo Direct", 
					"roll",0,
					"stages",LIST(0,		//never remove this zero		
					
								LEXICON(
										"m_initial",	62.72,
										"m_final",	32.98,
										"staging", LEXICON (
											"type","depletion",
											"ignition",	TRUE,
											"ullage", "none",
											"ullage_t",	0	
										),
										"engines",	LIST(
														LEXICON("thrust", 140.8, "isp", 306.4, "minThrottle", 0.1)	//3xLMDE-J
										)					
								)
							)
).






