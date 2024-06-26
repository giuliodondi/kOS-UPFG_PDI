IF (DEFINED pdi_siteslex) {UNSET pdi_siteslex.}
GLOBAL pdi_siteslex IS LEXICON(
		"Apollo 11", LEXICON(
				"name","Apollo 11",
				"position",BODY("Moon"):GEOPOSITIONLATLNG(0.6814,23.4597)
		),
		"Apollo 12", LEXICON(
				"name","Apollo 12",
				"position",BODY("Moon"):GEOPOSITIONLATLNG(-3.025,-23.4623)
		),
		"Apollo 14", LEXICON(
				"name","Apollo 14",
				"position",BODY("Moon"):GEOPOSITIONLATLNG(-3.6515,-17.4948)
		),
		"Apollo 15", LEXICON(
				"name","Apollo 15",
				"position",BODY("Moon"):GEOPOSITIONLATLNG(26.1293,3.6335)
		),
		"Apollo 16", LEXICON(
				"name","Apollo 16",
				"position",BODY("Moon"):GEOPOSITIONLATLNG(-8.99,15.4724)
		),
		"Apollo 17", LEXICON(
				"name","Apollo 17",
				"position",BODY("Moon"):GEOPOSITIONLATLNG(20.1926,30.7693)
		)
).
