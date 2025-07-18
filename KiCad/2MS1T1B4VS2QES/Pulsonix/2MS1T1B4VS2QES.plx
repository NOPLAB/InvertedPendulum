PULSONIX_LIBRARY_ASCII "SamacSys ECAD Model"
//12729985/1412633/2.50/5/4/Integrated Circuit

(asciiHeader
	(fileUnits MM)
)
(library Library_1
	(padStyleDef "c163.5_h109"
		(holeDiam 1.09)
		(padShape (layerNumRef 1) (padShapeType Ellipse)  (shapeWidth 1.635) (shapeHeight 1.635))
		(padShape (layerNumRef 16) (padShapeType Ellipse)  (shapeWidth 1.635) (shapeHeight 1.635))
	)
	(textStyleDef "Normal"
		(font
			(fontType Stroke)
			(fontFace "Helvetica")
			(fontHeight 1.27)
			(strokeWidth 0.127)
		)
	)
	(patternDef "2MS1T1B4VS2QES" (originalName "2MS1T1B4VS2QES")
		(multiLayer
			(pad (padNum 1) (padStyleRef c163.5_h109) (pt 0.000, 0.000) (rotation 90))
			(pad (padNum 2) (padStyleRef c163.5_h109) (pt 0.000, -2.540) (rotation 90))
			(pad (padNum 3) (padStyleRef c163.5_h109) (pt 0.000, -5.080) (rotation 90))
			(pad (padNum 4) (padStyleRef c163.5_h109) (pt 0.000, 2.540) (rotation 90))
			(pad (padNum 5) (padStyleRef c163.5_h109) (pt 0.000, -7.620) (rotation 90))
		)
		(layerContents (layerNumRef 18)
			(attr "RefDes" "RefDes" (pt 0.000, -2.540) (textStyleRef "Normal") (isVisible True))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.54 2.54) (pt 2.54 2.54) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 2.54 2.54) (pt 2.54 -7.62) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt 2.54 -7.62) (pt -2.54 -7.62) (width 0.025))
		)
		(layerContents (layerNumRef 28)
			(line (pt -2.54 -7.62) (pt -2.54 2.54) (width 0.025))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -3.54 4.358) (pt 3.54 4.358) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 3.54 4.358) (pt 3.54 -9.438) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt 3.54 -9.438) (pt -3.54 -9.438) (width 0.1))
		)
		(layerContents (layerNumRef Courtyard_Top)
			(line (pt -3.54 -9.438) (pt -3.54 4.358) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt 1 2.54) (pt 2.54 2.54) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt 2.54 2.54) (pt 2.54 -7.62) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt 2.54 -7.62) (pt 1 -7.62) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -1 2.54) (pt -2.54 2.54) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.54 2.54) (pt -2.54 -7.62) (width 0.1))
		)
		(layerContents (layerNumRef 18)
			(line (pt -2.54 -7.62) (pt -1 -7.62) (width 0.1))
		)
	)
	(symbolDef "2MS1T1B4VS2QES" (originalName "2MS1T1B4VS2QES")

		(pin (pinNum 1) (pt 0 mils -100 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -125 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 2) (pt 0 mils -200 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -225 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 3) (pt 0 mils -300 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -325 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 4) (pt 0 mils 0 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -25 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(pin (pinNum 5) (pt 0 mils -400 mils) (rotation 0) (pinLength 200 mils) (pinDisplay (dispPinName true)) (pinName (text (pt 230 mils -425 mils) (rotation 0]) (justify "Left") (textStyleRef "Normal"))
		))
		(line (pt 200 mils 100 mils) (pt 700 mils 100 mils) (width 6 mils))
		(line (pt 700 mils 100 mils) (pt 700 mils -500 mils) (width 6 mils))
		(line (pt 700 mils -500 mils) (pt 200 mils -500 mils) (width 6 mils))
		(line (pt 200 mils -500 mils) (pt 200 mils 100 mils) (width 6 mils))
		(attr "RefDes" "RefDes" (pt 750 mils 300 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))
		(attr "Type" "Type" (pt 750 mils 200 mils) (justify Left) (isVisible True) (textStyleRef "Normal"))

	)
	(compDef "2MS1T1B4VS2QES" (originalName "2MS1T1B4VS2QES") (compHeader (numPins 5) (numParts 1) (refDesPrefix IC)
		)
		(compPin "1" (pinName "NO") (partNum 1) (symPinNum 1) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "2" (pinName "COM") (partNum 1) (symPinNum 2) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "3" (pinName "NC") (partNum 1) (symPinNum 3) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "MH1" (pinName "MH1") (partNum 1) (symPinNum 4) (gateEq 0) (pinEq 0) (pinType Unknown))
		(compPin "MH2" (pinName "MH2") (partNum 1) (symPinNum 5) (gateEq 0) (pinEq 0) (pinType Unknown))
		(attachedSymbol (partNum 1) (altType Normal) (symbolName "2MS1T1B4VS2QES"))
		(attachedPattern (patternNum 1) (patternName "2MS1T1B4VS2QES")
			(numPads 5)
			(padPinMap
				(padNum 1) (compPinRef "1")
				(padNum 2) (compPinRef "2")
				(padNum 3) (compPinRef "3")
				(padNum 4) (compPinRef "MH1")
				(padNum 5) (compPinRef "MH2")
			)
		)
		(attr "Manufacturer_Name" "Dailywell")
		(attr "Manufacturer_Part_Number" "2MS1T1B4VS2QES")
		(attr "Mouser Part Number" "N/A")
		(attr "Mouser Price/Stock" "https://www.mouser.co.uk/ProductDetail/Dailywell/2MS1T1B4VS2QES?qs=Zz7%252BYVVL6bHHqAn5ycuDAQ%3D%3D")
		(attr "Arrow Part Number" "")
		(attr "Arrow Price/Stock" "")
		(attr "Description" "Toggle Switches")
		(attr "<Hyperlink>" "https://akizukidenshi.com/download/ds/cosland/2MS1-T1-B4-VS2-Q-E-S.pdf")
		(attr "<Component Height>" "23.61")
		(attr "<STEP Filename>" "2MS1T1B4VS2QES.stp")
		(attr "<STEP Offsets>" "X=0;Y=-2.54;Z=0.42")
		(attr "<STEP Rotation>" "X=90;Y=0;Z=0")
	)

)
