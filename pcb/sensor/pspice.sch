EESchema Schematic File Version 4
LIBS:pspice-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Perimter Wire Sensor"
Date ""
Rev ""
Comp "Meadow Robotics GmbH"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L pspice:R R3
U 1 1 5CDF1F94
P 2150 2400
F 0 "R3" H 2220 2446 50  0000 L CNN
F 1 "82k" H 2220 2355 50  0000 L CNN
F 2 "" V 2080 2400 50  0001 C CNN
F 3 "~" H 2150 2400 50  0001 C CNN
	1    2150 2400
	0    -1   -1   0   
$EndComp
Text Notes 1950 4150 0    50   ~ 0
2nd order RLC low pas\nRL Low Pass.\n85mH, 1M => 1.87MHz\n100mH, 100kR => 160kHz
$Comp
L pspice:0 #GND01
U 1 1 5CDF3153
P 1500 1600
F 0 "#GND01" H 1500 1500 50  0001 C CNN
F 1 "0" H 1500 1689 50  0000 C CNN
F 2 "" H 1500 1600 50  0001 C CNN
F 3 "~" H 1500 1600 50  0001 C CNN
	1    1500 1600
	1    0    0    -1  
$EndComp
$Comp
L pspice:VSOURCE V2
U 1 1 5CDF440D
P 1500 1250
F 0 "V2" H 1728 1296 50  0000 L CNN
F 1 "VSOURCE" H 1728 1205 50  0000 L CNN
F 2 "" H 1500 1250 50  0001 C CNN
F 3 "~" H 1500 1250 50  0001 C CNN
F 4 "V" H 1500 1250 50  0001 C CNN "Spice_Primitive"
F 5 "dc 5" H 1500 1250 50  0001 C CNN "Spice_Model"
F 6 "Y" H 1500 1250 50  0001 C CNN "Spice_Netlist_Enabled"
	1    1500 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 1600 1500 1550
Text Label 1750 950  0    50   ~ 0
5V
Text Label 1750 1550 0    50   ~ 0
GND
$Comp
L pspice:INDUCTOR L1
U 1 1 5CE3EBE5
P 1100 2450
F 0 "L1" V 1146 2406 50  0000 R CNN
F 1 "85m" V 1055 2406 50  0000 R CNN
F 2 "" H 1100 2450 50  0001 C CNN
F 3 "~" H 1100 2450 50  0001 C CNN
	1    1100 2450
	0    -1   -1   0   
$EndComp
Text GLabel 3300 2400 2    50   Input ~ 0
out
Text GLabel 1100 2200 0    50   Input ~ 0
in
$Comp
L pspice:R R1
U 1 1 5CE44491
P 2150 1000
F 0 "R1" H 2220 1046 50  0000 L CNN
F 1 "10k" H 2220 955 50  0000 L CNN
F 2 "" V 2080 1000 50  0001 C CNN
F 3 "~" H 2150 1000 50  0001 C CNN
	1    2150 1000
	1    0    0    -1  
$EndComp
$Comp
L pspice:R R2
U 1 1 5CE4467D
P 2150 1500
F 0 "R2" H 2220 1546 50  0000 L CNN
F 1 "10k" H 2220 1455 50  0000 L CNN
F 2 "" V 2080 1500 50  0001 C CNN
F 3 "~" H 2150 1500 50  0001 C CNN
	1    2150 1500
	1    0    0    -1  
$EndComp
Connection ~ 2150 1250
Wire Wire Line
	1500 950  2000 950 
Wire Wire Line
	2000 950  2000 750 
Wire Wire Line
	2000 750  2150 750 
Wire Wire Line
	2150 1750 2000 1750
Wire Wire Line
	2000 1750 2000 1550
Wire Wire Line
	2000 1550 1500 1550
Connection ~ 1500 1550
Wire Wire Line
	2150 1250 2550 1250
$Comp
L pspice:CAP C1
U 1 1 5CE45A02
P 2550 1500
F 0 "C1" H 2728 1546 50  0000 L CNN
F 1 "1u" H 2728 1455 50  0000 L CNN
F 2 "" H 2550 1500 50  0001 C CNN
F 3 "~" H 2550 1500 50  0001 C CNN
	1    2550 1500
	1    0    0    -1  
$EndComp
Connection ~ 2550 1250
Wire Wire Line
	2550 1250 2700 1250
Wire Wire Line
	2550 1750 2150 1750
Connection ~ 2150 1750
Text GLabel 2700 1250 2    50   Input ~ 0
v_gnd
Wire Wire Line
	1800 2400 1900 2400
Wire Wire Line
	2400 2400 2600 2400
$Comp
L pspice:R R4
U 1 1 5CE4C217
P 1100 3550
F 0 "R4" H 1170 3596 50  0000 L CNN
F 1 "300" H 1170 3505 50  0000 L CNN
F 2 "" V 1030 3550 50  0001 C CNN
F 3 "~" H 1100 3550 50  0001 C CNN
	1    1100 3550
	1    0    0    -1  
$EndComp
$Comp
L pspice:CAP C2
U 1 1 5CE4CC70
P 2600 2950
F 0 "C2" H 2778 2996 50  0000 L CNN
F 1 "47p" H 2778 2905 50  0000 L CNN
F 2 "" H 2600 2950 50  0001 C CNN
F 3 "~" H 2600 2950 50  0001 C CNN
	1    2600 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 2700 2600 2400
Connection ~ 2600 2400
$Comp
L pspice:VSOURCE V1
U 1 1 5CE4D79A
P 1100 3000
F 0 "V1" H 1328 3046 50  0000 L CNN
F 1 "ac" H 1328 2955 50  0000 L CNN
F 2 "" H 1100 3000 50  0001 C CNN
F 3 "~" H 1100 3000 50  0001 C CNN
F 4 "V" H 1100 3000 50  0001 C CNN "Spice_Primitive"
F 5 "dc 0 ac 100m" H 1100 3000 50  0001 C CNN "Spice_Model"
F 6 "Y" H 1100 3000 50  0001 C CNN "Spice_Netlist_Enabled"
	1    1100 3000
	1    0    0    -1  
$EndComp
Text GLabel 1100 3800 0    50   Input ~ 0
v_gnd
Wire Wire Line
	1100 2200 1800 2200
Wire Wire Line
	1800 2200 1800 2400
Wire Wire Line
	1100 3800 1800 3800
Wire Wire Line
	1800 3800 1800 3550
Wire Wire Line
	2600 3200 2600 3550
Wire Wire Line
	1800 3550 2600 3550
Wire Wire Line
	2600 2400 3100 2400
Wire Wire Line
	3100 2800 3100 2400
Connection ~ 3100 2400
Wire Wire Line
	3100 2400 3300 2400
Wire Wire Line
	3100 3100 3100 3550
Wire Wire Line
	3100 3550 2600 3550
Connection ~ 2600 3550
Wire Wire Line
	1800 2650 1800 2400
Connection ~ 1800 2400
Wire Wire Line
	1800 3150 1800 3550
Connection ~ 1800 3550
Text Notes 2450 5050 0    200  ~ 0
Cant get opamps to work. Use ltspice models
$EndSCHEMATC
