EESchema Schematic File Version 4
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L pspice:VSOURCE V1
U 1 1 5CDFC0A8
P 2500 3500
F 0 "V1" H 2728 3546 50  0000 L CNN
F 1 "dc 0 ac 100" H 2728 3455 50  0000 L CNN
F 2 "" H 2500 3500 50  0001 C CNN
F 3 "~" H 2500 3500 50  0001 C CNN
F 4 "V" H 2500 3500 50  0001 C CNN "Spice_Primitive"
F 5 "dc 0 ac 48 pulse(0 24 0.00005 0.0000005 0.0000005 0.00005 0.0001)" H 2500 3500 50  0001 C CNN "Spice_Model"
F 6 "Y" H 2500 3500 50  0001 C CNN "Spice_Netlist_Enabled"
	1    2500 3500
	1    0    0    -1  
$EndComp
$Comp
L pspice:R R1
U 1 1 5CDFC38F
P 4950 2900
F 0 "R1" V 4745 2900 50  0000 C CNN
F 1 "200" V 4836 2900 50  0000 C CNN
F 2 "" H 4950 2900 50  0001 C CNN
F 3 "~" H 4950 2900 50  0001 C CNN
	1    4950 2900
	0    1    1    0   
$EndComp
$Comp
L pspice:INDUCTOR L1
U 1 1 5CDFD257
P 6050 3450
F 0 "L1" V 6004 3528 50  0000 L CNN
F 1 "2m" V 6095 3528 50  0000 L CNN
F 2 "" H 6050 3450 50  0001 C CNN
F 3 "~" H 6050 3450 50  0001 C CNN
	1    6050 3450
	0    1    1    0   
$EndComp
Wire Wire Line
	2500 3200 2500 2900
Wire Wire Line
	2500 2900 4700 2900
Wire Wire Line
	6050 4400 3450 4400
Wire Wire Line
	2500 4400 2500 3800
$Comp
L pspice:0 #GND01
U 1 1 5CDFDB85
P 3450 4400
F 0 "#GND01" H 3450 4300 50  0001 C CNN
F 1 "0" H 3450 4489 50  0000 C CNN
F 2 "" H 3450 4400 50  0001 C CNN
F 3 "~" H 3450 4400 50  0001 C CNN
	1    3450 4400
	1    0    0    -1  
$EndComp
Connection ~ 3450 4400
Wire Wire Line
	3450 4400 2500 4400
Text GLabel 6050 2900 2    50   Input ~ 0
out
Text GLabel 2500 2900 0    50   Input ~ 0
in
Text Notes 2850 5150 0    50   ~ 0
.ac dec 10 1 100k\n.tran 1u 100m 2m
$Comp
L pspice:R R3
U 1 1 5CDFF611
P 6050 3950
F 0 "R3" H 5982 3904 50  0000 R CNN
F 1 "12" H 5982 3995 50  0000 R CNN
F 2 "" H 6050 3950 50  0001 C CNN
F 3 "~" H 6050 3950 50  0001 C CNN
	1    6050 3950
	-1   0    0    1   
$EndComp
Text Notes 6750 3250 0    50   ~ 0
1000m, 1.5mm² => 4mH, 25R
Text Notes 2300 1900 0    50   ~ 0
When using a capacitator we get a resonant circuit:\nMaximum AC output at minimum Z\n=> w^2*L*C = 1\n=> C= 1/((2*Pi*f)^2*L)\nSo C depends on f which is hard to get right and make configurable\n==> NO CAPACIATOR
Text Notes 4150 1000 0    50   ~ 0
Inductance calculator:\nhttps://www.goetting-agv.com/components/inductancecalculator
Wire Wire Line
	6050 4200 6050 4400
Wire Wire Line
	6050 2900 6050 3200
Wire Wire Line
	5200 2900 6050 2900
Text Notes 4200 2600 0    50   ~ 0
R1 can be used to adapt for short wires
$EndSCHEMATC
