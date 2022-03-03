EESchema Schematic File Version 4
EELAYER 30 0
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
L microchip-local:PIC18F27Q84 U1
U 1 1 604422A9
P 2750 4550
F 0 "U1" H 2750 5617 50  0000 C CNN
F 1 "PIC18F27Q84_ISS" H 2750 5526 50  0000 C CNN
F 2 "Package_SO:SOIC-28W_7.5x17.9mm_P1.27mm" H 2750 4300 50  0001 C CNN
F 3 "" H 2750 4500 50  0001 C CNN
F 4 "PIC18F27Q84_ISS" H 2750 4550 50  0001 C CNN "P/N"
	1    2750 4550
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LD1117S50TR_SOT223 U4
U 1 1 604437C3
P 6250 4000
F 0 "U4" H 6250 4050 50  0000 C CNN
F 1 "LDL1117S50" H 6250 3600 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 6275 3850 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/ua78.pdf" H 6250 3950 50  0001 C CNN
F 4 "LDL1117S50" H 6250 4000 50  0001 C CNN "P/N"
	1    6250 4000
	-1   0    0    1   
$EndComp
$Comp
L Device:Crystal_Small Y1
U 1 1 60444B4A
P 1650 4600
F 0 "Y1" V 1500 4550 50  0000 L CNN
F 1 "10Mhz" V 1800 4500 50  0000 L CNN
F 2 "Crystal:Crystal_HC18-U_Vertical" H 1650 4600 50  0001 C CNN
F 3 "~" H 1650 4600 50  0001 C CNN
	1    1650 4600
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C2
U 1 1 604455E3
P 1400 4700
F 0 "C2" V 1600 4700 50  0000 C CNN
F 1 "22pF" V 1500 4700 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D3.8mm_W2.6mm_P2.50mm" H 1400 4700 50  0001 C CNN
F 3 "~" H 1400 4700 50  0001 C CNN
	1    1400 4700
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR01
U 1 1 60445DAC
P 1250 4500
F 0 "#PWR01" H 1250 4250 50  0001 C CNN
F 1 "GND" V 1255 4372 50  0000 R CNN
F 2 "" H 1250 4500 50  0001 C CNN
F 3 "" H 1250 4500 50  0001 C CNN
	1    1250 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	1500 4500 1650 4500
Wire Wire Line
	1650 4500 1950 4500
Wire Wire Line
	1950 4500 1950 4550
Connection ~ 1650 4500
Wire Wire Line
	1650 4700 1500 4700
$Comp
L Device:C_Small C3
U 1 1 6044A0E3
P 1550 5050
F 0 "C3" H 1750 5000 50  0000 R CNN
F 1 "100nF" H 1850 5100 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 1550 5050 50  0001 C CNN
F 3 "~" H 1550 5050 50  0001 C CNN
	1    1550 5050
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR04
U 1 1 6044AD73
P 1850 4850
F 0 "#PWR04" H 1850 4700 50  0001 C CNN
F 1 "+3.3V" H 1865 5023 50  0000 C CNN
F 2 "" H 1850 4850 50  0001 C CNN
F 3 "" H 1850 4850 50  0001 C CNN
	1    1850 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 4850 1850 4850
Wire Wire Line
	1850 4850 1550 4850
Wire Wire Line
	1550 4850 1550 4950
Connection ~ 1850 4850
Wire Wire Line
	2000 5150 1950 5150
Connection ~ 1950 5150
Wire Wire Line
	1550 5150 1300 5150
Wire Wire Line
	1300 5150 1300 4700
Connection ~ 1550 5150
Connection ~ 1300 4700
Wire Wire Line
	1950 5250 1950 5150
Wire Wire Line
	3550 3950 4350 3950
Wire Wire Line
	3550 4050 4350 4050
$Comp
L power:GND #PWR015
U 1 1 6044E6A0
P 4850 4700
F 0 "#PWR015" H 4850 4450 50  0001 C CNN
F 1 "GND" H 4855 4527 50  0000 C CNN
F 2 "" H 4850 4700 50  0001 C CNN
F 3 "" H 4850 4700 50  0001 C CNN
	1    4850 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 4550 4850 4600
Wire Wire Line
	4350 4350 4350 4600
Wire Wire Line
	4350 4600 4850 4600
Connection ~ 4850 4600
Wire Wire Line
	4850 4600 4850 4700
$Comp
L power:+3.3V #PWR011
U 1 1 6044FE5A
P 4750 3500
F 0 "#PWR011" H 4750 3350 50  0001 C CNN
F 1 "+3.3V" H 4765 3673 50  0000 C CNN
F 2 "" H 4750 3500 50  0001 C CNN
F 3 "" H 4750 3500 50  0001 C CNN
	1    4750 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 60450282
P 5000 3500
F 0 "C4" V 5050 3450 50  0000 R CNN
F 1 "100nF" V 5100 3550 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5000 3500 50  0001 C CNN
F 3 "~" H 5000 3500 50  0001 C CNN
	1    5000 3500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR016
U 1 1 6045069B
P 5200 3500
F 0 "#PWR016" H 5200 3250 50  0001 C CNN
F 1 "GND" V 5205 3372 50  0000 R CNN
F 2 "" H 5200 3500 50  0001 C CNN
F 3 "" H 5200 3500 50  0001 C CNN
	1    5200 3500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5100 3500 5200 3500
Wire Wire Line
	5350 4150 5350 4050
Wire Wire Line
	6800 4150 6700 4250
Wire Wire Line
	6700 4250 5350 4250
Wire Wire Line
	6800 4250 6700 4150
Wire Wire Line
	6700 4150 5350 4150
$Comp
L Diode:1.5KExxA D2
U 1 1 6047203D
P 7100 3900
F 0 "D2" V 7150 4050 50  0000 R CNN
F 1 "SMBJ16A" H 7250 3800 50  0000 R CNN
F 2 "Diode_SMD:D_SMB_Handsoldering" H 7100 3700 50  0001 C CNN
F 3 "https://www.vishay.com/docs/88301/15ke.pdf" H 7050 3900 50  0001 C CNN
	1    7100 3900
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C9
U 1 1 6047790E
P 6600 3850
F 0 "C9" H 6508 3804 50  0000 R CNN
F 1 "10uF" H 6508 3895 50  0000 R CNN
F 2 "Capacitor_SMD:C_1210_3225Metric_Pad1.33x2.70mm_HandSolder" H 6600 3850 50  0001 C CNN
F 3 "~" H 6600 3850 50  0001 C CNN
	1    6600 3850
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C8
U 1 1 604783B2
P 5850 3850
F 0 "C8" H 6000 3800 50  0000 R CNN
F 1 "1uF" H 6100 3900 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5850 3850 50  0001 C CNN
F 3 "~" H 5850 3850 50  0001 C CNN
	1    5850 3850
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR021
U 1 1 60478ECB
P 7100 3650
F 0 "#PWR021" H 7100 3400 50  0001 C CNN
F 1 "GND" H 7105 3477 50  0000 C CNN
F 2 "" H 7100 3650 50  0001 C CNN
F 3 "" H 7100 3650 50  0001 C CNN
	1    7100 3650
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR018
U 1 1 604792EA
P 5750 4000
F 0 "#PWR018" H 5750 3850 50  0001 C CNN
F 1 "+5V" V 5765 4128 50  0000 L CNN
F 2 "" H 5750 4000 50  0001 C CNN
F 3 "" H 5750 4000 50  0001 C CNN
	1    5750 4000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5750 4000 5850 4000
Wire Wire Line
	5850 4000 5850 3950
Connection ~ 5850 4000
Wire Wire Line
	5850 4000 5950 4000
Wire Wire Line
	5850 3750 5850 3700
Wire Wire Line
	5850 3700 6250 3700
Connection ~ 7100 3750
Wire Wire Line
	7100 3650 7100 3750
Wire Wire Line
	7100 3750 6800 3750
Wire Wire Line
	6600 3750 6600 3700
Wire Wire Line
	6600 3700 6250 3700
Connection ~ 6600 3750
Connection ~ 6250 3700
Wire Wire Line
	6550 4000 6600 4000
Wire Wire Line
	6600 4000 6600 3950
Connection ~ 6600 4000
$Comp
L Device:R_Small R3
U 1 1 6048396A
P 1450 3750
F 0 "R3" V 1350 3750 50  0000 C CNN
F 1 "10K" V 1450 3750 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 1450 3750 50  0001 C CNN
F 3 "~" H 1450 3750 50  0001 C CNN
	1    1450 3750
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR02
U 1 1 60484075
P 1300 3700
F 0 "#PWR02" H 1300 3550 50  0001 C CNN
F 1 "+3.3V" H 1315 3873 50  0000 C CNN
F 2 "" H 1300 3700 50  0001 C CNN
F 3 "" H 1300 3700 50  0001 C CNN
	1    1300 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 3750 1300 3750
Wire Wire Line
	1300 3750 1300 3700
Text Label 1750 3750 0    50   ~ 0
MCLR
$Comp
L Connector:Conn_01x05_Male J3
U 1 1 60487D29
P 5050 5450
F 0 "J3" H 5100 5150 50  0000 R CNN
F 1 "Conn_01x05_Male" H 5350 5750 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 5050 5450 50  0001 C CNN
F 3 "~" H 5050 5450 50  0001 C CNN
	1    5050 5450
	-1   0    0    1   
$EndComp
Text Label 4600 5250 0    50   ~ 0
MCLR
$Comp
L power:GND #PWR012
U 1 1 6049394D
P 4750 5350
F 0 "#PWR012" H 4750 5100 50  0001 C CNN
F 1 "GND" V 4755 5222 50  0000 R CNN
F 2 "" H 4750 5350 50  0001 C CNN
F 3 "" H 4750 5350 50  0001 C CNN
	1    4750 5350
	0    1    1    0   
$EndComp
Wire Wire Line
	3550 4450 4100 4450
Wire Wire Line
	4850 5250 4600 5250
Wire Wire Line
	4750 5350 4850 5350
Text Label 4250 5550 0    50   ~ 0
PGD
Wire Wire Line
	3550 4350 4200 4350
Wire Wire Line
	4200 4350 4200 5450
Text Label 4250 5450 0    50   ~ 0
PGC
$Comp
L power:+3.3V #PWR013
U 1 1 6049A827
P 4750 5650
F 0 "#PWR013" H 4750 5500 50  0001 C CNN
F 1 "+3.3V" V 4765 5778 50  0000 L CNN
F 2 "" H 4750 5650 50  0001 C CNN
F 3 "" H 4750 5650 50  0001 C CNN
	1    4750 5650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4750 5650 4850 5650
$Comp
L Device:R_Small R8
U 1 1 6049C932
P 5150 5450
F 0 "R8" H 5100 5450 50  0000 R CNN
F 1 "10K" H 5100 5550 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 5150 5450 50  0001 C CNN
F 3 "~" H 5150 5450 50  0001 C CNN
	1    5150 5450
	-1   0    0    1   
$EndComp
Wire Wire Line
	5150 5350 4850 5350
Connection ~ 4850 5350
$Comp
L Device:C_Small C7
U 1 1 6049AEFE
P 5450 5350
F 0 "C7" H 5350 5300 50  0000 R CNN
F 1 "100nF" H 5350 5400 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5450 5350 50  0001 C CNN
F 3 "~" H 5450 5350 50  0001 C CNN
	1    5450 5350
	-1   0    0    1   
$EndComp
Wire Wire Line
	5450 5250 4850 5250
Connection ~ 4850 5250
Wire Wire Line
	5150 5350 5350 5350
Wire Wire Line
	5350 5350 5350 5450
Wire Wire Line
	5350 5450 5450 5450
Connection ~ 5150 5350
$Comp
L power:+12V #PWR020
U 1 1 604A3F02
P 6600 4000
F 0 "#PWR020" H 6600 3850 50  0001 C CNN
F 1 "+12V" H 6500 4050 50  0000 C CNN
F 2 "" H 6600 4000 50  0001 C CNN
F 3 "" H 6600 4000 50  0001 C CNN
	1    6600 4000
	-1   0    0    1   
$EndComp
Wire Wire Line
	7050 4050 7100 4050
Wire Wire Line
	7050 4000 7050 4050
$Comp
L Regulator_Linear:MIC5504-3.3YM5 U3
U 1 1 604B439A
P 5850 3150
F 0 "U3" H 5850 3200 50  0000 C CNN
F 1 "mic5504-3.3" H 5850 2750 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5_HandSoldering" H 5875 3000 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/ua78.pdf" H 5850 3100 50  0001 C CNN
F 4 "mic5504-3.3" H 5850 3150 50  0001 C CNN "P/N"
	1    5850 3150
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C6
U 1 1 604B4500
P 5450 3000
F 0 "C6" H 5600 2950 50  0000 R CNN
F 1 "1uF" H 5700 3050 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5450 3000 50  0001 C CNN
F 3 "~" H 5450 3000 50  0001 C CNN
	1    5450 3000
	-1   0    0    1   
$EndComp
$Comp
L power:+3.3V #PWR017
U 1 1 604B450A
P 5350 3250
F 0 "#PWR017" H 5350 3100 50  0001 C CNN
F 1 "+3.3V" V 5365 3378 50  0000 L CNN
F 2 "" H 5350 3250 50  0001 C CNN
F 3 "" H 5350 3250 50  0001 C CNN
	1    5350 3250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5450 2900 5450 2850
Wire Wire Line
	5450 2850 5850 2850
Connection ~ 5850 2850
Wire Wire Line
	5850 2850 6800 2850
Connection ~ 6800 3750
Wire Wire Line
	6800 3750 6600 3750
$Comp
L power:GND #PWR05
U 1 1 604C414B
P 3350 6150
F 0 "#PWR05" H 3350 5900 50  0001 C CNN
F 1 "GND" H 3450 6050 50  0000 R CNN
F 2 "" H 3350 6150 50  0001 C CNN
F 3 "" H 3350 6150 50  0001 C CNN
	1    3350 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 3500 4750 3750
Wire Wire Line
	4900 3500 4750 3500
Connection ~ 4750 3500
$Comp
L power:+5V #PWR014
U 1 1 604D4AE4
P 4850 3700
F 0 "#PWR014" H 4850 3550 50  0001 C CNN
F 1 "+5V" V 4865 3828 50  0000 L CNN
F 2 "" H 4850 3700 50  0001 C CNN
F 3 "" H 4850 3700 50  0001 C CNN
	1    4850 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 604D591C
P 5000 3750
F 0 "C5" V 5050 3700 50  0000 R CNN
F 1 "100nF" V 5100 3800 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder" H 5000 3750 50  0001 C CNN
F 3 "~" H 5000 3750 50  0001 C CNN
	1    5000 3750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4850 3750 4850 3700
Wire Wire Line
	4850 3750 4900 3750
Connection ~ 4850 3750
Wire Wire Line
	5200 3500 5200 3750
Wire Wire Line
	5200 3750 5100 3750
Connection ~ 5200 3500
$Comp
L Device:Q_NMOS_GSD Q3
U 1 1 604FF1AF
P 2850 5850
F 0 "Q3" H 3054 5896 50  0000 L CNN
F 1 "BSS123" H 3054 5805 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 3050 5950 50  0001 C CNN
F 3 "~" H 2850 5850 50  0001 C CNN
F 4 "BSS123 (stock)" H 2850 5850 50  0001 C CNN "P/N"
	1    2850 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R4
U 1 1 6050198F
P 2650 5950
F 0 "R4" H 2550 5950 50  0000 C CNN
F 1 "10K" V 2750 5950 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 2650 5950 50  0001 C CNN
F 3 "~" H 2650 5950 50  0001 C CNN
	1    2650 5950
	-1   0    0    1   
$EndComp
$Comp
L Device:R_Small R5
U 1 1 60507F6C
P 3600 5650
F 0 "R5" V 3500 5650 50  0000 C CNN
F 1 "1K" V 3600 5650 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 3600 5650 50  0001 C CNN
F 3 "~" H 3600 5650 50  0001 C CNN
	1    3600 5650
	0    1    1    0   
$EndComp
Wire Wire Line
	3700 5650 3750 5650
Wire Wire Line
	3500 5650 3450 5650
$Comp
L Connector:Screw_Terminal_01x03 J1
U 1 1 6052278E
P 2350 6250
F 0 "J1" V 2504 6062 50  0000 R CNN
F 1 "NMEA0183 (wind)" V 2413 6062 50  0000 R CNN
F 2 "TerminalBlock:TerminalBlock_bornier-3_P5.08mm" H 2350 6250 50  0001 C CNN
F 3 "~" H 2350 6250 50  0001 C CNN
F 4 "RS 790-1073" H 2350 6250 50  0001 C CNN "P/N"
	1    2350 6250
	0    -1   1    0   
$EndComp
Connection ~ 2950 6050
Wire Wire Line
	2650 5850 2350 5850
Wire Wire Line
	2350 5850 2350 6050
$Comp
L power:+12V #PWR03
U 1 1 60530E80
P 1700 5500
F 0 "#PWR03" H 1700 5350 50  0001 C CNN
F 1 "+12V" H 1650 5700 50  0000 C CNN
F 2 "" H 1700 5500 50  0001 C CNN
F 3 "" H 1700 5500 50  0001 C CNN
	1    1700 5500
	1    0    0    -1  
$EndComp
Connection ~ 2650 5850
Wire Wire Line
	2650 6050 2950 6050
Wire Wire Line
	2650 6050 2450 6050
Connection ~ 2650 6050
Wire Wire Line
	3550 5350 3550 5600
Wire Wire Line
	3550 5600 3450 5600
Wire Wire Line
	3450 5600 3450 5650
Connection ~ 3450 5650
Wire Wire Line
	3450 5650 2950 5650
$Comp
L Device:C_Small C1
U 1 1 604452D8
P 1400 4500
F 0 "C1" V 1171 4500 50  0000 C CNN
F 1 "22pF" V 1262 4500 50  0000 C CNN
F 2 "Capacitor_THT:C_Disc_D3.8mm_W2.6mm_P2.50mm" H 1400 4500 50  0001 C CNN
F 3 "~" H 1400 4500 50  0001 C CNN
	1    1400 4500
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R7
U 1 1 605B41CD
P 3950 5650
F 0 "R7" V 3850 5650 50  0000 C CNN
F 1 "4K7" V 3950 5650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 3950 5650 50  0001 C CNN
F 3 "~" H 3950 5650 50  0001 C CNN
	1    3950 5650
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R6
U 1 1 605CF8DB
P 3850 5500
F 0 "R6" V 3750 5500 50  0000 C CNN
F 1 "4K7" V 3850 5500 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 3850 5500 50  0001 C CNN
F 3 "~" H 3850 5500 50  0001 C CNN
	1    3850 5500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3550 4950 4050 4950
Wire Wire Line
	4050 4950 4050 5650
$Comp
L power:+3.3V #PWR010
U 1 1 604DEF60
P 3750 5450
F 0 "#PWR010" H 3750 5300 50  0001 C CNN
F 1 "+3.3V" H 3650 5600 50  0000 L CNN
F 2 "" H 3750 5450 50  0001 C CNN
F 3 "" H 3750 5450 50  0001 C CNN
	1    3750 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 5050 3950 5500
Wire Wire Line
	3750 5450 3750 5500
Wire Wire Line
	3750 5500 3750 5650
Connection ~ 3750 5500
Connection ~ 3750 5650
Wire Wire Line
	3750 5650 3850 5650
Wire Wire Line
	3550 5050 3950 5050
$Comp
L power:PWR_FLAG #FLG01
U 1 1 60667CDA
P 6800 2750
F 0 "#FLG01" H 6800 2825 50  0001 C CNN
F 1 "PWR_FLAG" H 6800 2923 50  0000 C CNN
F 2 "" H 6800 2750 50  0001 C CNN
F 3 "~" H 6800 2750 50  0001 C CNN
	1    6800 2750
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 606683BC
P 7350 3650
F 0 "#FLG02" H 7350 3725 50  0001 C CNN
F 1 "PWR_FLAG" H 7350 3823 50  0000 C CNN
F 2 "" H 7350 3650 50  0001 C CNN
F 3 "~" H 7350 3650 50  0001 C CNN
	1    7350 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 5450 4850 5450
Wire Wire Line
	4850 5550 4100 5550
Wire Wire Line
	4100 4450 4100 5550
Wire Wire Line
	4850 5550 5150 5550
Connection ~ 4850 5550
Text Label 5600 4150 0    50   ~ 0
CANH
Text Label 5600 4250 0    50   ~ 0
CANL
Text Label 3850 3950 0    50   ~ 0
CANTX
Text Label 3850 4050 0    50   ~ 0
CANRX
$Comp
L power:GND #PWR09
U 1 1 6069A878
P 3650 5150
F 0 "#PWR09" H 3650 4900 50  0001 C CNN
F 1 "GND" H 3655 4977 50  0000 C CNN
F 2 "" H 3650 5150 50  0001 C CNN
F 3 "" H 3650 5150 50  0001 C CNN
	1    3650 5150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3650 5150 3550 5150
Text Label 3600 4950 0    50   ~ 0
SCL
Text Label 3600 5050 0    50   ~ 0
SDA
Wire Wire Line
	1550 3750 1950 3750
Wire Wire Line
	1550 5150 1950 5150
Wire Wire Line
	1950 4650 1750 4650
Wire Wire Line
	1750 4650 1750 4700
Wire Wire Line
	1750 4700 1650 4700
Connection ~ 1650 4700
$Comp
L Interface_CAN_LIN:MCP2542FDxMF U2
U 1 1 60442D81
P 4850 4150
F 0 "U2" H 5000 3900 50  0000 C CNN
F 1 "TLE9250VSJXUMA1" H 5200 3750 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 4850 3650 50  0001 C CIN
F 3 "" H 4850 4150 50  0001 C CNN
F 4 "TLE9250VSJXUMA1" H 4850 4150 50  0001 C CNN "P/N"
	1    4850 4150
	1    0    0    -1  
$EndComp
$Comp
L Connector:RJ45 J4
U 1 1 6229A3AE
P 7900 3950
F 0 "J4" H 7570 4046 50  0000 R CNN
F 1 "RJ45" H 7570 3955 50  0000 R CNN
F 2 "Connector_RJ:RJ45_Amphenol_54602-x08_Horizontal" V 7900 3975 50  0001 C CNN
F 3 "~" V 7900 3975 50  0001 C CNN
F 4 "54601-908WPLF" H 7900 3950 50  0001 C CNN "P/N"
	1    7900 3950
	-1   0    0    -1  
$EndComp
$Comp
L Connector:RJ45 J5
U 1 1 6229A3B5
P 8850 3950
F 0 "J5" H 8520 4046 50  0000 R CNN
F 1 "RJ45" H 8520 3955 50  0000 R CNN
F 2 "Connector_RJ:RJ45_Amphenol_54602-x08_Horizontal" V 8850 3975 50  0001 C CNN
F 3 "~" V 8850 3975 50  0001 C CNN
F 4 "54601-908WPLF" H 8850 3950 50  0001 C CNN "P/N"
	1    8850 3950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7500 3550 8450 3550
Wire Wire Line
	8450 3650 7500 3650
Wire Wire Line
	7500 3750 8450 3750
Wire Wire Line
	8450 3850 7500 3850
Wire Wire Line
	7500 3950 8450 3950
Wire Wire Line
	8450 4050 7500 4050
Wire Wire Line
	7500 4150 8450 4150
Wire Wire Line
	8450 4250 7500 4250
Connection ~ 7500 4150
Connection ~ 7500 4250
Wire Wire Line
	7500 3750 7500 3850
Connection ~ 7500 3750
Connection ~ 7500 3850
Connection ~ 7500 4050
Wire Wire Line
	7500 4050 7500 3950
Connection ~ 7500 3950
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J6
U 1 1 6229A3CD
P 9700 4150
F 0 "J6" H 9750 4467 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 9750 4376 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" H 9700 4150 50  0001 C CNN
F 3 "~" H 9700 4150 50  0001 C CNN
	1    9700 4150
	-1   0    0    -1  
$EndComp
Connection ~ 8450 4150
Wire Wire Line
	8450 4250 9200 4250
Wire Wire Line
	9200 4250 9200 4550
Wire Wire Line
	9200 4550 10050 4550
Wire Wire Line
	10050 4550 10050 4150
Wire Wire Line
	10050 4150 9900 4150
Connection ~ 8450 4250
Wire Wire Line
	9400 4050 9400 3650
Wire Wire Line
	9400 3650 8450 3650
Connection ~ 8450 3650
Wire Wire Line
	8450 3550 9900 3550
Wire Wire Line
	9900 3550 9900 4050
Connection ~ 8450 3550
$Comp
L Device:R_Small R9
U 1 1 6229A3E1
P 9650 4450
F 0 "R9" V 9550 4450 50  0000 C CNN
F 1 "100" V 9650 4450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder" H 9650 4450 50  0001 C CNN
F 3 "~" H 9650 4450 50  0001 C CNN
F 4 "MCWF08P1000FTL" H 9650 4450 50  0001 C CNN "P/N"
	1    9650 4450
	0    1    1    0   
$EndComp
Wire Wire Line
	9400 4250 9400 4450
Wire Wire Line
	9400 4450 9550 4450
Wire Wire Line
	9900 4250 9900 4450
Wire Wire Line
	9900 4450 9750 4450
Wire Wire Line
	7500 4050 7350 4050
Wire Wire Line
	8450 4150 9400 4150
Wire Wire Line
	7100 3750 7500 3750
Connection ~ 7100 4050
Wire Wire Line
	6800 4250 7500 4250
Wire Wire Line
	6800 4150 7500 4150
$Comp
L Device:Q_NMOS_GSD Q1
U 1 1 621ED54D
P 1350 6600
F 0 "Q1" H 1554 6646 50  0000 L CNN
F 1 "BSS123" H 1554 6555 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1550 6700 50  0001 C CNN
F 3 "~" H 1350 6600 50  0001 C CNN
F 4 "BSS123 (stock)" H 1350 6600 50  0001 C CNN "P/N"
	1    1350 6600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R1
U 1 1 621EE51D
P 1100 6800
F 0 "R1" H 1000 6800 50  0000 C CNN
F 1 "10K" V 1200 6800 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 1100 6800 50  0001 C CNN
F 3 "~" H 1100 6800 50  0001 C CNN
	1    1100 6800
	-1   0    0    1   
$EndComp
$Comp
L Device:Q_PMOS_GSD Q2
U 1 1 621F0BF6
P 1600 5800
F 0 "Q2" H 1804 5754 50  0000 L CNN
F 1 "DMG2307L" H 1804 5845 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1800 5900 50  0001 C CNN
F 3 "~" H 1600 5800 50  0001 C CNN
F 4 "DMG2307L" H 1600 5800 50  0001 C CNN "P/N"
	1    1600 5800
	1    0    0    1   
$EndComp
$Comp
L Device:R_Small R2
U 1 1 62206AEA
P 1350 5650
F 0 "R2" H 1250 5650 50  0000 C CNN
F 1 "10K" V 1450 5650 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 1350 5650 50  0001 C CNN
F 3 "~" H 1350 5650 50  0001 C CNN
	1    1350 5650
	-1   0    0    1   
$EndComp
Wire Wire Line
	1700 5500 1700 5550
Wire Wire Line
	1350 5550 1700 5550
Connection ~ 1700 5550
Wire Wire Line
	1700 5550 1700 5600
Wire Wire Line
	1700 6000 2050 6000
Wire Wire Line
	2250 6000 2250 6050
Wire Wire Line
	1350 5750 1350 5800
Wire Wire Line
	1350 5800 1400 5800
Wire Wire Line
	1350 5800 1350 6200
Wire Wire Line
	1350 6200 1450 6200
Wire Wire Line
	1450 6200 1450 6400
Connection ~ 1350 5800
Wire Wire Line
	1150 6600 1100 6600
Wire Wire Line
	1100 6600 1100 6700
Wire Wire Line
	1100 6900 1450 6900
Wire Wire Line
	1450 6900 1450 6800
Wire Wire Line
	2950 6050 2950 6900
Wire Wire Line
	2950 6900 2050 6900
Connection ~ 1450 6900
Text Label 900  6600 0    50   ~ 0
12V_ON
Wire Wire Line
	900  6600 1100 6600
Connection ~ 1100 6600
Text Label 3600 5250 0    50   ~ 0
12V_ON
Wire Wire Line
	3600 5250 3550 5250
Wire Wire Line
	7350 3650 7350 4050
Connection ~ 7350 4050
Wire Wire Line
	7350 4050 7100 4050
Wire Wire Line
	6600 4000 7050 4000
Wire Wire Line
	6800 2750 6800 2850
Connection ~ 6800 2850
Wire Wire Line
	6800 2850 6800 3750
$Comp
L power:+5V #PWR019
U 1 1 62276FE5
P 6450 3250
F 0 "#PWR019" H 6450 3100 50  0001 C CNN
F 1 "+5V" V 6465 3378 50  0000 L CNN
F 2 "" H 6450 3250 50  0001 C CNN
F 3 "" H 6450 3250 50  0001 C CNN
	1    6450 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	5450 3100 5450 3250
Wire Wire Line
	5450 3250 5350 3250
Connection ~ 5450 3250
Wire Wire Line
	6250 3250 6350 3250
Wire Wire Line
	6250 3050 6350 3050
Wire Wire Line
	6350 3050 6350 3250
Connection ~ 6350 3250
Wire Wire Line
	6350 3250 6450 3250
Wire Wire Line
	3350 6150 3350 6050
Connection ~ 3350 6050
Wire Wire Line
	3350 6050 2950 6050
Wire Wire Line
	3850 6400 3850 6050
Wire Wire Line
	3950 5500 3950 5950
Connection ~ 3950 5500
Wire Wire Line
	3950 5950 4050 6050
Wire Wire Line
	4050 6050 4050 6400
Wire Wire Line
	4050 5650 4050 5950
Wire Wire Line
	4050 5950 3950 6050
Wire Wire Line
	3950 6050 3950 6400
Connection ~ 4050 5650
Wire Wire Line
	4150 6400 4150 6250
Wire Wire Line
	4150 6250 3750 6250
Connection ~ 3750 6250
Wire Wire Line
	3750 6250 3750 6400
Wire Wire Line
	3750 5650 3750 6250
Wire Wire Line
	4150 6250 4250 6250
Wire Wire Line
	4250 6250 4250 6400
Connection ~ 4150 6250
$Comp
L Diode:1.5KExxA D1
U 1 1 622FFB1C
P 2050 6200
F 0 "D1" V 2100 6350 50  0000 R CNN
F 1 "SMBJ16A" H 2200 6100 50  0000 R CNN
F 2 "Diode_SMD:D_SMB_Handsoldering" H 2050 6000 50  0001 C CNN
F 3 "https://www.vishay.com/docs/88301/15ke.pdf" H 2000 6200 50  0001 C CNN
	1    2050 6200
	0    1    1    0   
$EndComp
Wire Wire Line
	2050 6050 2050 6000
Connection ~ 2050 6000
Wire Wire Line
	2050 6000 2250 6000
Wire Wire Line
	2050 6350 2050 6900
Connection ~ 2050 6900
Wire Wire Line
	2050 6900 1450 6900
$Comp
L power:GND #PWR08
U 1 1 62313206
P 3650 4750
F 0 "#PWR08" H 3650 4500 50  0001 C CNN
F 1 "GND" H 3655 4577 50  0000 C CNN
F 2 "" H 3650 4750 50  0001 C CNN
F 3 "" H 3650 4750 50  0001 C CNN
	1    3650 4750
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3550 4650 3600 4650
Wire Wire Line
	3600 4650 3600 4750
Wire Wire Line
	3600 4750 3650 4750
Wire Wire Line
	3600 4750 3550 4750
Connection ~ 3600 4750
Wire Wire Line
	3600 4750 3600 4850
Wire Wire Line
	3600 4850 3550 4850
$Comp
L power:GND #PWR07
U 1 1 6232EA8C
P 3650 4200
F 0 "#PWR07" H 3650 3950 50  0001 C CNN
F 1 "GND" H 3655 4027 50  0000 C CNN
F 2 "" H 3650 4200 50  0001 C CNN
F 3 "" H 3650 4200 50  0001 C CNN
	1    3650 4200
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR06
U 1 1 6232F043
P 4050 3850
F 0 "#PWR06" H 4050 3600 50  0001 C CNN
F 1 "GND" H 4055 3677 50  0000 C CNN
F 2 "" H 4050 3850 50  0001 C CNN
F 3 "" H 4050 3850 50  0001 C CNN
	1    4050 3850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3550 3750 3600 3750
Wire Wire Line
	3550 4150 3600 4150
Wire Wire Line
	3600 4150 3600 4200
Wire Wire Line
	3600 4250 3550 4250
Wire Wire Line
	3650 4200 3600 4200
Connection ~ 3600 4200
Wire Wire Line
	3600 4200 3600 4250
Wire Wire Line
	1950 3850 1850 3850
Wire Wire Line
	1850 3850 1850 3950
Wire Wire Line
	1850 4350 1950 4350
Wire Wire Line
	1950 4250 1850 4250
Connection ~ 1850 4250
Wire Wire Line
	1850 4250 1850 4350
Wire Wire Line
	1950 4150 1850 4150
Connection ~ 1850 4150
Wire Wire Line
	1850 4150 1850 4250
Wire Wire Line
	1950 4050 1850 4050
Connection ~ 1850 4050
Wire Wire Line
	1850 4050 1850 4150
Wire Wire Line
	1950 3950 1850 3950
Connection ~ 1850 3950
Wire Wire Line
	1850 3950 1850 4050
Wire Wire Line
	1850 3950 1300 3950
Wire Wire Line
	1300 3950 1300 4500
Connection ~ 1300 4500
Wire Wire Line
	1300 4500 1300 4700
Wire Wire Line
	1250 4500 1300 4500
Wire Wire Line
	3350 6050 3850 6050
$Comp
L Connector:Conn_01x06_Male J2
U 1 1 62219342
P 3950 6600
F 0 "J2" V 4104 6212 50  0000 R CNN
F 1 "Conn_01x06_Male" V 4013 6212 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 3950 6600 50  0001 C CNN
F 3 "~" H 3950 6600 50  0001 C CNN
	1    3950 6600
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED_Small D3
U 1 1 6221A5B0
P 3950 3550
F 0 "D3" V 3996 3480 50  0000 R CNN
F 1 "red" V 3905 3480 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm" V 3950 3550 50  0001 C CNN
F 3 "~" V 3950 3550 50  0001 C CNN
	1    3950 3550
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R10
U 1 1 6221BF2C
P 3750 3450
F 0 "R10" H 3650 3450 50  0000 C CNN
F 1 "330" V 3850 3450 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" H 3750 3450 50  0001 C CNN
F 3 "~" H 3750 3450 50  0001 C CNN
	1    3750 3450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3600 3750 3600 3450
Wire Wire Line
	3600 3450 3650 3450
Wire Wire Line
	3850 3450 3950 3450
Wire Wire Line
	3950 3650 3950 3850
Wire Wire Line
	3950 3850 4050 3850
Wire Wire Line
	3550 3850 3950 3850
Connection ~ 3950 3850
$Comp
L Mechanical:MountingHole H1
U 1 1 62257829
P 6700 4850
F 0 "H1" H 6800 4896 50  0000 L CNN
F 1 "MountingHole" H 6800 4805 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.5mm" H 6700 4850 50  0001 C CNN
F 3 "~" H 6700 4850 50  0001 C CNN
	1    6700 4850
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 62257F08
P 6700 5050
F 0 "H2" H 6800 5096 50  0000 L CNN
F 1 "MountingHole" H 6800 5005 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.5mm" H 6700 5050 50  0001 C CNN
F 3 "~" H 6700 5050 50  0001 C CNN
	1    6700 5050
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 62258377
P 6700 5250
F 0 "H3" H 6800 5296 50  0000 L CNN
F 1 "MountingHole" H 6800 5205 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.5mm" H 6700 5250 50  0001 C CNN
F 3 "~" H 6700 5250 50  0001 C CNN
	1    6700 5250
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 6225869C
P 6700 5450
F 0 "H4" H 6800 5496 50  0000 L CNN
F 1 "MountingHole" H 6800 5405 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.5mm" H 6700 5450 50  0001 C CNN
F 3 "~" H 6700 5450 50  0001 C CNN
	1    6700 5450
	1    0    0    -1  
$EndComp
$EndSCHEMATC
