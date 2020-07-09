EESchema Schematic File Version 4
LIBS:ds2482_nano_master-cache
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
Text GLabel 4100 3600 0    50   Input ~ 0
SCL
Text GLabel 4100 3700 0    50   Input ~ 0
SDA
$Comp
L Connector:Screw_Terminal_01x04 J2
U 1 1 5EAC83A0
P 5050 5050
F 0 "J2" H 5130 5042 50  0000 L CNN
F 1 "Screw_Terminal_01x04" H 5130 4951 50  0000 L CNN
F 2 "TerminalBlock_RND:TerminalBlock_RND_205-00003_1x04_P5.00mm_Horizontal" H 5050 5050 50  0001 C CNN
F 3 "~" H 5050 5050 50  0001 C CNN
	1    5050 5050
	0    1    1    0   
$EndComp
Wire Wire Line
	4450 3350 4450 3200
$Comp
L ds2482-800:DS2482-800 U1
U 1 1 5EA8179C
P 4450 3250
F 0 "U1" H 4363 2596 50  0000 R CNN
F 1 "DS2482-800" H 4363 2505 50  0000 R CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 4450 3250 50  0001 C CNN
F 3 "" H 4450 3250 50  0001 C CNN
	1    4450 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 4950 7700 5100
Wire Wire Line
	7900 2950 8000 2950
Text GLabel 5300 4400 1    50   Input ~ 0
GND
Text GLabel 8000 2950 2    50   Input ~ 0
GND
Text GLabel 7700 5100 3    50   Input ~ 0
+5V
Text GLabel 4450 3200 1    50   Input ~ 0
+5V
Text GLabel 4450 4550 3    50   Input ~ 0
GND
$Comp
L Device:C C1
U 1 1 5EACB7FD
P 3850 4000
F 0 "C1" H 3965 4046 50  0000 L CNN
F 1 "220nF" H 3965 3955 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 3888 3850 50  0001 C CNN
F 3 "~" H 3850 4000 50  0001 C CNN
	1    3850 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C2
U 1 1 5EACBD99
P 5150 4500
F 0 "C2" H 5032 4454 50  0000 R CNN
F 1 "1000uF" H 5032 4545 50  0000 R CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 5188 4350 50  0001 C CNN
F 3 "~" H 5150 4500 50  0001 C CNN
	1    5150 4500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3850 3350 3850 3850
Connection ~ 4450 3350
Wire Wire Line
	3850 4150 3850 4550
Wire Wire Line
	3850 4550 4100 4550
Wire Wire Line
	4100 4100 4100 4200
Wire Wire Line
	4100 4300 4100 4200
Connection ~ 4100 4200
Wire Wire Line
	4100 4300 4100 4550
Connection ~ 4100 4300
Connection ~ 4100 4550
$Comp
L MCU_Module:Arduino_Nano_v3.x A1
U 1 1 5EA7DDC1
P 7900 3950
F 0 "A1" H 7900 2861 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 7900 2770 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 8050 3000 50  0001 L CNN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 7900 2950 50  0001 C CNN
	1    7900 3950
	-1   0    0    1   
$EndComp
Wire Wire Line
	4800 3800 4900 3800
Wire Wire Line
	7400 3550 7300 3550
Text GLabel 7300 3550 0    50   Input ~ 0
SDA
Wire Wire Line
	7400 3450 7300 3450
Text GLabel 7300 3450 0    50   Input ~ 0
SCL
Text GLabel 5250 2900 0    50   Input ~ 0
+5V
Text GLabel 8500 3950 2    50   Input ~ 0
ALARM
$Comp
L Device:R R1
U 1 1 5EB21A41
P 5000 4300
F 0 "R1" H 5070 4346 50  0000 L CNN
F 1 "10" H 5070 4255 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 4930 4300 50  0001 C CNN
F 3 "~" H 5000 4300 50  0001 C CNN
	1    5000 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4800 5050 4850
Wire Wire Line
	5300 4850 5150 4850
Wire Wire Line
	5300 4500 5300 4850
Wire Wire Line
	5300 4400 5300 4500
Connection ~ 5300 4500
Wire Wire Line
	5000 4450 5000 4500
Wire Wire Line
	5000 4500 5000 4800
Wire Wire Line
	5000 4800 5050 4800
Connection ~ 5000 4500
Wire Wire Line
	5000 4150 5000 4050
Text GLabel 5000 4050 1    50   Input ~ 0
+5V
Wire Wire Line
	4900 3800 4900 4400
Wire Wire Line
	4900 4800 4950 4800
Wire Wire Line
	4950 4800 4950 4850
Text Label 4900 4500 1    50   ~ 0
IO0
Wire Wire Line
	4450 3350 3850 3350
Wire Wire Line
	4100 4550 4350 4550
Wire Wire Line
	4100 4550 4450 4550
$Comp
L Connector:Screw_Terminal_01x04 J3
U 1 1 5EB39D85
P 5600 5050
F 0 "J3" H 5680 5042 50  0000 L CNN
F 1 "Screw_Terminal_01x04" H 5680 4951 50  0000 L CNN
F 2 "TerminalBlock_RND:TerminalBlock_RND_205-00003_1x04_P5.00mm_Horizontal" H 5600 5050 50  0001 C CNN
F 3 "~" H 5600 5050 50  0001 C CNN
	1    5600 5050
	0    1    1    0   
$EndComp
Text GLabel 5850 4400 1    50   Input ~ 0
GND
$Comp
L Device:CP C3
U 1 1 5EB39D8C
P 5700 4500
F 0 "C3" H 5582 4454 50  0000 R CNN
F 1 "1000uF" H 5582 4545 50  0000 R CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 5738 4350 50  0001 C CNN
F 3 "~" H 5700 4500 50  0001 C CNN
	1    5700 4500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 5EB39D93
P 5550 4300
F 0 "R2" H 5620 4346 50  0000 L CNN
F 1 "10" H 5620 4255 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 5480 4300 50  0001 C CNN
F 3 "~" H 5550 4300 50  0001 C CNN
	1    5550 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 4800 5600 4850
Wire Wire Line
	5850 4850 5700 4850
Wire Wire Line
	5850 4500 5850 4850
Wire Wire Line
	5850 4400 5850 4500
Connection ~ 5850 4500
Wire Wire Line
	5550 4450 5550 4500
Wire Wire Line
	5550 4500 5550 4800
Wire Wire Line
	5550 4800 5600 4800
Connection ~ 5550 4500
Wire Wire Line
	5550 4150 5550 4050
Text GLabel 5550 4050 1    50   Input ~ 0
+5V
Wire Wire Line
	5450 4800 5500 4800
Wire Wire Line
	5500 4800 5500 4850
$Comp
L Connector:Screw_Terminal_01x04 J4
U 1 1 5EB3BDC4
P 6150 5050
F 0 "J4" H 6230 5042 50  0000 L CNN
F 1 "Screw_Terminal_01x04" H 6230 4951 50  0000 L CNN
F 2 "TerminalBlock_RND:TerminalBlock_RND_205-00003_1x04_P5.00mm_Horizontal" H 6150 5050 50  0001 C CNN
F 3 "~" H 6150 5050 50  0001 C CNN
	1    6150 5050
	0    1    1    0   
$EndComp
Text GLabel 6400 4400 1    50   Input ~ 0
GND
$Comp
L Device:CP C4
U 1 1 5EB3BDCB
P 6250 4500
F 0 "C4" H 6132 4454 50  0000 R CNN
F 1 "1000uF" H 6132 4545 50  0000 R CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 6288 4350 50  0001 C CNN
F 3 "~" H 6250 4500 50  0001 C CNN
	1    6250 4500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 5EB3BDD2
P 6100 4300
F 0 "R3" H 6170 4346 50  0000 L CNN
F 1 "10" H 6170 4255 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 6030 4300 50  0001 C CNN
F 3 "~" H 6100 4300 50  0001 C CNN
	1    6100 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 4800 6150 4850
Wire Wire Line
	6400 4850 6250 4850
Wire Wire Line
	6400 4500 6400 4850
Wire Wire Line
	6400 4400 6400 4500
Connection ~ 6400 4500
Wire Wire Line
	6100 4450 6100 4500
Wire Wire Line
	6100 4500 6100 4800
Wire Wire Line
	6100 4800 6150 4800
Connection ~ 6100 4500
Wire Wire Line
	6100 4150 6100 4050
Text GLabel 6100 4050 1    50   Input ~ 0
+5V
Wire Wire Line
	6000 4800 6050 4800
Wire Wire Line
	6050 4800 6050 4850
$Comp
L Connector:Screw_Terminal_01x04 J5
U 1 1 5EB3D3BF
P 6700 5050
F 0 "J5" H 6780 5042 50  0000 L CNN
F 1 "Screw_Terminal_01x04" H 6780 4951 50  0000 L CNN
F 2 "TerminalBlock_RND:TerminalBlock_RND_205-00003_1x04_P5.00mm_Horizontal" H 6700 5050 50  0001 C CNN
F 3 "~" H 6700 5050 50  0001 C CNN
	1    6700 5050
	0    1    1    0   
$EndComp
Text GLabel 6950 4400 1    50   Input ~ 0
GND
$Comp
L Device:CP C5
U 1 1 5EB3D3C6
P 6800 4500
F 0 "C5" H 6682 4454 50  0000 R CNN
F 1 "1000uF" H 6682 4545 50  0000 R CNN
F 2 "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm" H 6838 4350 50  0001 C CNN
F 3 "~" H 6800 4500 50  0001 C CNN
	1    6800 4500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R4
U 1 1 5EB3D3CD
P 6650 4300
F 0 "R4" H 6720 4346 50  0000 L CNN
F 1 "10" H 6720 4255 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 6580 4300 50  0001 C CNN
F 3 "~" H 6650 4300 50  0001 C CNN
	1    6650 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 4800 6700 4850
Wire Wire Line
	6950 4850 6800 4850
Wire Wire Line
	6950 4500 6950 4850
Wire Wire Line
	6950 4400 6950 4500
Connection ~ 6950 4500
Wire Wire Line
	6650 4450 6650 4500
Wire Wire Line
	6650 4500 6650 4800
Wire Wire Line
	6650 4800 6700 4800
Connection ~ 6650 4500
Wire Wire Line
	6650 4150 6650 4050
Text GLabel 6650 4050 1    50   Input ~ 0
+5V
Wire Wire Line
	6550 4800 6600 4800
Wire Wire Line
	6600 4800 6600 4850
Wire Wire Line
	5450 3700 4800 3700
Wire Wire Line
	5450 3700 5450 4500
Wire Wire Line
	4800 3600 6000 3600
Wire Wire Line
	6000 3600 6000 4500
Wire Wire Line
	4800 3500 6550 3500
Wire Wire Line
	6550 3500 6550 4500
$Comp
L Connector_Generic:Conn_01x01 J7
U 1 1 5EB44E4F
P 4650 4850
F 0 "J7" H 4568 4625 50  0000 C CNN
F 1 "Conn_01x01" H 4568 4716 50  0000 C CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 4650 4850 50  0001 C CNN
F 3 "~" H 4650 4850 50  0001 C CNN
	1    4650 4850
	-1   0    0    1   
$EndComp
$Comp
L Device:R R5
U 1 1 5EB51C35
P 4900 4550
F 0 "R5" H 4970 4596 50  0000 L CNN
F 1 "R" H 4970 4505 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 4830 4550 50  0001 C CNN
F 3 "~" H 4900 4550 50  0001 C CNN
	1    4900 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 4700 4900 4800
$Comp
L Device:R R6
U 1 1 5EB52924
P 5450 4650
F 0 "R6" H 5520 4696 50  0000 L CNN
F 1 "R" H 5520 4605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 5380 4650 50  0001 C CNN
F 3 "~" H 5450 4650 50  0001 C CNN
	1    5450 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5EB52EFE
P 6000 4650
F 0 "R7" H 6070 4696 50  0000 L CNN
F 1 "R" H 6070 4605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 5930 4650 50  0001 C CNN
F 3 "~" H 6000 4650 50  0001 C CNN
	1    6000 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 5EB5317A
P 6550 4650
F 0 "R8" H 6620 4696 50  0000 L CNN
F 1 "R" H 6620 4605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 6480 4650 50  0001 C CNN
F 3 "~" H 6550 4650 50  0001 C CNN
	1    6550 4650
	1    0    0    -1  
$EndComp
Text GLabel 7250 3950 0    50   Input ~ 0
IO0
Text GLabel 4900 4150 0    50   Input ~ 0
IO0
Wire Wire Line
	7400 3950 7250 3950
Wire Wire Line
	8500 3950 8400 3950
Text GLabel 5450 4100 0    50   Input ~ 0
IO1
Text GLabel 7250 3850 0    50   Input ~ 0
IO1
Wire Wire Line
	7250 3850 7400 3850
Text GLabel 7250 3750 0    50   Input ~ 0
IO2
Text GLabel 7250 3650 0    50   Input ~ 0
IO3
Text GLabel 6000 4100 0    50   Input ~ 0
IO2
Text GLabel 6550 4100 0    50   Input ~ 0
IO3
Wire Wire Line
	7400 3750 7250 3750
Wire Wire Line
	7400 3650 7250 3650
$Comp
L Connector_Generic:Conn_01x01 J6
U 1 1 5EB6C5A7
P 5300 5200
F 0 "J6" V 5172 5280 50  0000 L CNN
F 1 "Conn_01x01" V 5263 5280 50  0000 L CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 5300 5200 50  0001 C CNN
F 3 "~" H 5300 5200 50  0001 C CNN
	1    5300 5200
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J9
U 1 1 5EB6D777
P 5850 5200
F 0 "J9" V 5722 5280 50  0000 L CNN
F 1 "Conn_01x01" V 5813 5280 50  0000 L CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 5850 5200 50  0001 C CNN
F 3 "~" H 5850 5200 50  0001 C CNN
	1    5850 5200
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J10
U 1 1 5EB6DA15
P 6400 5200
F 0 "J10" V 6272 5280 50  0000 L CNN
F 1 "Conn_01x01" V 6363 5280 50  0000 L CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 6400 5200 50  0001 C CNN
F 3 "~" H 6400 5200 50  0001 C CNN
	1    6400 5200
	0    1    1    0   
$EndComp
Wire Wire Line
	6500 4850 6450 4850
Wire Wire Line
	6450 4850 6450 4950
Wire Wire Line
	6450 4950 6400 4950
Wire Wire Line
	6400 4950 6400 5000
Wire Wire Line
	5950 4850 5900 4850
Wire Wire Line
	5900 4850 5900 4900
Wire Wire Line
	5900 4900 5850 4900
Wire Wire Line
	5850 4900 5850 5000
Wire Wire Line
	5400 4850 5350 4850
Wire Wire Line
	5350 4850 5350 4900
Wire Wire Line
	5350 4900 5300 4900
Wire Wire Line
	5300 4900 5300 5000
$Comp
L Connector:Screw_Terminal_01x02 J8
U 1 1 5EB7433E
P 5350 3200
F 0 "J8" H 5268 2875 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 5400 3300 50  0000 C CNN
F 2 "TerminalBlock_RND:TerminalBlock_RND_205-00001_1x02_P5.00mm_Horizontal" H 5350 3200 50  0001 C CNN
F 3 "~" H 5350 3200 50  0001 C CNN
	1    5350 3200
	-1   0    0    1   
$EndComp
Wire Wire Line
	6500 2800 6500 3200
Connection ~ 6500 2800
Wire Wire Line
	6450 2800 6500 2800
Wire Wire Line
	6500 2800 6550 2800
Wire Wire Line
	5800 2800 5800 2550
$Comp
L Device:Jumper JP1
U 1 1 5EB1CD42
P 5750 2900
F 0 "JP1" H 5750 3164 50  0000 C CNN
F 1 "Jumper" H 5750 3073 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5750 2900 50  0001 C CNN
F 3 "~" H 5750 2900 50  0001 C CNN
	1    5750 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 3000 6550 3000
Wire Wire Line
	6550 2900 6450 2900
Wire Wire Line
	5950 3000 5800 3000
Text GLabel 6550 2800 2    50   Input ~ 0
GND
Text GLabel 6550 2900 2    50   Input ~ 0
ALARM
Text GLabel 6550 3000 2    50   Input ~ 0
BUSY
Text GLabel 5800 3000 0    50   Input ~ 0
SCL
Text GLabel 5800 2550 1    50   Input ~ 0
SDA
Wire Wire Line
	5950 2800 5800 2800
$Comp
L Connector_Generic:Conn_02x03_Top_Bottom J1
U 1 1 5EAC6295
P 6150 2900
F 0 "J1" H 6200 3217 50  0000 C CNN
F 1 "Conn_02x03_Top_Bottom" H 6200 3126 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" H 6150 2900 50  0001 C CNN
F 3 "~" H 6150 2900 50  0001 C CNN
	1    6150 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 3200 6500 3200
Wire Wire Line
	5900 2900 5950 2900
Connection ~ 5950 2900
Wire Wire Line
	5950 2900 6050 2900
Wire Wire Line
	5550 2900 5550 3100
Connection ~ 5550 2900
Wire Wire Line
	5550 2900 5700 2900
Wire Wire Line
	5250 2900 5550 2900
$EndSCHEMATC
