EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Gait Analysis Schematic Circuit"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L power:GND #PWR0105
U 1 1 5F1FD677
P 3950 2400
F 0 "#PWR0105" H 3950 2150 50  0001 C CNN
F 1 "GND" H 3955 2227 50  0000 C CNN
F 2 "" H 3950 2400 50  0001 C CNN
F 3 "" H 3950 2400 50  0001 C CNN
	1    3950 2400
	0    1    1    0   
$EndComp
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5F206CBB
P 4700 1000
F 0 "#FLG0101" H 4700 1075 50  0001 C CNN
F 1 "PWR_FLAG" H 4700 1173 50  0000 C CNN
F 2 "" H 4700 1000 50  0001 C CNN
F 3 "~" H 4700 1000 50  0001 C CNN
	1    4700 1000
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5F208421
P 5150 1000
F 0 "#FLG0102" H 5150 1075 50  0001 C CNN
F 1 "PWR_FLAG" H 5150 1173 50  0000 C CNN
F 2 "" H 5150 1000 50  0001 C CNN
F 3 "~" H 5150 1000 50  0001 C CNN
	1    5150 1000
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0108
U 1 1 5F209414
P 4700 1000
F 0 "#PWR0108" H 4700 850 50  0001 C CNN
F 1 "VDD" H 4715 1173 50  0000 C CNN
F 2 "" H 4700 1000 50  0001 C CNN
F 3 "" H 4700 1000 50  0001 C CNN
	1    4700 1000
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 5F20ADF0
P 5150 1000
F 0 "#PWR0109" H 5150 750 50  0001 C CNN
F 1 "GND" H 5155 827 50  0000 C CNN
F 2 "" H 5150 1000 50  0001 C CNN
F 3 "" H 5150 1000 50  0001 C CNN
	1    5150 1000
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 5F20CEAC
P 5600 1000
F 0 "J1" H 5708 1181 50  0000 C CNN
F 1 "Conn_01x02_Male" H 5708 1090 50  0000 C CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x02_P1.00mm_Horizontal" H 5600 1000 50  0001 C CNN
F 3 "~" H 5600 1000 50  0001 C CNN
	1    5600 1000
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0112
U 1 1 5F2271EB
P 3100 5500
F 0 "#PWR0112" H 3100 5350 50  0001 C CNN
F 1 "VDD" V 3100 5050 50  0000 L CNN
F 2 "" H 3100 5500 50  0001 C CNN
F 3 "" H 3100 5500 50  0001 C CNN
	1    3100 5500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 5F227721
P 3100 5400
F 0 "#PWR0113" H 3100 5150 50  0001 C CNN
F 1 "GND" V 3100 5800 50  0000 C CNN
F 2 "" H 3100 5400 50  0001 C CNN
F 3 "" H 3100 5400 50  0001 C CNN
	1    3100 5400
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x15_Female J3
U 1 1 5F1E819B
P 2900 4800
F 0 "J3" H 2750 3850 50  0000 L CNN
F 1 "Conn_01x15_Female" H 2450 3950 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x15_P2.54mm_Vertical" H 2900 4800 50  0001 C CNN
F 3 "~" H 2900 4800 50  0001 C CNN
	1    2900 4800
	-1   0    0    1   
$EndComp
NoConn ~ 3100 2300
NoConn ~ 3100 2400
NoConn ~ 3100 2700
NoConn ~ 3100 2800
NoConn ~ 3100 2900
NoConn ~ 3100 3000
NoConn ~ 3100 4100
NoConn ~ 3100 4200
NoConn ~ 3100 4300
NoConn ~ 3100 4500
NoConn ~ 3100 4600
NoConn ~ 3100 4800
NoConn ~ 3100 4900
NoConn ~ 3100 5100
NoConn ~ 3100 5200
NoConn ~ 3100 5300
$Comp
L power:+3V3 #PWR0106
U 1 1 5F248771
P 3100 3500
F 0 "#PWR0106" H 3100 3350 50  0001 C CNN
F 1 "+3V3" V 3115 3628 50  0000 L CNN
F 2 "" H 3100 3500 50  0001 C CNN
F 3 "" H 3100 3500 50  0001 C CNN
	1    3100 3500
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR0107
U 1 1 5F24C5CA
P 3950 2300
F 0 "#PWR0107" H 3950 2150 50  0001 C CNN
F 1 "+3V3" V 3965 2428 50  0000 L CNN
F 2 "" H 3950 2300 50  0001 C CNN
F 3 "" H 3950 2300 50  0001 C CNN
	1    3950 2300
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x08_Female J4
U 1 1 5F271242
P 4150 2600
F 0 "J4" H 4200 3100 50  0000 L CNN
F 1 "Conn_01x08_Female" H 3900 3000 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical" H 4150 2600 50  0001 C CNN
F 3 "~" H 4150 2600 50  0001 C CNN
	1    4150 2600
	1    0    0    -1  
$EndComp
Text Notes 4200 2300 0    39   ~ 0
VCC
Text Notes 4200 2400 0    39   ~ 0
GND\n
Text Notes 4200 2500 0    39   ~ 0
SCL\n
Text Notes 4200 2600 0    39   ~ 0
SDA
Text Notes 4200 2700 0    39   ~ 0
XDA
Text Notes 4200 2800 0    39   ~ 0
XCL
Text Notes 4200 2900 0    39   ~ 0
AD0
Text Notes 4200 3000 0    39   ~ 0
INT\n
Wire Wire Line
	3500 2500 3500 2600
Wire Wire Line
	3500 2600 3950 2600
Wire Wire Line
	3100 2500 3500 2500
Wire Wire Line
	3100 2200 3550 2200
Wire Wire Line
	3550 2200 3550 2500
Wire Wire Line
	3550 2500 3950 2500
$Comp
L power:GND #PWR0104
U 1 1 5F1FCCBA
P 3100 3400
F 0 "#PWR0104" H 3100 3150 50  0001 C CNN
F 1 "GND" H 3105 3227 50  0000 C CNN
F 2 "" H 3100 3400 50  0001 C CNN
F 3 "" H 3100 3400 50  0001 C CNN
	1    3100 3400
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5F1FC7BD
P 4750 3500
F 0 "#PWR0103" H 4750 3250 50  0001 C CNN
F 1 "GND" V 4755 3372 50  0000 R CNN
F 2 "" H 4750 3500 50  0001 C CNN
F 3 "" H 4750 3500 50  0001 C CNN
	1    4750 3500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5F1FA691
P 5550 3150
F 0 "#PWR0102" H 5550 2900 50  0001 C CNN
F 1 "GND" V 5555 3022 50  0000 R CNN
F 2 "" H 5550 3150 50  0001 C CNN
F 3 "" H 5550 3150 50  0001 C CNN
	1    5550 3150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5F1F9EA2
P 4700 3900
F 0 "#PWR0101" H 4700 3650 50  0001 C CNN
F 1 "GND" V 4705 3772 50  0000 R CNN
F 2 "" H 4700 3900 50  0001 C CNN
F 3 "" H 4700 3900 50  0001 C CNN
	1    4700 3900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4700 3500 4750 3500
Wire Wire Line
	5500 3150 5550 3150
Wire Wire Line
	4650 3900 4700 3900
$Comp
L Device:R R3
U 1 1 5F1F330B
P 4250 3500
F 0 "R3" V 4043 3500 50  0000 C CNN
F 1 "R" V 4134 3500 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 4180 3500 50  0001 C CNN
F 3 "~" H 4250 3500 50  0001 C CNN
	1    4250 3500
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5F1F2FB6
P 5050 3150
F 0 "R2" V 4843 3150 50  0000 C CNN
F 1 "R" V 4934 3150 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 4980 3150 50  0001 C CNN
F 3 "~" H 5050 3150 50  0001 C CNN
	1    5050 3150
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 5F1F229B
P 4200 3900
F 0 "R1" V 3993 3900 50  0000 C CNN
F 1 "R" V 4084 3900 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 4130 3900 50  0001 C CNN
F 3 "~" H 4200 3900 50  0001 C CNN
	1    4200 3900
	0    1    1    0   
$EndComp
$Comp
L Device:LED D1
U 1 1 5F1BC301
P 4500 3900
F 0 "D1" H 4493 3645 50  0000 C CNN
F 1 "LED" H 4493 3736 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" H 4500 3900 50  0001 C CNN
F 3 "~" H 4500 3900 50  0001 C CNN
	1    4500 3900
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D2
U 1 1 5F1BBDAB
P 5350 3150
F 0 "D2" H 5343 2895 50  0000 C CNN
F 1 "LED" H 5343 2986 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" H 5350 3150 50  0001 C CNN
F 3 "~" H 5350 3150 50  0001 C CNN
	1    5350 3150
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D3
U 1 1 5F1BA6DF
P 4550 3500
F 0 "D3" H 4543 3245 50  0000 C CNN
F 1 "LED" H 4543 3336 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" H 4550 3500 50  0001 C CNN
F 3 "~" H 4550 3500 50  0001 C CNN
	1    4550 3500
	-1   0    0    1   
$EndComp
NoConn ~ 3100 2600
NoConn ~ 3950 2700
NoConn ~ 3950 2800
NoConn ~ 3950 2900
$Comp
L power:GND #PWR0111
U 1 1 5F21021F
P 5800 1100
F 0 "#PWR0111" H 5800 850 50  0001 C CNN
F 1 "GND" V 5805 972 50  0000 R CNN
F 2 "" H 5800 1100 50  0001 C CNN
F 3 "" H 5800 1100 50  0001 C CNN
	1    5800 1100
	0    -1   -1   0   
$EndComp
$Comp
L power:VDD #PWR0110
U 1 1 5F20F9CA
P 5800 1000
F 0 "#PWR0110" H 5800 850 50  0001 C CNN
F 1 "VDD" V 5815 1128 50  0000 L CNN
F 2 "" H 5800 1000 50  0001 C CNN
F 3 "" H 5800 1000 50  0001 C CNN
	1    5800 1000
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x15_Female J2
U 1 1 5F1E59ED
P 2900 2800
F 0 "J2" H 2750 1850 50  0000 L CNN
F 1 "Conn_01x15_Female_MPU" H 2400 1950 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x15_P2.54mm_Vertical" H 2900 2800 50  0001 C CNN
F 3 "~" H 2900 2800 50  0001 C CNN
	1    2900 2800
	-1   0    0    1   
$EndComp
Wire Wire Line
	3950 3000 3450 3000
Wire Wire Line
	3450 3000 3450 2100
Wire Wire Line
	3450 2100 3100 2100
Wire Wire Line
	3100 3300 3900 3300
Wire Wire Line
	3900 3300 3900 3500
Wire Wire Line
	3900 3500 4100 3500
NoConn ~ 3100 5000
Wire Wire Line
	3100 3200 4600 3200
Wire Wire Line
	4600 3200 4600 3150
Wire Wire Line
	4600 3150 4900 3150
Wire Wire Line
	3100 3100 3650 3100
Wire Wire Line
	3650 3100 3650 3900
Wire Wire Line
	3650 3900 4050 3900
NoConn ~ 3100 4400
NoConn ~ 3100 4700
$EndSCHEMATC
