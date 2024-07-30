#pragma once
/* ||| Smart Prosthesis Socket Liner (SPSL) |||
	Author: A. B. Ambrose
	Date Created: 10/01/2023
	Decription: Custom library for controlling the SPSL	
	
	TO DO:
	- Implement capacitors on the GPIO inputs to decouple the buttons
	- Add test pins to see if things are running smoothly on oscilloscope



Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#ifndef SPSL_h
#define SPSL_h

// Macros
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

#define I1 0 		// X01 - Valve numbers
#define D1 6 		// X07
#define I2 1 		// X02
#define D2 7 		// X08
#define I3 2 		// X03
#define D3 8 		// X09
#define I4 3 		// X04
#define D4 9 		// X10
#define I5 4 		// X05
#define D5 10 		// X11
#define I6 5 		// X06
#define D6 11 		// X12

#define V1 27 		// X17
#define V2 28 		// X18
#define V3 26 		// X16

#define EV1 12		// X13 - Spare Valves
#define EV2 24		// X14 - Spare Valves
#define EV3 25		// X15 - Spare Valves

#define P1 29		// IF - Pump
#define P2 30		// IE - Pump

#define GPIO1 31	// Open Digital pin - Blue Debug
#define GPIO2 32	// Open Digital pin - Green Debug
#define GPIO3 41	// Open Digital pin - Yellow Debug

#define S1 34		// Buttons
#define S2 36
#define S3 35
#define S4 33

#define VIBR 37		// Vibration Motor - MOSFET Control Pin - 100 mA
#define LEDR 38		// Red LED
#define LEDG 39		// Green LED
#define LEDW 40		// White LED
#define BUZZ 14		// Buzzer (PWM)


// Declaring Global Vars
extern volatile float A[12]; // ADC Pressure value array
extern float C[12];
extern float B; // ADC value to Pa
extern float D; // gain for volumetric flow sensors to get L/s
extern volatile float V[6]; // vector to store the relative volumes of the bladders
extern volatile float M_dot[6];
extern int N; // Number of bladders in SPSL

extern volatile float T[4];
extern volatile float Temp;
extern float p1; // coefficients for converting Voltage to Temperature
extern float p2;
extern float p3;
extern float p4;

extern int32_t Acc1[3]; // IMU Vectors
extern int32_t Gyro1[3];
extern int32_t Acc2[3];
extern int32_t Gyro2[3];
extern int32_t Acc1Zero[3];
extern int32_t Gyro1Zero[3];
extern int32_t Acc2Zero[3];
extern int32_t Gyro2Zero[3];
extern float imu1[6];
extern float imu2[6];
extern int32_t calibrateTime;
extern float calibrateLen;

// Reference Pressure for every bladder
extern volatile float Ref;
extern float a;

extern volatile float U[16]; // Valve and Pump Command vector
extern float Kp; // Valve System Gains
extern float Ki;
extern float Kd;
extern float deadBand; // deadBand in Pa (do nothing if error is less than xxx)
extern float Kpv;
extern float Kiv;
extern float Kdv;
extern float deadBand_v;
extern float DC_res; // resolution of pwm

extern float Kp_pump; // pump system gains
extern float Ki_pump;
extern float Kd_pump;

extern volatile float R[8]; // vector of reference pressures
extern volatile float E[10]; // Pressure error array for bladders and source pressures
extern volatile float E_prev[10]; // Previous Pressure Error
extern volatile float E_dot[10]; // Derivative of Pressure Error
extern volatile float E_int[10]; // Integral of Pressure Error

extern volatile float E_v[6]; // Vector of adjusted pressure values
extern volatile float E_v_int[6]; //Inegral of volume error
extern volatile float E_v_dot[6]; // Derivative of colume error
extern volatile float E_v_prev[6];
extern volatile float U_v[12]; // Vector for volume control commands
extern float W[2];

extern float f1; // frequency of sensor acquisition timer
extern float f2; // frequency of valve control timer
extern float f_valve; // frequency of valve pwm

extern float G_thresh;
extern float M_thresh;
extern float sum_G[100];
extern float sum_M_dot[100];
extern float avg_G;
extern float avg_M_dot;

extern bool start; // start program flag
extern volatile uint8_t startCount;
extern volatile uint8_t teleoperateFlag;
extern volatile uint8_t stopCount; // toggle flag
extern bool newData; // flag for computing new errors and commands
extern int waitLen;
extern volatile int Count; // counter for emergency stop timeout
extern volatile float Time;
extern volatile bool detachFlag; //Flag to tell if the buttons are atached to interrupts
extern volatile int windowTime; // current index in window
extern int n; // window length for activity classification in seconds
extern volatile uint8_t classifyFlag; // flag for running activity classification
extern volatile float haltTime; // flag for halting classification
extern volatile float pauseClassify; // Time (s) to pause classifier for settling
extern volatile uint8_t curActivity; // keep track of current activity
extern volatile uint8_t prevActivity; // keep track of last activity

extern const int chipSelect;


// Function Declarations
extern void Initialize(void);
extern void Start(void);
extern void Idle(void);
extern void EStop(void);
extern void Vent(void);
extern void ControlUpdate(void);
extern void TIMER_1_ISR(void);
extern void TIMER_2_ISR(void);
extern void GPIO_1_ISR(void);
extern void GPIO_2_ISR(void);
extern void GPIO_3_ISR(void);
extern void GPIO_4_ISR(void);
extern void GPIOHandler(void);
extern void Calibrate(void);
extern void VibrateMotor(uint16_t t);
extern void StoreData(int message);

#endif //SPSL_h
