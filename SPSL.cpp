/*
	Author: A. B. Ambrose
	Date Created: 10/01/2023
	Decription: Custom library for controlling the SPSL	
	
	///// TO DO /////:
	- Implement a flyback diode on the vibration motor wire

*/
#include <Metro.h> // Include the Metro library
#include <Wire.h> // I2C Library
#include <ADS1X15.h> // Custom Library for ADCs
#include <LSM6DS3Sensor.h> // Custom Library for IMUs
#include "SPSL.h"
#include <SD.h>

// Global Vars
volatile float A[12] = {0}; // ADC Pressure value array
float C[12] = {106.5, 106.8, 103.2, 108.6, 106.0, 105.5, 107.9, 102.0, 106.9, 106.1, 0.504, 0.507}; // Offsets for sensors
float B = 48.852; // ADC value to Pa
float D = 0.0264; // gain for mass flow sensors to get g/s
volatile float V[6] = {0}; // vector to store the relative volumes of the bladders
volatile float M_dot[6] = {0};
int N = 6; // Number of bladders in SPSL


volatile float T[4] = {0}; // Temperature Vector
volatile float Temp = 0;
float p1 = 6.196; // coefficients for converting Voltage to Temperature
float p2 = -28.42;
float p3 = 69.59;
float p4 = -40.81;

int32_t Acc1[3] = {0}; // 3-axis accelerations for IMU1
int32_t Gyro1[3] = {0}; // 3-axis angular velocity for IMU1
int32_t Acc2[3] = {0}; // 3-axis accelerations for IMU2
int32_t Gyro2[3] = {0}; // 3-axis angular velocity for IMU2
int32_t Acc1Zero[3] = {0}; // Offsets for IMU data
int32_t Gyro1Zero[3] = {0};
int32_t Acc2Zero[3] = {0};
int32_t Gyro2Zero[3] = {0};
float imu1[6] = {0};
float imu2[6] = {0};
int32_t calibrateTime = 0; // initialize calibration time for IMUs
float calibrateLen = 10;

// Reference Pressure for every bladder
volatile float Ref = 10000; ///////////////////////////////////////////////////// Reference Pressure
float a = 20; // Gain for volumetric control

float Kp = 5E-4; // Valve System Gains
float Ki = 5E-4;
float Kd = 0;
float deadBand = 150; // deadBand in Pa (do nothing if error is less than this number)
float Kpv = 25;
float Kiv = 25;
float Kdv = 0;
float deadBand_v = 0.05;
float DC_res = 5; // resolution of pwm duty cycle in %

float Kp_pump = 0.8E-4; // pump system gains
float Ki_pump = 1E-4;
float Kd_pump = 0;

volatile float R[8] = {0}; // vector of reference pressures
volatile float E[10] = {0}; // Pressure error array for bladders and source pressures
volatile float E_prev[10] = {0}; // Previous Pressure Error
volatile float E_dot[10] = {0}; // Derivative of Pressure Error
volatile float E_int[10] = {0}; // Integral of Pressure Error
volatile float U[16] = {0}; // Valve and Pump Command vector

volatile float E_v[6] = {0}; // Vector of adjusted pressure values
volatile float E_v_int[6] = {0}; //Inegral of volume error
volatile float E_v_dot[6] = {0}; // Derivative of colume error
volatile float E_v_prev[6] = {0};
volatile float U_v[12] = {0}; // Vector for volume control commands

float W[2] = {1, 0}; // Weights for combining volume and pressure control

float f1 = 25; // frequency of sensor acquisition timer
float f2 = 10; // frequency of valve control timer
float f_valve = 5; // frequency of valve pwm

float G_thresh = 27.5;
float M_thresh = 0.0003;
float sum_G[100] = {0};
float sum_M_dot[100] = {0};
float avg_G = 0;
float avg_M_dot = 0;

bool start = true; // start program flag
volatile uint8_t pinNumber = 0;
volatile uint8_t startCount = 0;
volatile uint8_t teleoperateFlag = 0;
volatile uint8_t stopCount = 0; // toggle flag
bool newData = false; // flag for computing new errors and commands
int waitLen = 5E5; // how long to wait for button debouncing/noise
volatile int Count = 0; // counter for when to print the Temperatures
volatile float Time = 0;
volatile bool detachFlag = false;
volatile int windowTime = 0; // current index in window
int n = 1; // window length for activity classification in seconds
volatile uint8_t classifyFlag = 0; // only classify if flag = true
volatile float haltTime = 0; // flag for halting classification
volatile float pauseClassify = 4; // Time (s) to pause classifier for settling
volatile uint8_t curActivity = 0; // keep track of previous activities
volatile uint8_t prevActivity = 0; // keep track of previous activities
/*
0 - Minimal
1 - Quasi-Static
2 - Dynamic
*/

const int chipSelect = BUILTIN_SDCARD; // SD Card pin

const char filename[] = "Temp_Control.txt"; /////////////////////////////// File Name to create on SD Card


// Declare external objects that exist in the main ??? Does this Work ???
extern ADS1015 adc1;
extern ADS1015 adc2;
extern ADS1015 adc3;
extern ADS1015 adc4;; // Thermistors 1-4

extern LSM6DS3Sensor IMU1;
extern LSM6DS3Sensor IMU2;

extern IntervalTimer TIMER_1;
extern IntervalTimer TIMER_2;

extern File sdCard;

// Functions
void Initialize(void)
{
	// Start serial line with baudrate of 38400 bps
	Serial.begin(38400);
	delay(10);
	if (CrashReport) // Print Crash Report if needed
	{
		Serial.println("\n" __FILE__ " " __DATE__ " " __TIME__); // File Information
		Serial.println(CrashReport);
	}

	// Set timer priorities
	TIMER_1.priority(0); // runs ADC
	TIMER_2.priority(1); // Control timer

	// I2C master mode pins 17 (SDA) and 16 (SCL) for ADCs and IMUs
	Wire.begin();
	//Wire.setClock(400000UL);

	// Start the ADCs
	adc1.begin();
	adc2.begin(); 
	adc3.begin(); 
	adc4.begin();
	adc1.setWireClock(1000000);
	adc2.setWireClock(1000000);
	adc3.setWireClock(1000000);
	adc4.setWireClock(1000000);
	adc1.setDataRate(6); // fast data rate
	adc2.setDataRate(6);
	adc3.setDataRate(6);
	adc4.setDataRate(6);
	adc1.setMode(1); // Single shot mode
	adc2.setMode(1);
	adc3.setMode(1);
	adc4.setMode(1);
	adc1.setGain(0); // turn off amplification
	adc2.setGain(0);
	adc3.setGain(0);
	adc4.setGain(0);
	// Start the IMUs
	IMU1.begin(); 
	IMU1.Set_X_FS(LSM6DS3_ACC_SENSITIVITY_FOR_FS_4G);
	delay(10);
	IMU1.Set_G_FS(LSM6DS3_GYRO_SENSITIVITY_FOR_FS_500DPS);
	delay(10);
	IMU1.Enable_X(); // Enable the accelerometer
	IMU1.Enable_G(); // Enable the Gyroscope
	IMU2.begin();
	IMU2.Set_X_FS(LSM6DS3_ACC_SENSITIVITY_FOR_FS_4G);
	delay(10);
	IMU2.Set_G_FS(LSM6DS3_GYRO_SENSITIVITY_FOR_FS_500DPS);
	delay(10);
	IMU2.Enable_X();
	IMU2.Enable_G();

	// setup PWM for bladder valves (Intake and Deflate valves)
	analogWriteFrequency(I1, f_valve); // frequency > 3 Hz
	analogWriteFrequency(D1, f_valve);
	analogWriteFrequency(I2, f_valve);
	analogWriteFrequency(D2, f_valve);
	analogWriteFrequency(I3, f_valve);
	analogWriteFrequency(D3, f_valve);
	analogWriteFrequency(I4, f_valve);
	analogWriteFrequency(D4, f_valve);
	analogWriteFrequency(I5, f_valve);
	analogWriteFrequency(D5, f_valve);
	analogWriteFrequency(I6, f_valve);
	analogWriteFrequency(D6, f_valve);
	// setup PWM for pumps
	analogWriteFrequency(P1, 100); // pump pin, f = 100 Hz
	// setup digital outputs for valves and pumps
	pinMode(V1, OUTPUT); 
	pinMode(V2, OUTPUT);
	pinMode(V3, OUTPUT);
	pinMode(P2, OUTPUT);
	// setup digital pins for debugging
	pinMode(GPIO1, OUTPUT);
	pinMode(GPIO2, OUTPUT);
	pinMode(GPIO3, OUTPUT);
	// setup digital output pins for LEDs, buzzer, and vibration motor
	pinMode(LEDR, OUTPUT);
	pinMode(LEDG, OUTPUT);
	pinMode(LEDW, OUTPUT);
	pinMode(VIBR, OUTPUT);
	analogWriteFrequency(BUZZ, 1000); // 1 kHz for buzzer
	// initialize all pins to LOW
	analogWrite(I1|D1|I2|D2|I3|D3|I4|D4|I5|D5|I6|D6|P1|P2|BUZZ, LOW);
	digitalWrite(V1|V2|V3|P2|GPIO1|GPIO2|GPIO3|LEDR|LEDG|LEDW|VIBR, LOW);  
	// Set up the GPIO interrupts for the four buttons
	pinMode(S1, INPUT);
	attachInterrupt(S1, GPIO_1_ISR, HIGH);
	pinMode(S2, INPUT);
	attachInterrupt(S2, GPIO_2_ISR, HIGH); 
	pinMode(S3, INPUT);
	attachInterrupt(S3, GPIO_3_ISR, HIGH); 
	pinMode(S4, INPUT);
	attachInterrupt(S4, GPIO_4_ISR, HIGH);

	// Setup the micro SD Card
	if(!SD.begin(chipSelect)){ // initialize SC card
		Serial.println("SD Card Ititialization Failed");
	} else
	{
		sdCard = SD.open(filename, FILE_WRITE); // open file on sd card
		delay(100);
		// Add column titles
		if (sdCard)
		{
			// Write Headers
			sdCard.print("Messages\t");
			sdCard.print("Time\t");
			sdCard.print("Acc_X\t");
			sdCard.print("Acc_Y\t");
			sdCard.print("Acc_Z\t");
			sdCard.print("Gyro_X\t");
			sdCard.print("Gyro_Y\t");
			sdCard.print("Gyro_Z\t");
			sdCard.print("Error_1\t");
			sdCard.print("Error_2\t");
			sdCard.print("Error_3\t");
			sdCard.print("Error_4\t");
			sdCard.print("Error_5\t");
			sdCard.print("Error_6\t");
			sdCard.print("Mass_1\t");
			sdCard.print("Mass_2\t");
			sdCard.print("Mass_3\t");
			sdCard.print("Mass_4\t");
			sdCard.print("Mass_5\t");
			sdCard.print("Mass_6\t");
			sdCard.println("Temperature");
			// Add in the starting desired pressure;
			sdCard.print("Starting Pressure: ");
			sdCard.println(Ref);
			sdCard.close(); // close file on sd card
		} else
		{
			Serial.println("Cannot Write Data");
		}
	}
	delay(100); // pause a little
}

void Start(void)
{
	// Set the desired pressures
    int ii = 0;
    for (ii=0; ii<N; ii++){
        R[ii] = Ref;
    }
    R[6] = Ref + 6800; // set accumulator to 1 psi higher than bladders
	
	// LEDs and stuff
	digitalWrite(LEDR, LOW);
	digitalWrite(VIBR, LOW);
	analogWrite(BUZZ, 0);
	
	// Begin the timers	
	TIMER_1.begin(TIMER_1_ISR, (int) round((1./f1)*pow(10, 6))); 
	TIMER_2.begin(TIMER_2_ISR, (int) round((1./f2)*pow(10, 6)));
}

void Idle(void)
{
	// pause timers
	TIMER_1.end();
	TIMER_2.end();
	// Print to Console/UART
	//Serial.println("System is Idle...");
	// Set main valves LOW
	analogWrite(I1, 0);
	analogWrite(D1, 0);
	analogWrite(I2, 0);
	analogWrite(D2, 0);
	analogWrite(I3, 0);
	analogWrite(D3, 0);
	analogWrite(I4, 0);
	analogWrite(D4, 0);
	analogWrite(I5, 0);
	analogWrite(D5, 0);
	analogWrite(I6, 0);
	analogWrite(D6, 0);
	// Turn off Pump
	analogWrite(P1, 0);
	digitalWrite(P2, LOW);
	// Set Seconday Valves LOW
	digitalWrite(V1, LOW);
	digitalWrite(V2, LOW);
	digitalWrite(V3, LOW);
	// LEDs and stuff
	digitalWrite(LEDR, LOW);
	digitalWrite(LEDG, LOW);
	digitalWrite(LEDW, LOW);
	digitalWrite(VIBR, LOW);
	analogWrite(BUZZ, 0);
}

void EStop(void)
{
	// pause control update timer
	TIMER_1.end();
	TIMER_2.end();
	// Print to Console/UART
	//Serial.println("Pulling Vaccuum...");
	// Vent Everything
	analogWrite(I1, 255);
	analogWrite(D1, 255);
	analogWrite(I2, 255);
	analogWrite(D2, 255);
	analogWrite(I3, 255);
	analogWrite(D3, 255);
	analogWrite(I4, 255);
	analogWrite(D4, 255);
	analogWrite(I5, 255);
	analogWrite(D5, 255);
	analogWrite(I6, 255);
	analogWrite(D6, 255);
	// Pull vaccuum on SPSL
	analogWrite(P1, 255);
	digitalWrite(P2, LOW);
	// Connect vaccum pump port to SPSL using secondary valves
	digitalWrite(V1, HIGH);
	digitalWrite(V2, HIGH);
	digitalWrite(V3, LOW);
	// LEDs and stuff
	digitalWrite(LEDR, HIGH);
	digitalWrite(LEDG, LOW);
	digitalWrite(LEDW, LOW);
	digitalWrite(VIBR, LOW);
	analogWrite(BUZZ, 0);
}

void Vent(void)
{
	// pause control update timer
	TIMER_1.end();
	TIMER_2.end();
	// Print to Console/UART
	//Serial.println("Venting...");
	// Vent Everything
	analogWrite(I1, 0);
	analogWrite(D1, 255);
	analogWrite(I2, 0);
	analogWrite(D2, 255);
	analogWrite(I3, 0);
	analogWrite(D3, 255);
	analogWrite(I4, 0);
	analogWrite(D4, 255);
	analogWrite(I5, 0);
	analogWrite(D5, 255);
	analogWrite(I6, 0);
	analogWrite(D6, 255);
	// Turn off pumps
	analogWrite(P1, 0);
	digitalWrite(P2, LOW);
	// Vent secondary valves
	digitalWrite(V1, HIGH);
	digitalWrite(V2, LOW);
	digitalWrite(V3, HIGH);
	// LEDs and stuff
	digitalWrite(LEDR, HIGH);
	digitalWrite(LEDG, LOW);
	digitalWrite(LEDW, HIGH);
	digitalWrite(VIBR, LOW);
	analogWrite(BUZZ, 0);
}

void ControlUpdate(void) // (computation time < 1 ms)
{

	int ii = 0;    
	// Update Pressure Error Terms
	for (ii=0; ii<6; ii++)
	{
		E[ii] = R[ii]-A[ii];
		E_dot[ii] = (E[ii] - E_prev[ii])*f1;
		E_int[ii] = E_int[ii] + E[ii]/f1;
		if (sgn(E[ii]) != sgn(E_prev[ii]))
		{ // Integral Zero Error Cross Over Reset
			E_int[ii] = 0;
		}
		E_prev[ii] = E[ii];

		// Update Bladder Valve Commands
		U[ii] = Kp*E[ii] + Kd*E_dot[ii] + Ki*E_int[ii];
		// Add a deadband
		if (abs(E[ii]) <= deadBand)
		{
			U[ii] = 0;
		}
	}

	// Determine the Approximate Mass in Each Bladder in L
	float U_total_in = 0;
	float U_total_out = 0;
	for (ii=0; ii<N; ii++) // find total duty cycles for all bladder valves
	{
		U_total_in = U_total_in + U[ii];
		U_total_out = U_total_out + U[ii+6];
	}
	// Don't divide by zero
	if (U_total_in == 0)
	{U_total_in = 1;}
	if (U_total_out == 0)
	{U_total_out = 1;}
		
	// numerically integrate weighted mass flowrates
	for (ii=0; ii<N; ii++)
	{
		M_dot[ii] = (A[10]*U[ii]/U_total_in - A[11]*U[ii+6]/U_total_out)/f1;
		V[ii] = V[ii] + M_dot[ii]; 
	}

	// Update Air Mass Error, is the sign correct?
	//E_v[0] = V[0] - (V[1]+V[2])/2;
	//E_v[1] = V[1] - (V[0]+V[2])/2;
	//E_v[2] = V[2] - (V[0]+V[1])/2;
	//E_v[3] = V[3] - (V[4]+V[5])/2;
	//E_v[4] = V[4] - (V[3]+V[5])/2;
	//E_v[5] = V[5] - (V[3]+V[4])/2;
	
	E_v[0] = (V[1]+V[2])/2 - V[0];
	E_v[1] = (V[0]+V[2])/2 - V[1];
	E_v[2] = (V[0]+V[1])/2 - V[2];
	E_v[3] = (V[4]+V[5])/2 - V[3];
	E_v[4] = (V[3]+V[5])/2 - V[4];
	E_v[5] = (V[3]+V[4])/2 - V[5];
	for (ii=0; ii<N; ii++)
	{
		// Implement mass control
		E_v_dot[ii] = (E_v[ii] - E_v_prev[ii])*f1;
		E_v_int[ii] = E_v_int[ii] + E_v[ii]/f1;
		if (sgn(E_v[ii]) != sgn(E_v_prev[ii]))
		{ // Integral Zero Error Cross Over Reset
			E_v_int[ii] = 0;
		}
		E_v_prev[ii] = E_v[ii];
		
		U_v[ii] = Kpv*E_v[ii] + Kiv*E_v_int[ii] + Kdv*E_v_dot[ii];
		// Add deadband
		if (abs(E_v[ii]) <= deadBand_v)
		{
			U_v[ii] = 0;
		}
	}

	// Combine Mass and Pressure Commands with Weights
	for (ii=0; ii<N; ii++) // can be inlfating and deflating a single bladder ************************************
	{
		U[ii] = W[0]*U[ii] + W[1]*U_v[ii];
		U[ii] = floor(100/DC_res*U[ii])/(100/DC_res);
		// Saturation Limits
		if (U[ii] > 1)
		{
			U[ii] = 1;
		} else if(U[ii] < -1)
		{
			U[ii] = -1;
		}
		// Check sign (+ -> inflate & - -> deflate))
		if (U[ii] < 0)
		{ // change to deflate valve
			U[ii+6] = -U[ii];
			U[ii] = 0;
		} else
		{
			U[ii+6] = 0;
		}
	}
	
	// Do Activity Calssification
	// Update features
	sum_M_dot[windowTime] = abs(M_dot[0]) + abs(M_dot[1]) + abs(M_dot[2]) + abs(M_dot[3]) + 
		abs(M_dot[4]) + abs(M_dot[5]); // sum mass flow rates
	sum_G[windowTime] = sqrt(imu1[3]*imu1[3] + imu1[4]*imu1[4] + imu1[5]*imu1[5]); // magnitude of gyroscope measurements
	windowTime++;
	
	if (classifyFlag == 1) // only classify when flags true
	{
		// Average the sums of the mass flow rates and gyroscope measurements
		for (ii=0; ii<windowTime-1; ii++)
		{
			avg_M_dot = avg_M_dot + sum_M_dot[ii];
			avg_G = avg_G + sum_G[ii];
		}
		avg_M_dot = avg_M_dot/(windowTime-1);
		avg_G = avg_G/(windowTime-1);
		// wait for SPSL to inflate for 10 s - assume minimal activity
		if (Time < 10)
		{
			// Minimal Activity
			Ref = 10000; // Update desired pressure
			W[0] = 1.0; // Update mass control weights
			W[1] = 0.0;
			curActivity = 0;
		} else
		{
			//Decision Tree Classification
			if (avg_G > G_thresh)
			{
				// Dynamic Activity
				Ref = 14000; // Update desired pressure
				W[0] = 0.75; // Update mass control weights
				W[1] = 0.25;
				//Serial.println("Dynamic Activity\t");
				curActivity = 2;
			} else
			{
				if (avg_M_dot < M_thresh)
				{
					// Minimal Activity
					Ref = 10000; // Update desired pressure
					W[0] = 1.0; // Update mass control weights
					W[1] = 0.0;
					//Serial.println("Minimal Activity\t");
					curActivity = 0;
				} else
				{
					// Quasi-Static Activity
					Ref = 14000; // Update desired pressure
					W[0] = 1.0; // Update mass control weights
					W[1] = 0.0;
					//Serial.println("Quasi-Static Activity\t");
					curActivity = 1;
				}
			}
			if (curActivity != prevActivity) // activity change detected
			{
				haltTime = Time;
			}
			prevActivity = curActivity;
			//Serial.print(U[0]);
			//Serial.print("\t");
			//Serial.print(U[6]);
			//Serial.print("\t");
			//Serial.print(E[0]);
			//Serial.print("\t");
			//Serial.println(E_v[0]);
		}
		windowTime = 0; // reset feature index
		for (ii=0; ii<N; ii++) // update desired pressures
		{
			R[ii] = Ref;
		}
		R[6] = Ref + 6800; // set accumulator to 1 psi higher than bladders
		classifyFlag = 0;
		// reset previous activity array everytime activity changes
	}
	
	// Accumulator Controller
	E[6] = R[6]-A[6];
	E_dot[6] = (E[6] - E_prev[6])*f1;
	E_int[6] = E_int[6] + E[6]/f1;
	if (sgn(E[6]) != sgn(E_prev[6]))
	{ // Integral Zero Error Cross Over Reset
		E_int[6] = 0;
	}
	E_prev[6] = E[6];
	// Implement Accumulator Valve and Pump Commands
	U[12] = Kp_pump*E[6] + Kd_pump*E_dot[6] + Ki_pump*E_int[6];
	U[13] = LOW; // Water Pump Command
	U[14] = LOW;
	if (U[12] > 1)
	{
		U[12] = 1;
		U[14] = LOW;
	} else if(U[12] < .2)
	{
		U[12] = 0;
		if (E[6] < -1000) // Vent iff pressure is too high
		{
			U[14] = HIGH;
		}
	}
	U[15] = LOW; // bladder outputs to ATM
	
	// Compile IMU Data
	for (ii=0; ii<3;ii++)
	{
		//Subtract zero values and store in imu array
		imu1[ii] = ((float)Acc1[ii] - (float)Acc1Zero[ii])/1000;
		imu1[ii+3] = ((float)Gyro1[ii] - (float)Gyro1Zero[ii])/1000;
		imu2[ii] = ((float)Acc2[ii] - (float)Acc2Zero[ii])/1000;
		imu2[ii+3] = ((float)Gyro2[ii] - (float)Gyro2Zero[ii])/1000;
		// Average IMU data together into IMU1
		imu1[ii] = (imu1[ii]+imu2[ii])/2;
		imu1[ii+3] =(imu1[ii+3]+imu2[ii+3])/2;
	}
}

void TIMER_1_ISR(void) // sensing timer (computation time < 8 ms)
{
	
	// Measure Pressures
	int ii;
	for (ii=0; ii<12; ii++){
		if (ii < 4)
		{
			A[ii] = ((float)adc1.readADC(ii) - C[ii])*B; // pressure seonsors 1-4
		} else if (ii < 8)
		{
			A[ii] = ((float)adc2.readADC(ii-4) - C[ii])*B; // pressure sensors 5-8
		} else if (ii < 10)
		{
			A[ii] = ((float)adc3.readADC(ii-8) - C[ii])*B; // pressure sensors 9 and 10
		} else
		{
			A[ii] = (adc3.toVoltage(adc3.readADC(ii-8)) - C[ii]); // flow rate sensors 11 and 12
			A[ii] = D*A[ii]*A[ii]; // Now in g/s flow 
			if (A[ii] < 0)
			{
				A[ii] = 0;
			}
		}
	}

	// Store IMU values
	IMU1.Get_X_Axes(Acc1);
	IMU1.Get_G_Axes(Gyro1);
	IMU2.Get_X_Axes(Acc2);
	IMU2.Get_G_Axes(Gyro2);
	
	Time = Time + 1/f1;
	newData = true;
	
}

void TIMER_2_ISR(void) // control timer (computation time < 2 ms)
{
	float curTime = Time;
	// Update main valves
	analogWrite(I1, U[0]*255);
	analogWrite(D1, U[6]*255);
	analogWrite(I2, U[1]*255);
	analogWrite(D2, U[7]*255);
	analogWrite(I3, U[2]*255);
	analogWrite(D3, U[8]*255);
	analogWrite(I4, U[3]*255);
	analogWrite(D4, U[9]*255);
	analogWrite(I5, U[4]*255);
	analogWrite(D5, U[10]*255);
	analogWrite(I6, U[5]*255);
	analogWrite(D6, U[11]*255);
	// Update pump pwms
	analogWrite(P1, U[12]*255);
	digitalWrite(P2, U[13]);
	// Update secondary valves
	digitalWrite(V1, U[14]);
	digitalWrite(V2, U[15]);
	digitalWrite(V3, HIGH); // Connect Pump Intake to ATM	
	
	int ii;
	// Record the Termpertures @ 1 Hz
	if (Count % 10 == 0)
	{
		// pressure-duration warning
		if (Time > 900) // sound buzzer if wear time exceeds this (15 mins)
		{
			Vent();
			analogWrite(BUZZ, 127);
			delay(10000);
			Idle();
			stopCount = 2;
			
		}
		for (ii = 0; ii <4; ii++)
		{
			T[ii] = (float)adc4.toVoltage(adc4.readADC(ii)); // Read Thermistors 1-4
			//T[ii] = p1*pow(T[ii], 3) + p2*pow(T[ii], 2) + p3*T[ii] + p4; // Convert Voltage to T
		}		
		Temp = (T[0] + T[1] + T[2] + T[3])*0.25;
		Temp = p1*pow(Temp, 3) + p2*pow(Temp, 2) + p3*Temp + p4; // Convert Voltage to T
	}
	
	// reset count so it doesnt get too big
	if (Count%(n*(int)f2+1) == 0)
	{
		if (teleoperateFlag == 0) // no activity classification if flag
		{
			classifyFlag = 0;
			windowTime = 0;
		} else
		{
			if (curTime - haltTime <= pauseClassify)
			{
				classifyFlag = 0; // pause classifier
				windowTime = 0; // reset window index to overwrite classification features
			} else 
			{
				classifyFlag = 1; // resume classifier
			}
		}
		Count = 0; // reset count so it doesnt roll over;
	}
	Count++;
}

void GPIO_1_ISR(void) // pin 33 (stop control)
{
	detachFlag = true; // toggle flag
	pinNumber = 1;
	
	// detach interrupts from switches (debouncing)
	//detachInterrupt(S1);
	//detachInterrupt(S2); 
	//detachInterrupt(S3); 
	//detachInterrupt(S4);
	
	//volatile bool switchState = digitalRead(S1); // read switch state
	//while(switchState){switchState = digitalRead(S1);} // wait until button release
}

void GPIO_2_ISR(void) // pin 34 (start control)
{
	detachFlag = true;
	pinNumber = 2;
	
	// detach interrupts from switches (debouncing)
	//detachInterrupt(S1);
	//detachInterrupt(S2); 
	//detachInterrupt(S3); 
	//detachInterrupt(S4);
	
	//volatile bool switchState = digitalRead(S2);
	//while(switchState){switchState = digitalRead(S2);}
}

void GPIO_3_ISR(void) // S3 (decrease pressure)
{
	detachFlag = true;
	pinNumber = 3;
	
	// detach interrupts from switches (debouncing)
	//detachInterrupt(S1);
	//detachInterrupt(S2); 
	//detachInterrupt(S3); 
	//detachInterrupt(S4);
	
	//volatile bool switchState = digitalRead(S3); // recheck the button
	//while(switchState){switchState = digitalRead(S3);} // wait for button release
	//VibrateMotor(500);
}

void GPIO_4_ISR(void) // S4 (increase pressure)
{
	detachFlag = true;
	pinNumber = 4;
	
	// detach interrupts from switches (debouncing)
	//detachInterrupt(S1);
	//detachInterrupt(S2); 
	//detachInterrupt(S3); 
	//detachInterrupt(S4);

	//volatile bool switchState = digitalRead(S4); // recheck the button
	//while(switchState){switchState = digitalRead(S4);} // wait for button release
}

void GPIOHandler(void)
{
	int ii = 0;
	switch(pinNumber)
	{
		case 1:
			//digitalWrite(GPIO2, HIGH);		
			
			// pause control update timer
			TIMER_2.end();
			TIMER_1.end();
			startCount = 0;	
			StoreData(1);
			
			if (stopCount == 0) // Iterate through functionalities
			{
				EStop(); // pull vaccuum on bladders
				stopCount++;
			} else if (stopCount == 1)
			{
				Vent(); // open all valves
				stopCount++;
			} else
			{
				Idle(); // shut off all valves and pumps
				stopCount = 0;
			}
				
			//digitalWrite(GPIO2, LOW);
			break;
		case 2:
			//digitalWrite(GPIO1, HIGH);
			
			stopCount = 0; // reset S1 State
			StoreData(2);

			if (startCount == 0) // switch control teleoperation states
			{
				Time = 0; // reset time first time through
				startCount++; // switch state next time
				digitalWrite(LEDG, HIGH);
				digitalWrite(LEDW, LOW);
			} else if (startCount == 1)
			{
				digitalWrite(LEDG, HIGH);
				digitalWrite(LEDW, HIGH);
				teleoperateFlag = 1; // Classify
				//Serial.println("Yes");
				startCount++; // switch state next time
				
			} else
			{
				digitalWrite(LEDG, HIGH);
				digitalWrite(LEDW, LOW);
				teleoperateFlag = 0; // No Classifier
				//Serial.println("No");
				startCount = 1; // switch state next time
			}	
			
			Start();
			
			//digitalWrite(GPIO1, LOW);
			break;
		case 3:			
			Ref = Ref - 250; // increase desired pressure
			if (Ref < 3000) // saturation limit (3 kPa)
			{
				Ref = 3000;
			}
			// Update the desired pressures
			for (ii=0; ii<N; ii++)
			{
				R[ii] = Ref;
			}
			R[6] = Ref + 6800; // set accumulator to 1 psi higher than bladders
			
			StoreData(3);
			
			//VibrateMotor(150);
			break;
		case 4:
			Ref = Ref + 250; // increase desired pressure
			if (Ref > 20000) // saturation limit (13.333 kPa)
			{
				Ref = 20000;
			}
			// Update the desired pressures
			for (ii=0; ii<N; ii++)
			{
				R[ii] = Ref;
			}
			R[6] = Ref + 6800; // set accumulator to 1 psi higher than bladders
			
			StoreData(3);
			
			//VibrateMotor(150);
			break;
		default: 
			Serial.println("Button Switch Case Error");
			break;
	}
}

void Calibrate(void)
{
	int ii=0;
	while (calibrateTime < calibrateLen)
	{
		//IMU1.Get_X_Axes(Acc1);
		IMU1.Get_G_Axes(Gyro1);
		//IMU2.Get_X_Axes(Acc2);
		IMU2.Get_G_Axes(Gyro2);
		for (ii=0; ii<3;ii++)
		{
			// sum the measurements over period of time for calculating the mean
			//Acc1Zero[ii] = Acc1Zero[ii] + Acc1[ii]; 
			Gyro1Zero[ii] = Gyro1Zero[ii] + Gyro1[ii];
			//Acc2Zero[ii] = Acc2Zero[ii] + Acc2[ii];
			Gyro2Zero[ii] = Gyro2Zero[ii] + Gyro2[ii];
		}
		if (calibrateTime == calibrateLen-1)
		{
			for (ii=0; ii<3;ii++)
			{
				// find the mean measurement
				//Acc1Zero[ii] = (int32_t)round((float)Acc1Zero[ii]/calibrateLen);
				Gyro1Zero[ii] = (int32_t)round((float)Gyro1Zero[ii]/calibrateLen);
				//Acc2Zero[ii] = (int32_t)round((float)Acc2Zero[ii]/calibrateLen);
				Gyro2Zero[ii] = (int32_t)round((float)Gyro2Zero[ii]/calibrateLen);
			}
		}
		delay(100);
		calibrateTime++;
	}
}

void VibrateMotor(uint16_t t)
{
	// vibrate motor for specified amount of time
	digitalWrite(VIBR, HIGH);
	delay(t);
	digitalWrite(VIBR, LOW);
}

void StoreData(int message)
{
	sdCard = SD.open(filename, FILE_WRITE); // open file on sd card
	if (sdCard)
	{
		if (message == 0) // Store Data
		{
			// Store data to sd card
			int ii;
			sdCard.print("\t");
			sdCard.print(Time); // Write Time
			sdCard.print("\t");
			for (ii=0;ii<6;ii++) // Write IMU Data
			{
				sdCard.print(imu1[ii], 4);
				sdCard.print("\t");
			}
			for (ii=0;ii<6;ii++) // Write Pressure Data
			{
				sdCard.print(E[ii]);
				sdCard.print("\t");
			}
			for (ii=0;ii<6;ii++) // Write Mass Data
			{
				sdCard.print(M_dot[ii], 4);
				sdCard.print("\t");
			}
			sdCard.println(Temp);
		} else if (message == 1) // Stopping Control
		{
			if (stopCount == 0)
			{
				sdCard.println("EMERGENCY STOP");
			} else if (stopCount == 1)
			{
				sdCard.println("VENTING");
			} else
			{
				sdCard.println("IDLE");
			}			
		} else if (message == 2) // Starting Control
		{
			sdCard.println("STARTING CONTROL");
		} else if (message == 3) // Reference PRessure Change
		{
			sdCard.print("New Desired Pressure: ");
			sdCard.println(Ref, 0);
		} else
		{
			sdCard.println("ERROR");
		}
		sdCard.close(); // close file on sd card
	} else
	{
		Serial.println("Card failed");
	}
}