# Smart-Prosthesis-Socket-Liner
The Smart Prosthesis Socket Liner (SPSL) was a prototype wearable interfacing system. This system used a multitude of pneumatic solenoid valves to control dependent soft actuators inside a prosthesis socket. Additionally, a low-pressure hydraulic loop was used to remove heat from a user's residual limb. A real-time time-domain acitivtiy classifier was used to autonomously adapt the interface stiffness to suit the catgory of tasks the user was performing.

This repository contains:

SPSL_Controller_V2.ino - The main controller program that runs on the teensy 4.1 at 600 MHz

SPSL.h and SPSL.c - The declaration of the global and local variables, objects, and functions needed to run the main controller program:

--- Initialize() - Sets up the pins needed on the Teensy for DIO, I2C, PWM, etc. and starts communication protocols (UART for debugging and I2C for sensor communications).

--- Start() - Starts the two timer threads and sets the hardcoded desired pressures for the six soft-pneumatic actuators.

--- Idle() - Stops both timers (kills control updates) and closes all of the valves (holds the current controller states).

--- EStop() - Stops both timers (kills control updates), pulls a vaccum on the sink grounding all ot the soft-pneumatic actuators, and opens the vent valves for all of the actuators (sucks the air out of the SPSL).

--- Vent() - Stops both timers (kills control updates), turns off the compressor, and opens the vent valves for all of the actuators (venting passively to ATM).

--- ControlUpdate() - Computes the current control outputs based on previous contoller states. PI control with zero-error cross over reset for intergral windup prevention. Gathers and computes the actiivty category the user is performing based on previous IMU information and controller states. The activity category performes gain scheduling and desired pressures to adjust the fit of the SPSL.

--- TIMER_1_ISR() - Interrupt service routing for the first timer thread. Measures the static air pressure of the six spft-pneumatic actuators and the accumulator, the mass flow rate sensors, and the 6-DOF IMUs. sets flag to indicate that new data has been saved in global variables.

--- TIMER_2_ISR() - Interrupt service routing for the second timer thread. Commands the valves and pumps to the previously computed commands. MEasures the four thermistors and averages their temperature of the SPSL-human interface. Checks to see if the user has been wearaing the device for too long, sounds a buzzer if pressure exposure has reached unsafe levels. sets a flag every second for the activity classifier in ControlUpdate() to perform activity classification.

--- GPIO_1_ISR()-GPIO_4_ISR() -  Interrupt service routines for the four push buttons on the shell of the SPSL. to avoid switch bouncing the interrupts are disables for a small amount of time after a button has been pressed.

--- GPIOHandler() - Determines which button has been pressed after switch debouncing. 1: cycles through Idle, Vent, and EStop functions. 2: cycles through the start function toggling hte activitiy classifier. 3: decreases the desired pressures of the soft-pneumatic actuators. 4: increases the desired pressures of the soft-pneumatic actuators.

--- Calibrate() - Calibrates the gyros for the IMUs by zeroing out the gyro.

--- VibrateMotor(uint16_t t) - Turns on the minature vibrating motor for the specified duration.

--- StoreData(int message) - Stores the specified data onto the micro SD card inserted in the Teensy 4.1 for posthoc data recovery.
