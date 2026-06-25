//! --- Motor ----
//? Left Motor (Treated as Motor A)
#define PWMR 26
#define RIN1 4
#define RIN2 25

//? Right Motor (Treated as Motor B)
#define LIN1 17
#define LIN2 5
#define PWML 18 

//? --- Standby Pin ---
#define STBY 16

//? --- Motor Offsets ---
// These are constants that help fix wiring issues. 
// If a motor spins backward when it should go forward, change its offset from 1 to -1.
const int offsetL = -1;
const int offsetR = 1;  

//!  --- Encoder Pins ---  
#define ENCL_A 19
#define ENCL_B 23
#define ENCR_A 2
#define ENCR_B 15

//! --- LED Pins ---

#define LEDL 12
#define LEDR 32

//! --- Button Pins ---

#define BTNL 14
#define BTNR 35

//! --- VL53L0X Sensor Pins and Addresses ---

//? Define XSHUT pins according to your setup
#define XSHUT_CENTER 27
#define XSHUT_LEFT   13
#define XSHUT_RIGHT  33

//? Define new I2C Addresses for the sensors 
// (The default is 0x29, so we assign them new ones)
#define ADDRESS_CENTER 0x30
#define ADDRESS_LEFT   0x31
#define ADDRESS_RIGHT  0x32

//! --- MPU6050 Sensor Pins and Addresses ---

//? Define new I2C Addresses for the MPU6050 Gyroscope/Accelerometer
#define MPU_ADDR 0x68