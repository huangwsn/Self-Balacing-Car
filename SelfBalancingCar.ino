/************************************************************************************
 * Filename:      
 *                SelfBalacing.ino
 * Description:
 *                This module contrains functions to perform self-balacing of a 
 *                2-wheel car using PID control. The parameters for the PID control 
 *                could be determined empirically or be found through physical modeling.
 * History
 * When           Who             What/Why
 * ----------------------------------------------------------------------------------
 * 2018/12/17     Winson Huang    Implemented PID control scheme in response to 
 *                                accelerometer readings. PID Parameters are to be 
 *                                determined.
 ***********************************************************************************/

 
/*--------------------------------- Include Files ---------------------------------*/
#include <string.h>
#include <Wire.h>
/*--------------------------------- Module Defines --------------------------------*/
#define ACCELEROMETER_MODULE (0x53)
#define XYZ_REGISTER (0x32)
#define LED_PIN 3
#define MOTOR_PIN_ONE 5
#define MOTOR_PIN_TWO 6
#define MAX_PWM_DUTY_CYCLE 255
#define acc_Z_REFERENCE 240
#define FALL_OVER_THRESOLD 180
#define Kp 15                         // Proportional gain (P) in PID control
#define Kd 0                          // Derivative gain (D) in PID control
#define Ki 0                          // Integral gain (I) in PID control
/*--------------------------------- Module Functions -------------------------------*/
void initializeAccelerometer(void);
void initializeMotor(void);
void update_acc_XYZ(int *acc_x_addr, int *acc_y_addr, int *acc_z_addr);
void print_acc_XYZ(int acc_x, int acc_y, int acc_z);
bool isStanding(int acc_y);
int getLEDPWM(int acc_Z);
void moveWheel(int acc_y, int acc_z);
/*--------------------------------- Module Variables -------------------------------*/
byte values[6];


/*----------------------------------- Module Code ----------------------------------*/


void setup() {
  initializeAccelerometer();
  initializeMotor();
  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 0);
}


void loop() {
  int acc_x, acc_y, acc_z;

  // Acquire acceleration readings
  update_acc_XYZ(&acc_x, &acc_y, &acc_z);
  print_acc_XYZ(acc_x, acc_y, acc_z);

  // Update PWM for visual debugging LED Indicator
  analogWrite(LED_PIN, getLEDPWM(acc_z));

  // Update PWM for wheel
  if(isStanding(acc_y)) {
    moveWheel(acc_y, acc_z);
  } else {
    stopWheel();
  }
  
}
/*-------------------------------- Helper Functions --------------------------------*/
/*************************************************************************************
 * Function 
 *              initializeAccelerometer
 * Parameters
 *              None
 * Returns
 *              None
 * Description
 *              This function initializes the required pins and setting for the accelerometer 
 *              (ADXL345) using I2C protocol.
 * Author
 *              Winson Huang
 *************************************************************************************/
void initializeAccelerometer(void) {
  Wire.begin();
  Serial.begin(9600);
  Wire.beginTransmission(ACCELEROMETER_MODULE);
  Wire.write(0x2D);
  Wire.write(16);
  Wire.endTransmission();
  Wire.beginTransmission(ACCELEROMETER_MODULE);
  Wire.write(0x2D);
  Wire.write(8);
  Wire.endTransmission();
  Serial.print("Accelerometer Initialization Done.\n");  
}



/*************************************************************************************
 * Function 
 *              initializeMotor
 * Parameters
 *              None
 * Returns
 *              None
 * Description
 *              This function initializes the pins for controlling a DC motor.
 * Author
 *              Winson Huang
 *************************************************************************************/
void initializeMotor(void) {
  pinMode(MOTOR_PIN_ONE, OUTPUT);
  pinMode(MOTOR_PIN_TWO, OUTPUT);
  analogWrite(MOTOR_PIN_ONE, 0);
  analogWrite(MOTOR_PIN_TWO, 0);
  Serial.print("Motor Initialization Done.\n");  
}



/*************************************************************************************
 * Function 
 *              update_acc_XYZ
 * Parameters
 *              int *acc_x_Addr: An integer pointer to acc_x for value updating
 *              int *acc_y_Addr: An integer pointer to acc_y for value updating
 *              int *acc_z_Addr: An integer pointer to acc_z for value updating
 * Returns
 *              None
 * Description
 *              This function updates the value of acc_x, acc_y, acc_y with the measurements 
 *              taken by the accelerometer (ADXL345). 
 * Author
 *              Winson Huang
 *************************************************************************************/
void update_acc_XYZ (int *acc_x_Addr, int *acc_y_Addr, int *acc_z_Addr) {
  Wire.beginTransmission(ACCELEROMETER_MODULE);
  Wire.write(XYZ_REGISTER);
  Wire.endTransmission();

  Wire.beginTransmission(ACCELEROMETER_MODULE);
  Wire.requestFrom(ACCELEROMETER_MODULE, 6); 
  
  int i = 0;
  while(Wire.available()) {
    values[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();

  *acc_x_Addr = ( ((int)values[1] ) << 8) | values[0];
  *acc_y_Addr = ( ((int)values[3] ) << 8) | values[2];
  *acc_z_Addr = ( ((int)values[5] ) << 8) | values[4];
}



/*************************************************************************************
 * Function 
 *              print_acc_XYZ
 * Parameters
 *              int acc_x: Acceleration measured along x direction
 *              int acc_y: Acceleration measured along y direction
 *              int acc_z: Acceleration measured along z direction
 * Returns
 *              None
 * Description
 *              This function prints the values of acc_x, acc_y, acc_z in an organized 
 *              fashion. 
 * Author
 *              Winson Huang
 *************************************************************************************/
void print_acc_XYZ(int acc_x, int acc_y, int acc_z) {
  Serial.print(acc_x, DEC);
  Serial.print(" ");
  Serial.print(acc_y, DEC);
  Serial.print(" ");
  Serial.print(acc_z, DEC);
  Serial.print("\n");
}



/*************************************************************************************
 * Function 
 *              isStanding
 * Parameters
 *              int acc_y: Acceleration measured along y direction
 * Returns
 *              bool      : True if car has not fallen over. (|acc_y| < 180)
 *                          False if car has fallen over. (|acc_y| > 180)
 * Description
 *              This function determines whether the car has fallen over based on acc_y 
 *              reading.
 * Author
 *              Winson Huang
 *************************************************************************************/
bool isStanding(int acc_y) {
  return ((acc_y <= FALL_OVER_THRESOLD) && (acc_y >= -FALL_OVER_THRESOLD));
}



/*************************************************************************************
 * Function 
 *              getLEDPWM
 * Parameters
 *              int acc_z: Acceleration measured along z direction
 * Returns
 *              Int      : The PWM duty cycle for the LED indicator
 * Description
 *              This function computes the corresponding PWM duty cycle for a given 
 *              acceleration measured along z direction.The PWM duty cycle reflects the 
 *              current acceleration measured along z direction.
 * Author
 *              Winson Huang
 *************************************************************************************/
int getLEDPWM(int acc_z) {
  int LED_PWM = acc_Z_REFERENCE - acc_z;
  if(LED_PWM < 0) {
    return 0;
  }
  else if(LED_PWM > MAX_PWM_DUTY_CYCLE) {
    return MAX_PWM_DUTY_CYCLE;
  }
  return LED_PWM;
}



/*************************************************************************************
 * Function 
 *              moveWheel
 * Parameters
 *              int acc_y: Acceleration measured along y direction
 *              int acc_z: Acceleration measured along z direction
 * Returns
 *              None
 * Description
 *              This function uses PID control scheme to compute the required motor PWM 
 *              and its spinning direction to balacne the inverted pendulum.
 * Author
 *              Winson Huang
 *************************************************************************************/
void moveWheel(int acc_y, int acc_z) {
  int Wheel_PWM = getWheelPWM(acc_y, acc_z);
  
  if(acc_y < 0) {
    analogWrite(MOTOR_PIN_ONE, Wheel_PWM);
    analogWrite(MOTOR_PIN_TWO, 0);    
  } else {
    analogWrite(MOTOR_PIN_ONE, 0);
    analogWrite(MOTOR_PIN_TWO, Wheel_PWM);    
  }
}



/*************************************************************************************
 * Function 
 *              getWheelPWM
 * Parameters
 *              int acc_y: Acceleration measured along y direction
 *              int acc_z: Acceleration measured along z direction
 * Returns
 *              int      : PWM duty cycle for motor (between 0 and 255)
 * Description
 *              This function uses PID control scheme to compute the required motor PWM.
 * Author
 *              Winson Huang
 *************************************************************************************/
int getWheelPWM(int acc_y, int acc_z){
  int wheel_PWM = 0;
  static int last_acc_z_Error = 0; 
  int current_acc_z_Error = acc_Z_REFERENCE - acc_z;
  int p_Term = current_acc_z_Error;
  int d_Term = last_acc_z_Error - current_acc_z_Error; 
  static int i_Term = 0;

  i_Term = i_Term + current_acc_z_Error;
  wheel_PWM = (Kp * p_Term) - (Kd * d_Term) + (Ki * i_Term);

  if(wheel_PWM > MAX_PWM_DUTY_CYCLE) {
    wheel_PWM = MAX_PWM_DUTY_CYCLE;
  }
  
  last_acc_z_Error = current_acc_z_Error;

  return wheel_PWM;  
}



/*************************************************************************************
 * Function 
 *              stopWheel
 * Parameters
 *              None
 * Returns
 *              None
 * Description
 *              This function stops the motor by applying 0 voltage to both ends of the 
 *              motor leads.
 * Author
 *              Winson Huang
 *************************************************************************************/
void stopWheel(void) {
  analogWrite(MOTOR_PIN_ONE, 0);
  analogWrite(MOTOR_PIN_TWO, 0); 
}
