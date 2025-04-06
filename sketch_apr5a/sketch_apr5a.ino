#include <Arduino.h>
#include "nelito.h"

#define CONTROL_PERIOD 1000 //ms
#define LOW 0 //GND
#define HIGH 3.3 //VCC

#define LEFT_MOTOR_M1_PIN
#define LEFT_MOTOR_M2_PIN
#define LEFT_MOTOR_ENC_PIN_A 5 // Channel A (CW)
//#define LEFT_MOTOR_ENC_PIN_B 13 // Channel B (CW)

#define RIGHT_MOTOR_M1_PIN
#define RIGHT_MOTOR_M2_PIN
#define RIGHT_MOTOR_ENC_PIN_A 21 // Channel A (CW)
//#define RIGHT_MOTOR_ENC_PIN_B 12 // Channel B (CW)

#define IR_RECEIVER_PIN 6

#define LINE_SENSOR_PWM_PIN 7
#define LINE_SENSOR_1_PIN 16
#define LINE_SENSOR_2_PIN 17
#define LINE_SENSOR_3_PIN 18
#define LINE_SENSOR_4_PIN 19
#define LINE_SENSOR_5_PIN 20

#define MAX_LIN_VEL 1
#define MAX__ANG_VEL 0
#define MAX_LIN_ACCEL 100
#define MAX_ANG_ACCEL 100

#define EXTREME_LINE_ERROR 5.0
#define SMALL_LINE_ERROR 2.0

// Control variables
unsigned long lastTime = 0;
unsigned long currentTime = 0;

double leftMotorSpeed = 1, rightMotorSpeed = 1;
double sensor1_black, sensor2_black, sensor3_black, sensor4_black, sensor5_black, num_blacks;
double lineSensorError;
double last_lineSensorError = 0.0;
double accum_lineSensorError = 0.0;
bool lost = false;

double robot_v = 0, robot_w = 0;
double last_robot_v = 0, last_robot_w = 0;
double Kp = 10.0;
double Kd = 0.0;
double Ki = 0.0;


//! Controlo de velocidade
double convert_velocity_to_speed(double v, double w){
  
  return 0.0;
}

void limitAcceleration(){
  if(last_robot_v - robot_v < -MAX_LIN_ACCEL){
    robot_v = last_robot_v - MAX_LIN_ACCEL;
    return;
  }

  if(last_robot_v - robot_v > MAX_LIN_ACCEL){
    robot_v = last_robot_v + MAX_LIN_ACCEL;
    return;
  }

  if(last_robot_w - robot_w < -MAX_ANG_ACCEL){
    robot_w = last_robot_w - MAX_ANG_ACCEL;
    return;
  }

  if(last_robot_w - robot_w > MAX_ANG_ACCEL){
    robot_w = last_robot_w + MAX_ANG_ACCEL;
    return;
  }
}

void convertSpeedToPWM(int speed) {
}

//! PID
void PID(){
  last_robot_v = robot_v;
  last_robot_w = robot_w;

  robot_v = MAX_LIN_VEL * (1.0 - abs(lineSensorError) / EXTREME_LINE_ERROR);
  robot_w = Kp * lineSensorError;  
}


//! Leitura da linha
//# 1 = Branco
//# 0 = Preto
void readLineSensor(){
  sensor1_black = 1.0 - (double)digitalRead(LINE_SENSOR_1_PIN);
  sensor2_black = 1.0 - (double)digitalRead(LINE_SENSOR_2_PIN);
  sensor3_black = 1.0 - (double)digitalRead(LINE_SENSOR_3_PIN);
  sensor4_black = 1.0 - (double)digitalRead(LINE_SENSOR_4_PIN);
  sensor5_black = 1.0 - (double)digitalRead(LINE_SENSOR_5_PIN);
  num_blacks = sensor1_black + sensor2_black + sensor3_black + sensor4_black + sensor5_black;

  lineSensorError = sensor1_black * EXTREME_LINE_ERROR
                    + sensor2_black * SMALL_LINE_ERROR
                    - sensor4_black * SMALL_LINE_ERROR
                    - sensor5_black * EXTREME_LINE_ERROR;

  if(num_blacks == 0){
    lost = true;

  }
  else{
    lost = false;
    lineSensorError = lineSensorError / num_blacks;
  }
}


void setup() {
  Serial.begin(9600);
  pinMode(LINE_SENSOR_1_PIN, INPUT);
  pinMode(LINE_SENSOR_2_PIN, INPUT);
  pinMode(LINE_SENSOR_3_PIN, INPUT);
  pinMode(LINE_SENSOR_4_PIN, INPUT);
  pinMode(LINE_SENSOR_5_PIN, INPUT);

  // Init Encoders
  bool init_encoder_flag = encoder_init(LEFT_MOTOR_ENC_PIN_A, RIGHT_MOTOR_ENC_PIN_A);
  if (!init_encoder_flag){
    Serial.println("Encoder Init Failed!");
  }

  // Init motor control
  motor_init();
}

void loop() {
  currentTime = millis();

  bool init_encoder_flag; 

  double left_motor_channelA = 0;
  double right_motor_channelA = 0;

  if(currentTime - lastTime > CONTROL_PERIOD) {
    lastTime = currentTime;
    
    //! READ INPUTS
    if(left_motor_channelA != digitalRead(LEFT_MOTOR_ENC_PIN_A)){
      Serial.print("Speed at edge left:");
      Serial.println(leftMotorSpeed);
    } 
    if(right_motor_channelA != digitalRead(RIGHT_MOTOR_ENC_PIN_A)){
      Serial.print("Speed at edge right:");
      Serial.println(rightMotorSpeed);
    } 
    left_motor_channelA = digitalRead(LEFT_MOTOR_ENC_PIN_A);
    right_motor_channelA = digitalRead(RIGHT_MOTOR_ENC_PIN_A);

    //! PROCESSING
    leftMotorSpeed = leftMotorSpeed * 2;
    rightMotorSpeed = rightMotorSpeed * 2;
    
    //readLineSensor();
    /*if(lost){
      Serial.println("LOST");
      return;
    }
    Serial.print("Line error: ");
    Serial.println(lineSensorError);*/

    //! OUTPUT
    set_motor(leftMotorSpeed, rightMotorSpeed);
    Serial.print("Motor Speed: ");
    Serial.print("left: ");
    Serial.print(leftMotorSpeed);
    Serial.print(" right: ");
    Serial.print(rightMotorSpeed);
    Serial.print(" left_motor_channelA: ");
    Serial.print(left_motor_channelA);
    Serial.print(" right_motor_channelA: ");
    Serial.println(right_motor_channelA);

    //Serial.println("v1: %.2f v2: %.2f channel_A left: %.2f channel_A_right: %.2f", leftMotorSpeed, rightMotorSpeed, left_motor_channelA, right_motor_channelA);
  }
}


