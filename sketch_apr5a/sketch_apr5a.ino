#include <Arduino.h>
#include "nelito.h"

#define CONTROL_PERIOD 1 //ms
#define LOW 0 //GND
#define HIGH 3.3 //VCC

#define LEFT_MOTOR_ENC_PIN_A 5 // Channel A (CW)
//#define LEFT_MOTOR_ENC_PIN_B 13 // Channel B (CW)

#define RIGHT_MOTOR_ENC_PIN_A 21 // Channel A (CW)
//#define RIGHT_MOTOR_ENC_PIN_B 12 // Channel B (CW)

#define IR_RECEIVER_PIN 6

#define LINE_SENSOR_PWM_PIN 7
#define LINE_SENSOR_1_PIN 16
#define LINE_SENSOR_2_PIN 17
#define LINE_SENSOR_3_PIN 18
#define LINE_SENSOR_4_PIN 19
#define LINE_SENSOR_5_PIN 20

#define BTN_PIN 22

#define WHEEL_DISTANCE 0.15 //m
#define MAX_LIN_VEL 8000
#define MAX__ANG_VEL 100000
#define MAX_LIN_ACCEL 12500
#define MAX_ANG_ACCEL 100

#define EXTREME_LINE_ERROR 18.0
#define SMALL_LINE_ERROR 2.0

// Control variables
unsigned long lastTime = 0;
unsigned long currentTime = 0;

double left_motor_channelA = 0;
double right_motor_channelA = 0;
double leftMotorSpeed = 1, rightMotorSpeed = 1;

double sensor1_black, sensor2_black, sensor3_black, sensor4_black, sensor5_black, num_blacks;
double lineSensorError = 0.0;
double last_lineSensorError = 0.0;
double accum_lineSensorError = 0.0;
bool lost = false;

double robot_v = 0, robot_w = 0;
double last_robot_v = 0, last_robot_w = 0;
double Kp = 15000.0;
double Kd = 0.0;
double Ki = 0.0;
double speed_of_time = 1.25;

int state = 0;

//! Controlo de velocidade
void convert_velocity_to_speed(){
  robot_v *= speed_of_time;
  robot_w *= speed_of_time;
  leftMotorSpeed = robot_v - robot_w*WHEEL_DISTANCE/2;
  rightMotorSpeed = robot_v + robot_w*WHEEL_DISTANCE/2;
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

  robot_v = MAX_LIN_VEL * (1.0 - 2.5* abs(lineSensorError) / EXTREME_LINE_ERROR);
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
  int sensor_code = sensor1_black * 10000
                  + sensor2_black * 1000
                  + sensor3_black * 100
                  + sensor4_black * 10
                  + sensor5_black;

  num_blacks = sensor1_black + sensor2_black + sensor3_black + sensor4_black + sensor5_black;

  if(num_blacks == 0 || num_blacks == 5){
    lost = true;
  }
  else if(sensor_code >= 10100 && sensor_code <= 10111){
    lineSensorError = 10.5;
  }
  else if(sensor_code == 101 || sensor_code == 1101 || sensor_code == 11101){
    lineSensorError = - 10.5;
  }
  else{
    lost = false;
    lineSensorError = sensor1_black * EXTREME_LINE_ERROR
                    + sensor2_black * SMALL_LINE_ERROR
                    - sensor4_black * SMALL_LINE_ERROR
                    - sensor5_black * EXTREME_LINE_ERROR;

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
  pinMode(LINE_SENSOR_PWM_PIN, OUTPUT);
  pinMode(IR_RECEIVER_PIN, INPUT);

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

  if(currentTime - lastTime > CONTROL_PERIOD) {
    lastTime = currentTime;

    //! READ INPUTS
    // left_motor_channelA = digitalRead(LEFT_MOTOR_ENC_PIN_A);
    // right_motor_channelA = digitalRead(RIGHT_MOTOR_ENC_PIN_A);
    readLineSensor();
    //! PROCESSING
    if(state == 0){
      PID();
      Serial.println("State 0");
      if(digitalRead(IR_RECEIVER_PIN) == LOW){
        state = 1;
      }
    }
    else if(state == 1){
      robot_v = 7000;
      robot_w = 20000;
      Serial.println("State 1");
      if(sensor3_black != 1.0){
        state = 2;
      }
    }
    else if(state == 2){
      Serial.println("State 2");
      if(sensor1_black == 1.0 || sensor2_black == 1.0 || sensor3_black == 1.0){
        state = 0;
      }
      robot_v = 7000;
      robot_w = 20000;
    }
    //limitAcceleration()

    convert_velocity_to_speed();

    Serial.print("Left Motor: ");
    Serial.println(leftMotorSpeed);
    Serial.print("Right Motor: ");
    Serial.println(rightMotorSpeed);


    //! OUTPUT
    set_motor(leftMotorSpeed, rightMotorSpeed);
    analogWrite(LINE_SENSOR_PWM_PIN, 60);
  }
}


