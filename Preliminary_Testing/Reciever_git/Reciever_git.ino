// used Rui Santos tutorial on esp_now communication, below is a helpful link
/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

// esp_now used for communication between transmittor and reciever
#include <esp_now.h>
#include <WiFi.h>
// Using ESP32Servo library, regular "Servo.h" does not work with ESP32
#include <ESP32Servo.h>
// Using robojax_L298N_DC_motor library. Not required. Can use PWM signal on enable pins and high/low on input pins for diretion
#include <Robojax_L298N_DC_motor.h>


Servo Right_Servo;  // right front servo motor: MG995
Servo Left_Servo; // left front servo motor: MG995

// #define Right_servo_Pin 32 providing PWM signal to right wheel
// #define Left_servo_Pin 33 providing PWM signal to right wheel
int Right_servo_Pin = 32;
int Left_servo_Pin = 33;

// setting PWM properties - only using rear_right in this testing. Same signal will be sent to both wheels
const int freq = 5000;
const int rear_r_PWM = 0;
const int rear_l_PWM = 1;
const int resolution = 8;

// Rear right and left motors


// motor settings
#define IN1 19
#define IN2 4
#define ENA 5 // this pin must be PWM enabled pin

const int CCW = 2; // defined for robojax contoller
const int CW  = 1; // defined for robojax contoller


// use the line below for single motor
Robojax_L298N_DC_motor motor(IN1, IN2, ENA);

// Structure of message: a is power data, b is steering data
typedef struct struct_message {
    int a;
    int b;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  /*
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Vel: ");
  Serial.println(myData.a);
  Serial.print("Steer: ");
  Serial.println(myData.b);
  Serial.println();
  */
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  
  //L298N DC Motor by Robojax.com
  motor.begin();
  
  Right_Servo.attach(Right_servo_Pin, 500, 2400);  // Right Wheel, must specify the PWM range with this model to get full ROM
  Left_Servo.attach(Left_servo_Pin, 500, 2400);  // Left Wheel, must specify the PWM range with this model to get full ROM
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  Right_Servo.attach(Right_servo_Pin, 500, 2400);  // Right Wheel
  Left_Servo.attach(Left_servo_Pin, 500, 2400);  // Left Wheel
  int angle = map(myData.b, 0, 255, 0, 180);  // convert 8 bit angle to 180 degree signal for servo
  Right_Servo.write(angle); //Turn right wheel
  Left_Servo.write(angle); // Turn left Wheel
  Serial.print("Angle: ");
  Serial.println(angle);
  int drive_pwr = map(myData.a, 0, 255, -255, 255); // mapping 8 bit power data to forward and reverse

  // send signals to L298 motor controller for forward movement
  if (drive_pwr >= 0) {
    int robo_pwr = map(drive_pwr, 0, 255, 0, 100);
    motor.rotate(motor1, robo_pwr, CW);
    Serial.println(robo_pwr);
    Serial.println("CW");
  }
  // send signals to L298 motor controller for reverse movement
  else {
    int robo_pwr = map(-drive_pwr, 1, 255, 0, 100);
    motor.rotate(motor1, robo_pwr, CCW);
    Serial.println(robo_pwr);
    Serial.println("CCW");
  }
}
