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
// #include <HardwareSerial.h>

// HardwareSerial SerialPort(2); // first encoder using UART 2

Servo Right_Servo;  // right front servo motor: MG995
Servo Left_Servo; // left front servo motor: MG995

int angle = 0; // intial angle
int drive_pwr = 122; // intial power
int encode_l = 0;
int encode_r = 0;
int last_millis = 0;

// Encoder Period (1 / Frequency)
float encoder_per = 0.02;

// Encoder Counts per rotation
float count_const = 2248.86;

// Real Speeds
float ave_real_speed = 0;
float l_real_speed = 0;
float r_real_speed = 0;


float speed_desired = 0;
float speed_error = 0;
float speed_error_integral = 0;


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


// Motor Connections (Both must use PWM pins)

// LEFT wheel PWM pins
#define left_fwd_PWM 22
#define left_bwd_PWM 23

// RIGHT wheel PWM pins
#define right_fwd_PWM 15
#define right_bwd_PWM 4

// RIGHT Wheel



// Structure of message: a is power data, b is steering data
typedef struct struct_message {
    int id;
    int a;
    int b;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;
struct_message board3;

// Create an array with all the structures
struct_message boardsStruct[3] = {board1, board2, board3};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  // Serial.print("Packet received from: ");
  // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  // Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.printf("Board ID %u: %u bytes\n", myData.id, len);
  // Update the structures with the new incoming data
  boardsStruct[myData.id-1].a = myData.a;
  boardsStruct[myData.id-1].b = myData.b;
  // Serial.printf("a value: %d \n", boardsStruct[myData.id-1].a);
  // Serial.printf("b value: %d \n", boardsStruct[myData.id-1].b);
  // Serial.println();
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set motor connections as outputs
  pinMode(left_fwd_PWM, OUTPUT);
  pinMode(left_bwd_PWM, OUTPUT);
  pinMode(right_fwd_PWM, OUTPUT);
  pinMode(right_bwd_PWM, OUTPUT);

  // SerialPort.begin(9600, SERIAL_8N1, 16, 17);
  
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
  int angle = map(boardsStruct[0].b, 0, 255, 0, 180);  // convert 8 bit angle to 180 degree signal for servo
  Right_Servo.write(angle); //Turn right wheel
  Left_Servo.write(angle); // Turn left Wheel
  // Serial.print("Angle: ");
  // Serial.println(angle);

  if (millis() > (last_millis + 20)){
  
    int drive_pwr = map(boardsStruct[0].a, 0, 255, -255, 255); // mapping 8 bit power data to forward and reverse
    /*
    // send signals for forward movement
    if (drive_pwr >= 0) {
      analogWrite(left_fwd_PWM, drive_pwr);
      analogWrite(left_bwd_PWM, 0);
      analogWrite(right_fwd_PWM, drive_pwr);
      analogWrite(right_bwd_PWM, 0);
      
      Serial.print("Power:");
      Serial.print(drive_pwr);
      // Serial.println("CCW");
      Serial.print(",");
      
    }
    // send signals to L298 motor controller for reverse movement
    else {
      analogWrite(left_bwd_PWM, -drive_pwr);
      analogWrite(left_fwd_PWM, 0);
      analogWrite(right_bwd_PWM, -drive_pwr);
      analogWrite(right_fwd_PWM, 0);
 
      Serial.print("Power:");
      Serial.print(drive_pwr);
      // Serial.println("CCW");
      Serial.print(",");
      
    }
    */


    // left - right readings of encoders
    int encode_r = boardsStruct[1].a;
    int encode_l = boardsStruct[2].a;

    ////////////////////////////////////////////////////////
    // PID Controller //////////////////////////////////////
    ////////////////////////////////////////////////////////

    

    l_real_speed = (encode_l / encoder_per) / count_const;
    r_real_speed = (encode_r / encoder_per) / count_const;

    ave_real_speed = (l_real_speed + r_real_speed) / 2;

    // max desired speed ~ 2.6667 rotations per second
    speed_desired = map(boardsStruct[0].a, 0, 255, -1000, 1000); // mapping 8 bit power data to forward and reverse
    speed_desired = speed_desired / 375 ; // max desired speed ~ 2.6667 rotations per second
    
    // speed_desired = (drive_pwr / 255) * (160 / 60);
    
    Serial.print("Desired Speed:");
    Serial.print(speed_desired,2);
    Serial.print(",");

    speed_error = speed_desired - ave_real_speed;

    speed_error_integral = speed_error_integral + encoder_per * speed_error;
    

    // K values - PI
    int Kp = 100;
    int Ki = 10;

    float power_input_float = speed_error * Kp + speed_error_integral * Ki;

    if (power_input_float > 255) {
      power_input_float = 255;
    }
    if (power_input_float < -255) {
      power_input_float = -255;
    }

    // LOGIC: motor brake protection
    if (ave_real_speed > 0) {
      if (power_input_float < 0) {
        // Reduce power to 10% of value to avoid rapid reversing
        power_input_float = 0.1 * power_input_float;
      }
    }
    if (ave_real_speed < 0) {
      if (power_input_float > 0) {
        // Reduce power to 10% of value to avoid rapid reversing
        power_input_float = 0.1 * power_input_float;
      }
    }

    int power_input_int = int(round(power_input_float));

    Serial.print("Real Speed:");
    Serial.print(ave_real_speed,2);
    Serial.print(",");



    Serial.print("Recommend:");
    Serial.print(power_input_int);
    Serial.print(",");


    ///////////////////////
    /// INPUTS ////////////
    ///////////////////////

    if (power_input_int >= 0) {
      analogWrite(left_fwd_PWM, power_input_int);
      analogWrite(left_bwd_PWM, 0);
      analogWrite(right_fwd_PWM, power_input_int);
      analogWrite(right_bwd_PWM, 0);
      
      Serial.print("Power:");
      Serial.print(power_input_int);
      // Serial.println("CCW");
      Serial.print(",");
      
    }
    // send signals to motor controller for reverse movement
    else {
      analogWrite(left_bwd_PWM, -power_input_int);
      analogWrite(left_fwd_PWM, 0);
      analogWrite(right_bwd_PWM, -power_input_int);
      analogWrite(right_fwd_PWM, 0);
 
      Serial.print("Power:");
      Serial.print(power_input_int);
      // Serial.println("CCW");
      Serial.print(",");
      
    }

    
    ///////////////////////////////
    
    Serial.print("Right:");
    Serial.print(encode_r);
    Serial.print(",");
    Serial.print("Left:");
    Serial.println(encode_l);
    
    last_millis = millis();
    
  }
  delay(1);
  
}
