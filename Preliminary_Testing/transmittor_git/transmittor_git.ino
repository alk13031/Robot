// used Rui Santos tutorial on esp_now communication, below is a helpful link
/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <WiFi.h>

// must put your specific broadcast addres in place of the "XX", this is for the reciever
uint8_t broadcastAddress[] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX};

typedef struct struct_message {
  int a;
  int b;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// the number of the LED pin ***NOT REQUIRED***, used to verify steering (right, left) and power (forward, reverse)
const int right_ledPin = 15;  // 15 corresponds to GPIO15
const int left_ledPin = 23;  // 15 corresponds to GPIO23

const int fwd_ledPin = 19;  // 15 corresponds to GPIO15
const int back_ledPin = 21;  // 15 corresponds to GPIO15

// setting PWM properties
const int freq = 5000;
const int ledChannel_r = 0;
const int ledChannel_l = 1;
const int ledChannel_f = 2;
const int ledChannel_b = 3;
const int resolution = 8;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  WiFi.mode(WIFI_STA);
  

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  // configure LED PWM functionalitites
  ledcSetup(ledChannel_r, freq, resolution);
  ledcSetup(ledChannel_l, freq, resolution);
  ledcSetup(ledChannel_f, freq, resolution);
  ledcSetup(ledChannel_b, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(right_ledPin, ledChannel_r);
  ledcAttachPin(left_ledPin, ledChannel_l);
  ledcAttachPin(fwd_ledPin, ledChannel_f);
  ledcAttachPin(back_ledPin, ledChannel_b);
  
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on each analog pin potentiometer pin
  int steer_analogValue = analogRead(35);
  int velocity_analogValue = analogRead(34);

  // map analog values
  if (steer_analogValue > 2047) {
    int right_steer_pwm = map(steer_analogValue, 2048, 4095, 0, 255);
    int left_steer_pwm = 0;
    ledcWrite(ledChannel_r, right_steer_pwm);
    ledcWrite(ledChannel_l, left_steer_pwm);
  }
  if (steer_analogValue < 2047) {
    int right_steer_pwm = 0;
    int left_steer_pwm = map(steer_analogValue, 2047, 0, 0, 255);
    ledcWrite(ledChannel_r, right_steer_pwm);
    ledcWrite(ledChannel_l, left_steer_pwm);
  }

  // map analog values: my power potentiometer did not have full ROM, so a smaller mapped range was used to match
  if (velocity_analogValue > 1765) {
    int fwd_steer_pwm = 0;
    int back_steer_pwm = map(velocity_analogValue, 1766, 2418, 0, 255);
    ledcWrite(ledChannel_f, fwd_steer_pwm);
    ledcWrite(ledChannel_b, back_steer_pwm);
  }
  if (velocity_analogValue < 1765) {
    int back_steer_pwm = 0;
    int fwd_steer_pwm = map(velocity_analogValue, 1765, 1114, 0, 255);
    ledcWrite(ledChannel_f, fwd_steer_pwm);
    ledcWrite(ledChannel_b, back_steer_pwm);
  }

  // velocity value from 0 - 255, (0 - full backward, 255 - full forwards)
  int velocity_msg = map(velocity_analogValue, 1114, 2418, 255, 0);

  // steer value from 0 - 255, (0 - full left, 255 - full right)
  int steer_msg = map(steer_analogValue, 0, 4095, 0, 255);

  myData.a = velocity_msg;
  myData.b = steer_msg;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  // print out the value you read:
  Serial.print("Power: ");
  Serial.println(velocity_msg);
  Serial.print("Steering: ");
  Serial.println(steer_msg);

  delay(10);
}
