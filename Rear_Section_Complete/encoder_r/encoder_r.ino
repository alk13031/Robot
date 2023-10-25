#ifdef ESP32
  #include <esp_now.h>
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
  #include <espnow.h>
#endif


int encoder_1_pin = D5;
int encoder_2_pin = D6;

int delta_pos = 0;
int last_millis = 0;

// right
int id = 3;

// b variable not needed
int b = 0;

// REPLACE WITH THE RECEIVER'S MAC Address : 08:3A:F2:B9:92:C4
uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0xB9, 0x85, 0x3C};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int id; // must be unique for each sender board
    int a;
    int b;
} struct_message;


// Create a struct_message called myData
struct_message myData;

// unsigned long lastTime = 0;  
// unsigned long timerDelay = 2000;  // send readings timer

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}



// Funtions
void encoder_1_change(){
  if (digitalRead(encoder_1_pin) == HIGH){
    if (digitalRead(encoder_2_pin) == LOW){
      delta_pos++;
    } else {
      delta_pos--;
    }
  } else {
    if (digitalRead(encoder_2_pin) == HIGH){
      delta_pos++;
    } else {
      delta_pos--;
    }    
  }
}
void encoder_2_change(){
  if (digitalRead(encoder_2_pin) == HIGH){
    if (digitalRead(encoder_1_pin) == HIGH){
      delta_pos++;
    } else {
      delta_pos--;
    }
  } else {
    if (digitalRead(encoder_1_pin) == LOW){
      delta_pos++;
    } else {
      delta_pos--;
    }    
  }
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(encoder_1_pin, INPUT);    // sets the digital pin 1 as input
  pinMode(encoder_2_pin, INPUT);    // sets the digital pin 2 as input
  
  // Encoder Interrupts
  attachInterrupt(encoder_1_pin, encoder_1_change, CHANGE);
  attachInterrupt(encoder_2_pin, encoder_2_change, CHANGE);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  int en_1 = digitalRead(encoder_1_pin);
  int en_2 = digitalRead(encoder_2_pin);
  /*
  Serial.print("Val 1: "); Serial.print(en_1); Serial.print("  ");
  Serial.print("Val 2: "); Serial.print(en_2); Serial.print("  ");
  Serial.println("uT");
  */

  if (millis() > (last_millis + 20)){
    noInterrupts();
    Serial.print("Delta Position: "); Serial.print(delta_pos); Serial.print("  ");
    Serial.println("");

    // Set values to send
    myData.id = 3;
    myData.a = delta_pos;
    myData.b = b;
    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    last_millis = millis();
    delta_pos = 0;
    interrupts();
  }
  
  // Serial.print("Val 2: ");
  // Serial.println(val_2);
  delay(1);

}
