/*
  Rui Santos
  Complete project details at
  https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <WiFi.h>
#include <esp_now.h>

// REPLACE WITH YOUR RECEIVER MAC Address
//uint8_t broadcastAddress[] = {0x10, 0x52, 0x1C, 0x81, 0x60, 0x8C};
uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0x44, 0x6D, 0x0C}; //ESP-Spider

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  bool d;
} struct_message;

// Create a struct_message called myData
struct_message myData;

//const int statusLED = 2;
const int UP_DOWN = 35; // Off = 0   UP =  Down =
const int LEFT_RIGHT = 34; // Off = 0   Left =  Right =
const int gumb_A = 32; // Off = 4095  On = 0
const int gumb_B = 33; //  Off = 4095  On = 0
const int gumb_Select = 27; //  Off = 4095  On = 0
const int gumb_Start = 39; //  Off = 4095  On = 0
const int gumb_Vol = 0; //  Off = 4095  On = 0
const int gumb_Menu = 13; //  Off = 3431 On = 0
const int tempAnalogInPin = 26; // Analogni input pin Senzora Temperature
float tempC1;
int v;
const bool serialoutput = true;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
//                                                : "Delivery Fail");
}

#define RXD2 2
#define TXD2 4

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(0, INPUT);
  //pinMode(2, OUTPUT);
  pinMode(13, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(32, INPUT);
  pinMode(33, INPUT);
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  pinMode(39, INPUT);
  //digitalWrite(2, LOW);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    //Serial.println("Failed to add peer");
    return;
  }
}

float temperature(int analog) {
  double temp =
      analog * 3.30 / 4095.0; // Analognu vrijednost pretvaramo u otpor
  temp = temp - 0.40;         // Oduzimamo 500mA tj. -40°C
  temp = temp / 0.01;         // Pretvaramo vrijednosti u °C
  return temp;
}

void loop() {
  // Set values to send
  // strcpy(myData.a, "THIS IS A CHAR");
  // myData.b = random(1,20);
  // myData.c = 1.2;
  // myData.d = false;

  tempC1 = temperature(analogRead(tempAnalogInPin)); // Pozivamo funkciju za pretvorbu analognih
                                    // očitanja u stupnjeve Cezijeve

  if ( analogRead(UP_DOWN) == 4095 ) { // || OR -
    strcpy(myData.a, "UP");
    myData.b = 35;
    myData.c = tempC1;
    myData.d = true;
    //return 0;
    if (serialoutput) {
      //Serial.print("UP ");
    }
    Serial2.print("1");
  }

    if ( analogRead(UP_DOWN) > 1700 && analogRead(UP_DOWN) < 2100 ) { // || OR -
    strcpy(myData.a, "DOWN");
    myData.b = 350;
    myData.c = tempC1;
    myData.d = true;
    //return 0;
    if (serialoutput) {
      //Serial.print("DOWN ");
    }
    Serial1.print("2");
  }

  if ( analogRead(LEFT_RIGHT) == 4095 ) { // || OR -
    strcpy(myData.a, "LEFT");
    myData.b = 34;
    myData.c = tempC1;
    myData.d = true;
    //return 0;
    if (serialoutput) {
      //Serial.print("LEFT ");
    }
    Serial2.print("3");
  }

    if ( analogRead(LEFT_RIGHT) > 1700 && analogRead(LEFT_RIGHT) < 2100 ) { // || OR -
    strcpy(myData.a, "RIGHT");
    myData.b = 340;
    myData.c = tempC1;
    myData.d = true;
    //return 0;
    if (serialoutput) {
      //Serial.print("RIGHT ");
    }
    Serial2.print("4");
  }

  if ( analogRead(gumb_A) == 0 ) { // || OR -
    strcpy(myData.a, "gumb_A");
    myData.b = 32;
    myData.c = tempC1;
    myData.d = true;
    //return 0;
    if (serialoutput) {
      //Serial.print("gumb_A ");
    }
    Serial2.print("Bok");
  }

  if ( analogRead(gumb_B) == 0 ) { // || OR -
    strcpy(myData.a, "gumb_B");
    myData.b = 33;
    myData.c = tempC1;
    myData.d = true;
    //return 0;
    if (serialoutput) {
      //Serial.print("gumb_B ");
    }
  }

  if ( analogRead(gumb_Select) == 4095 ) { // || OR -
    strcpy(myData.a, "gumb_Select");
    myData.b = 27;
    myData.c = tempC1;
    myData.d = true;
    //return 0;
    if (serialoutput) {
      //Serial.print("gumb_Select ");
    }
  }

    if ( analogRead(gumb_Start) == 0 ) { // || OR -
    strcpy(myData.a, "gumb_Start");
    myData.b = 39;
    myData.c = tempC1;
    myData.d = true;
    //return 0;
    if (serialoutput) {
      //Serial.print("gumb_Start ");
    }
  }

    if ( analogRead(gumb_Vol) == 4095 ) { // || OR -
    strcpy(myData.a, "gumb_Vol");
    myData.b = 0;
    myData.c = tempC1;
    myData.d = true;
    //return 0;
    if (serialoutput) {
      //Serial.print("gumb_Vol ");
    }
  }
// POPRAVITI VRIJEDNOSTI
    if ( analogRead(gumb_Menu) == 3431 ) { // || OR - POPRAVITI VRIJEDNOSTI
    strcpy(myData.a, "gumb_Menu");
    myData.b = 13;
    myData.c = tempC1;
    myData.d = true;
    //return 0;
    if (serialoutput) {
      //Serial.print("gumb_Menu ");
    }
    Serial2.print("9");
  }
//Serial.print("gumb_Menu ");
//Serial.println(analogRead(gumb_Menu));
  // Send message via ESP-NOW
  esp_err_t result =
      esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    //Serial.println("Sent with success");
  } else {
    //Serial.println("Error sending the data");
  }
  delay(50);
}
