/*

  RS485_HalfDuplex.pde - example using ModbusMaster library to communicate
  with EPSolar LS2024B controller using a half-duplex RS485 transceiver.

  This example is tested against an EPSolar LS2024B solar charge controller.
  See here for protocol specs:
  http://www.solar-elektro.cz/data/dokumenty/1733_modbus_protocol.pdf

  Library:: ModbusMaster
  Author:: Marius Kintel <marius at kintel dot net>

  Copyright:: 2009-2016 Doc Walker

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/

#include <HardwareSerial.h>
#include <ModbusMaster.h>
#include <DHTesp.h>         // DHT for ESP32 library
#include <WiFi.h>           // WiFi control for ESP32
#include <ThingsBoard.h>    // ThingsBoard SDK



#ifdef __cplusplus
  extern "C" {
 #endif
 
  uint8_t temprature_sens_read();
 
#ifdef __cplusplus
}
#endif
 
uint8_t temprature_sens_read();




// Helper macro to calculate array size
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

// WiFi access point
#define WIFI_AP_NAME        "Phong Tro"
// WiFi password
#define WIFI_PASSWORD       "12345678hailan"

// See https://thingsboard.io/docs/getting-started-guides/helloworld/ 
// to understand how to obtain an access token
#define TOKEN               "Pi9SkK2zOqrcGpK7vlk4"
// ThingsBoard server instance.
#define THINGSBOARD_SERVER  "cloud-obit.com"

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD    115200

// Initialize ThingsBoard client
WiFiClient espClient;
// Initialize ThingsBoard instance
ThingsBoard tb(espClient);
// the Wifi radio's status
int status = WL_IDLE_STATUS;




//#define wait 20
#define wait 20
#define number_device  1


/*!
  We're using a MAX485-compatible RS485 Transceiver.
  Rx/Tx is hooked up to the hardware serial port at 'Serial'.
  The Data Enable and Receiver Enable pins are hooked up as follows:
*/


// instantiate ModbusMaster object


void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    digitalWrite(2,!digitalRead(2));
    delay(1000);
    Serial.print(".");
  }
  digitalWrite(2,1);
  Serial.println("Connected to AP");
}



void setup()
{
  pinMode(2,OUTPUT); //led wifi
  digitalWrite(2,1);
  delay(100);
  Serial.begin(SERIAL_DEBUG_BAUD);
  WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
  InitWiFi();
  Serial.println("Init gateway");
  // Modbus communication runs at 115200 baud
}


bool subscribed = false;
bool state = true;
uint8_t result, b;
uint16_t data[6];
float red, green, orange, temp, vref, vin;
uint32_t tick;
char* gateway_age = "gateway_age(mS):)";


const char* a= "test";
char buff[30];
uint32_t count_read;  
unsigned long check_wifi = 30000;

void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      //WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}



void loop()
{
 

  // Read 16 registers starting at 0x3100)
  // Reconnect to WiFi, if needed
  
  if ((WiFi.status() != WL_CONNECTED) && (millis() > check_wifi))  {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    for(int i=0; i<5; i++)
    {
    digitalWrite(2,!digitalRead(2));
    delay(100);
    }
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    check_wifi = millis() + 30000;
    digitalWrite(2,0);
  }
  
  // Reconnect to ThingsBoard, if needed
  if (!tb.connected()) {
    subscribed = false;
    digitalWrite(2,0);
    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect");
      for(int i=0; i<5; i++)
      {
      digitalWrite(2,1);
      delay(100);
      digitalWrite(2,0);
      delay(200);
      }
      return;
    }
  }
  else { digitalWrite(2,1); }// on led
  

    long rssi_esp = WiFi.RSSI();
    tb.sendTelemetryFloat("rssi_test: ",rssi_esp);
    Serial.print("RSSI");
    Serial.println(rssi_esp);
    //tb.sendTelemetryInt(buff,tick);

  
  delay(1000);
  tb.loop();
}
