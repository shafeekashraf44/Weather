#include <WiFi.h>
#include <PubSubClient.h>
#include "sensor_driver.h"
 
const char* ssid = "IQ_Shared";
const char* password =  "6R$$gB@12";

 

void setup() 
{
 
  Serial.begin(115200);
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
 
  Serial.println("Connected to the WiFi network");
 
  mqtt_setup();
  sht21_setup();
  wind_speed_setup();
  rainsensor_setup();

 
}
 
void loop() 
{
  delay(1000);
  sht21_read();
  wind_speed_read();
  wind_direction_read();
  mqtt_publish();
}
