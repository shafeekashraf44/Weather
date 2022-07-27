#ifndef SESNOR_DRIVER_H
#define SESNOR_DRIVER_H

#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <PubSubClient.h>
#include <Arduino.h>

#define WindSensorPin (14)
#define WindVanePin (32) 
#define VaneOffset 0
//Defines
//Function prototypes
void sht21_read();
void sht21_setup();

String readings_packet();

String readings();

void mqtt_publish();

void wind_speed_setup();
void wind_speed_read();
void wind_direction_read();

void rainsensor_setup();
void mqtt_setup();
#endif
