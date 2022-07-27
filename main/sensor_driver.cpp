#include "sensor_driver.h"
#include "SHT2x.h"
#include <iostream>
#include <PubSubClient.h>


uint32_t start;
uint32_t stop;
SHT2x sht;

unsigned long lastMillis;
long r;
String createPayload;

long last_time = 0;

WiFiClient espClient;
PubSubClient client(espClient);
const char* mqttServer = "mqtt.ambeedata.com";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";


volatile unsigned long Rotations;         // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine
///////////////////////////////////////////////////////////////
int VaneValue;    // raw analog value from wind vane
int Direction;    // translated 0 - 360 direction
int CalDirection; // converted value with offset applied
int LastValue;
char direct[] = "";
#define Offset 0;
//////////////////////////////////////////////////////////////
float WindSpeed; // speed miles per hour
String wind_direction;
int wd;

// volatile unsigned long tiptime = millis();
#define Bucket_Size 0.2             // bucket size to trigger tip count
#define RG11_pin 35                 // digital pin RG11 connected to
volatile unsigned long tipCount;    // rain bucket tip counter used in interrupt routine
volatile unsigned long contactTime; // timer to manage any rain contact bounce in interrupt routine
volatile float totalRainfall;       // total amount of rainfall detected


int temperature;
int humidity;
String mac = WiFi.macAddress();

String post;

void sht21_setup()
{
  // Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("SHT2x_LIB_VERSION: \t");
  Serial.println(SHT2x_LIB_VERSION);

  sht.begin();

  uint8_t stat = sht.getStatus();
  Serial.print(stat, HEX);
  Serial.println();
}

void sht21_read()
{

  start = micros();
  sht.read();
  stop = micros();

  // Serial.print("\t");
  // Serial.print(stop - start);
  // Serial.print("\t");
  Serial.print(sht.getTemperature(), 1);
  temperature = sht.getTemperature();
  Serial.print("\t");
  Serial.println(sht.getHumidity(), 1);
  humidity = sht.getHumidity();
  delay(1000);
}

String readings()
{
  String payload = "{";
  payload += "\"devId\":";
  payload += "\"";
  payload += mac;
  payload += "\"";
  payload += ",";
  payload += "\"temp\":";
  payload += temperature;
  payload += ",";
  payload += "\"hum\":";
  payload += humidity;
  payload += ",";
  payload += "\"ws\":";
  payload += WindSpeed;
  payload += ",";
  payload += "\"wd\":";
  payload += wd;
  payload += ",";
  payload += "\"rs\":";
  payload +=  totalRainfall;;

  String payload2 = "{";
  payload2 += "\"data\":";
  payload2 += "[";
  payload2 += payload;
  payload2 += "}";
  payload2 += "]";
  payload2 += "}";
  Serial.println(payload2);
  return payload2;
}

void mqtt_publish()
{
  if (millis() - lastMillis >= 2 * 10 * 100)
  {
    
    char buff_pub[130];
    String payload_pub = readings();
    sprintf(buff_pub,"Ambee_AQV4_Climatesense22","%d", payload_pub.length());
    client.publish("Ambee_AQV4_Climatesense22",payload_pub.c_str());
    totalRainfall = 0;
    tipCount = 0;

  }
}

// Anemometer
void getHeading(int direction)
{
  if (direction < 22)
  {
    Serial.println("N");
    wind_direction = "N";
  }

  else if (direction < 67)
  {
    Serial.println("NE");
    wind_direction = "NE";
  }

  else if (direction < 112)
  {
    Serial.println("E");
    wind_direction = "E";
  }

  else if (direction < 157)
  {
    Serial.println("SE");
    wind_direction = "SE";
  }

  else if (direction < 212)
  {
    Serial.println("S");
    wind_direction = "S";
  }

  else if (direction < 247)
  {
    Serial.println("SW");
    wind_direction = "SW";
  }

  else if (direction < 292)
  {
    Serial.println("W");
    wind_direction = "W";
  }

  else if (direction < 337)
  {
    Serial.println("NW");
    wind_direction = "NW";
  }

  else
  {
    Serial.println("N");
    wind_direction = "N";
  }
  Serial.println("WIND DIRECTION");
  Serial.print("\t\t");
  Serial.println(wind_direction);
}

void isr_rotation()
{

  if ((millis() - ContactBounceTime) > 15)
  { // debounce the switch contact.
    Rotations++;
    ContactBounceTime = millis();
  }
}

void wind_speed_setup()
{
  pinMode(WindSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);
}

void wind_speed_read()
{
  Rotations = 0; // Set Rotations count to 0 ready for calculations

  sei();       // Enables interrupts
  delay(2000); // Wait 3 seconds to average
  cli();       // Disable interrupts

  // convert to mp/h using the formula V=P(2.25/T)
  // V = P(2.25/3) = P * 0.75

  WindSpeed = Rotations * 0.75;

  Serial.print(Rotations);
  Serial.print("\t\t");
  Serial.println(WindSpeed);
}

void wind_direction_read()
{

  VaneValue = analogRead(WindVanePin);
  Direction = map(VaneValue, 0, 4095, 0, 359);
  CalDirection = Direction + VaneOffset;

  if (CalDirection > 360)
    CalDirection = CalDirection - 360;

  if (CalDirection < 0)
    CalDirection = CalDirection + 360;

  if (abs(CalDirection - LastValue) > 5)
  {
    Serial.print(VaneValue);
    Serial.print("\t\t");
    Serial.print(CalDirection);
    Serial.print("\t\t");
    wd = CalDirection;
    // getHeading(CalDirection);
    LastValue = CalDirection;
  }
}




void isr_rg()
{
  if ((millis() - contactTime) > 100)
  { // debounce of sensor signal
    tipCount++;
    totalRainfall = tipCount * Bucket_Size;
    contactTime = millis();
  }
}

void rainsensor_setup()
{
  //cli();//Disable interrupts
  tipCount = 0;
  totalRainfall = 0;
  pinMode(RG11_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(RG11_pin), isr_rg, FALLING);
  sei(); // Enable Interrupts

  //if this is disrupting consider cli();
}

void mqtt_setup()
{
  client.setServer(mqttServer, mqttPort);
 
  while (!client.connected()) 
  {
    Serial.println("Connecting to MQTT...");
 
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
 
      Serial.println("connected");
 
    } else {
 
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
 
    }
  }
}
