/**
 * This example connects to the device specific endpoint in AWS in order to
 * publish and retrieve MQTT messages.
 */

#include <Arduino.h>

#include <ecc608.h>
#include <led_ctrl.h>
#include <log.h>
#include <lte.h>
#include <mqtt_client.h>


/* Ex1_Combined_Basic_Example_ENS160_BME280.ino

This example shows basic data retrieval from the SparkFun Environmental Combo Breakout
from the Air Quality Sensor (ENS160) and Atmospheric Sensor (BME280).

This example shows how to read sensor readings from the ENS160 (air quality index tVOC, and eCO2)
and BME280 (humidity, pressure, and current temperature) over I2C.

Modified by:
Ho Yun "Bobby" Chan @ SparkFun Electronics August, 2023
Basic Example for the ENS160 Originally Written by:
Elias Santistevan @ SparkFun Electronics October, 2022
Basic Example for the ENS160 Originally Written by:
Nathan Seidle @ SparkFun Electronics March 9th, 2018

Products:
Air Quality Sensor  (ENS160)             -  https://www.sparkfun.com/products/20844
Humidity and Temperature Sensor (BME280) -  https://www.sparkfun.com/products/13676

Repository:
https://github.com/sparkfun/SparkFun_Indoor_Air_Quality_Sensor-ENS160_Arduino_Library

SparkFun code, firmware, and software is released under the MIT
License(http://opensource.org/licenses/MIT).

*/
//#define Serial3 //Serial3USB  //Uncomment if you are using a native USB like the Atmega32U4 or SAMD21


#include <Wire.h>
#include "SparkFun_ENS160.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ENS160
#include "SparkFunBME280.h"   // Click here to get the library: http://librarymanager/All#SparkFun_BME280



SparkFun_ENS160 myENS;
BME280 myBME280;

int ensStatus;

// AWS defines which topic you are allowed to subscribe and publish too. This is
// defined by the policy The default policy with the Microchip IoT Provisioning
// Tool allows for publishing and subscribing on thing_id/topic. If you want to
// publish and subscribe on other topics, see the AWS IoT Core Policy
// documentation.
const char MQTT_SUB_TOPIC_FMT[] PROGMEM = "%s/sensors";
const char MQTT_PUB_TOPIC_FMT[] PROGMEM = "%s/sensors";

char mqtt_sub_topic[128];
char mqtt_pub_topic[128];


void setup() {
  Log.begin(115200);
  Wire1.begin();

  Serial3.begin(115200);

  if (!myENS.begin(Wire1)) {
    Serial3.println("Did not begin.");
    while (1)
      ;
  }

  if (myBME280.beginI2C(Wire1) == false)  //Begin communication over I2C
  {
    Serial3.println("The sensor did not respond. Please check wiring.");
    while (1)
      ;  //Freeze
  }

  // Reset the indoor air quality sensor's settings.
  if (myENS.setOperatingMode(SFE_ENS160_RESET)) {
    Serial3.println("Ready.");
  }
  delay(100);


  // Device needs to be set to idle to apply any settings.
  // myENS.setOperatingMode(SFE_ENS160_IDLE);

  // Set to standard operation
  // Others include SFE_ENS160_DEEP_SLEEP and SFE_ENS160_IDLE
  myENS.setOperatingMode(SFE_ENS160_STANDARD);

  // There are four values here:
  // 0 - Operating ok: Standard Operation
  // 1 - Warm-up: occurs for 3 minutes after power-on.
  // 2 - Initial Start-up: Occurs for the first hour of operation.
  //                                              and only once in sensor's lifetime.
  // 3 - No Valid Output
  ensStatus = myENS.getFlags();
  Serial3.print("Gas Sensor Status Flag: ");
  Serial3.println(ensStatus);

  LedCtrl.begin();
  LedCtrl.startupCycle();

  Log.info(F("Starting MQTT for AWS example\r\n"));

  if (!initTopics()) {
    Log.error(F("Unable to initialize the MQTT topics. Stopping..."));
    while (1) {}
  }

  if (!Lte.begin()) {
    Log.error(F("Failed to connect to operator"));
    while (1) {}
  }

  // Attempt to connect to AWS
  if (MqttClient.beginAWS()) {
    MqttClient.subscribe(mqtt_sub_topic);
  } else {
    Log.error(F("Failed to connect to AWS"));
    while (1) {}
  }






  for (uint8_t i = 0; i < 3; i++) {

    int AQI = myENS.getAQI();
    int TVOC = myENS.getTVOC();
    int ECO2 = myENS.getECO2();
    String publishMessage = "{\"aqi\":"  + String(AQI)  +String(", \"tvoc\": ") +String(TVOC) +String(", \"ecotwo\": ") +String(ECO2)+ String("\"}");
    char publishMessageChar[30] = "";
    publishMessage.toCharArray(publishMessageChar,30);


    const bool published_successfully =
      MqttClient.publish(mqtt_pub_topic, publishMessageChar);
      //MqttClient.publish(mqtt_pub_topic, "{\"light\": 9, \"temp\": 9}");

    if (published_successfully) {
      Log.info(F("Published message"));
    } else {
      Log.error(F("Failed to publish\r\n"));
    }

    delay(2000);

    String message = MqttClient.readMessage(mqtt_sub_topic);

    // Read message will return an empty string if there were no new
    // messages, so anything other than that means that there were a
    // new message
    if (message != "") {
      Log.infof(F("Got new message: %s\r\n"), message.c_str());
    }

    delay(60000);
  }

  delay(200);


  // Test MQTT publish and receive
  /*
  for (uint8_t i = 0; i < 3; i++) {

    const bool published_successfully =
      MqttClient.publish(mqtt_pub_topic, "{\"light\": 9, \"temp\": 9}");

    if (published_successfully) {
      Log.info(F("Published message"));
    } else {
      Log.error(F("Failed to publish\r\n"));
    }

    delay(2000);

    String message = MqttClient.readMessage(mqtt_sub_topic);

    // Read message will return an empty string if there were no new
    // messages, so anything other than that means that there were a
    // new message
    if (message != "") {
      Log.infof(F("Got new message: %s\r\n"), message.c_str());
    }

    delay(2000);
  }
  */
  Log.info(F("Closing MQTT connection"));

  MqttClient.end();
}

void loop() {

  
  /*
  if (myENS.checkDataStatus()) {
    Serial3.print("Air Quality Index (1-5) : ");
    Serial3.println(myENS.getAQI());
    int AQI = myENS.getAQI();

    Serial3.print("Total Volatile Organic Compounds: ");
    Serial3.print(myENS.getTVOC());
    Serial3.println("ppb");

    Serial3.print("CO2 concentration: ");
    Serial3.print(myENS.getECO2());
    Serial3.println("ppm");

    Serial3.print("Humidity: ");
    Serial3.print(myBME280.readFloatHumidity(), 0);
    Serial3.println("RH%");

    Serial3.print("Pressure: ");
    Serial3.print(myBME280.readFloatPressure(), 0);
    Serial3.println("Pa");

    Serial3.print("Alt: ");
    //Serial3.print(myBME280.readFloatAltitudeMeters(), 1);
    //Serial3.println("meters");
    Serial3.print(myBME280.readFloatAltitudeFeet(), 1);
    Serial3.println("feet");

    Serial3.print("Temp: ");
    //Serial3.print(myBME280.readTempC(), 2);
    //Serial3.println(" degC");
    Serial3.print(myBME280.readTempF(), 2);
    Serial3.println(" degF");

    Serial3.println();
  }
  */

/*
  for (uint8_t i = 0; i < 3; i++) {

    int AQI = myENS.getAQI();
    int TVOC = myENS.getTVOC();
    int ECO2 = myENS.getECO2();
    String publishMessage = "{\"aqi\":"  + String(AQI)  +String(", \"tvoc\": ") +String(TVOC) +String(", \"ecotwo\": ") +String(ECO2)+ String("\"}");
    char publishMessageChar[30] = "";
    publishMessage.toCharArray(publishMessageChar,30);


    const bool published_successfully =
      MqttClient.publish(mqtt_pub_topic, publishMessageChar);
      //MqttClient.publish(mqtt_pub_topic, "{\"light\": 9, \"temp\": 9}");

    if (published_successfully) {
      Log.info(F("Published message"));
    } else {
      Log.error(F("Failed to publish\r\n"));
    }

    delay(2000);

    String message = MqttClient.readMessage(mqtt_sub_topic);

    // Read message will return an empty string if there were no new
    // messages, so anything other than that means that there were a
    // new message
    if (message != "") {
      Log.infof(F("Got new message: %s\r\n"), message.c_str());
    }

    delay(60000);
  }
  */
  delay(200);
}


bool initTopics() {
  ATCA_STATUS status = ECC608.begin();

  if (status != ATCA_SUCCESS) {
    Log.errorf(F("Failed to initialize ECC, error code: %X\r\n"), status);
    return false;
  }

  // Find the thing ID and set the publish and subscription topics
  uint8_t thing_name[128];
  size_t thing_name_length = sizeof(thing_name);

  status =
    ECC608.readProvisionItem(AWS_THINGNAME, thing_name, &thing_name_length);

  if (status != ATCA_SUCCESS) {
    Log.errorf(
      F("Could not retrieve thingname from the ECC, error code: %X\r\n"),
      status);
    return false;
  }

  snprintf_P(mqtt_sub_topic,
             sizeof(mqtt_sub_topic),
             MQTT_SUB_TOPIC_FMT,
             thing_name);
  snprintf_P(mqtt_pub_topic,
             sizeof(mqtt_pub_topic),
             MQTT_PUB_TOPIC_FMT,
             thing_name);

  return true;
}
