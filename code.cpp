/****************************************
   Include Libraries
 ****************************************/
#include <WiFiManager.h>  // Include the WiFiManager library
#include <PubSubClient.h>
#include <stdio.h>
#include <SparkFun_ADXL345.h>  // SparkFun ADXL345 Library
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <Adafruit_BMP085.h>
#include <ESP_Mail_Client.h>

unsigned long lastReconnectAttempt = 0;        // Tracks the last reconnect attempt
const unsigned long reconnectInterval = 5000;  // Reconnect interval in milliseconds

// #define USE_WIFI_MANAGER

/****************************************
   Define Constants
 ****************************************/
#define TOKEN "BBUS-Gg1IdSgRCQcaLFMtrjvvQIc4lPG7lH"  // Put your Ubidots' TOKEN

#ifndef USE_WIFI_MANAGER
const char* ssid = "****";
const char* password = "***";
#endif

#define DEVICE_LABEL "FallDetect"  // Assign the device label
#define MQTT_CLIENT_NAME "fall"    // MQTT client Name

#define SMTP_HOST "smtp.gmail.com"

#define SMTP_PORT esp_mail_smtp_port_587  // port 465 is not available for Outlook.com

/* The log in credentials */
#define AUTHOR_EMAIL "****"
#define AUTHOR_PASSWORD "****"

/* Recipient email address */
#define RECIPIENT_EMAIL "******"

/* Declare the global used SMTPSession object for SMTP transport */
SMTPSession smtp;

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status);

char mqttBroker[] = "industrial.api.ubidots.com";
char payload[300];  // Adjusted payload size for a single value
char topic[150];

int accX, accY, accZ;
int32_t pressure, altitude;
int heartRate, spo2;
bool fallDetected = false;

bool mqttConnectFlag = false;

volatile unsigned long lastPublishTime1 = 0;  // Stores the last time data1 was published
volatile unsigned long lastPublishTime2 = 0;  // Stores the last time data2 was published
volatile unsigned long lastPublishTime3 = 0;  // Stores the last time data3 was published

/****************************************
   Initialize constructors for objects
 ****************************************/
#ifdef USE_WIFI_MANAGER
WiFiManager wifiManager;  // Initialize WiFiManager
#endif

WiFiClient ubidots;
PubSubClient client(ubidots);
ADXL345 adxl = ADXL345();

// PulseOximeter is the higher level interface to the sensor
// it offers:
//  * beat detection reporting
//  * heart rate calculation
//  * SpO2 (oxidation level) calculation
PulseOximeter pox;
Adafruit_BMP085 bmp;
/* Declare the message class */
SMTP_Message message;

/* Declare the Session_Config for user defined session credentials */
Session_Config config;

void initAccelerometer() {
  adxl.powerOn();  // Power on the ADXL345

  adxl.setRangeSetting(16);  // Give the range settings
  // Accepted values are 2g, 4g, 8g or 16g
  // Higher Values = Wider Measurement Range
  // Lower Values = Greater Sensitivity

  adxl.setSpiBit(0);  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
  // Default: Set to 1
  // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library

  adxl.setActivityXYZ(1, 0, 0);   // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(75);  // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)

  adxl.setInactivityXYZ(1, 0, 0);   // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityThreshold(75);  // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(10);       // How many seconds of no activity is inactive?

  adxl.setTapDetectionOnXYZ(0, 0, 1);  // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)

  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(50);      // 62.5 mg per increment
  adxl.setTapDuration(15);       // 625 μs per increment
  adxl.setDoubleTapLatency(80);  // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);  // 1.25 ms per increment

  // Set values for what is considered FREE FALL (0-255)
  adxl.setFreeFallThreshold(6);  // (5 - 9) recommended - 62.5mg per increment 7
  adxl.setFreeFallDuration(45);  // (20 - 70) recommended - 5ms per increment 30

  // Setting all interupts to take place on INT1 pin
  //adxl.setImportantInterruptMapping(1, 1, 1, 1, 1);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);"
  // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
  // This library may have a problem using INT2 pin. Default to INT1 pin.

  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  adxl.InactivityINT(1);
  adxl.ActivityINT(1);
  adxl.FreeFallINT(1);
  adxl.doubleTapINT(1);
  adxl.singleTapINT(1);

  // attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);   // Attach Interrupt
}

void onBeatDetected() {
  Serial.println("Beat!");
}

void initMax30100() {
  if (!pox.begin()) {
    Serial.println("FAILED");
    for (;;)
      ;
  } else {
    Serial.println("SUCCESS");
  }

  pox.setIRLedCurrent(MAX30100_LED_CURR_20_8MA);

  // Register a callback for the beat detection
  pox.setOnBeatDetectedCallback(onBeatDetected);
}

void initPressureSensor() {
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
}

/****************************************
   Auxiliar Functions
 ****************************************/

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected to MQTT broker");
      mqttConnectFlag = true;
    } else {
      Serial.print("Connection failed, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 2 seconds...");
      mqttConnectFlag = false;
      delay(2000);  // Short delay before retry
    }
  }
  // else{
  //   Serial.println("connected");
  //   mqttConnectFlag = true;
  // }
}

void sendMail() {
  /* Set the message headers */
  message.sender.name = F("name of the sender");
  message.sender.email = AUTHOR_EMAIL;

  String subject = "!!!Fall Alert!!! ";
  message.subject = subject;

  message.addRecipient(F("recipient name"), RECIPIENT_EMAIL);

  String textMsg = "A fall has being detected.\n";


  message.text.content = textMsg;

  message.text.transfer_encoding = "base64";  // recommend for non-ASCII words in message.

  /** If the message to send is a large string, to reduce the memory used from internal copying  while sending,
     you can assign string to message.text.blob by cast your string to uint8_t array like this

     String myBigString = "..... ......";
     message.text.blob.data = (uint8_t *)myBigString.c_str();
     message.text.blob.size = myBigString.length();

     or assign string to message.text.nonCopyContent, like this

     message.text.nonCopyContent = myBigString.c_str();

     Only base64 encoding is supported for content transfer encoding in this case.
  */

  /** The Plain text message character set e.g.
     us-ascii
     utf-8
     utf-7
     The default value is utf-8
  */
  message.text.charSet = F("utf-8");  // recommend for non-ASCII words in message.

  // If this is a reply message
  // message.in_reply_to = "<parent message id>";
  // message.references = "<parent references> <parent message id>";

  /** The message priority
     esp_mail_smtp_priority_high or 1
     esp_mail_smtp_priority_normal or 3
     esp_mail_smtp_priority_low or 5
     The default value is esp_mail_smtp_priority_low
  */
  message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_low;

  // message.response.reply_to = "someone@somemail.com";
  // message.response.return_path = "someone@somemail.com";

  /** The Delivery Status Notifications e.g.
     esp_mail_smtp_notify_never
     esp_mail_smtp_notify_success
     esp_mail_smtp_notify_failure
     esp_mail_smtp_notify_delay
     The default value is esp_mail_smtp_notify_never
  */
  // message.response.notify = esp_mail_smtp_notify_success | esp_mail_smtp_notify_failure | esp_mail_smtp_notify_delay;

  /* Set the custom message header */
  message.addHeader(F("Message-ID: <abcde.fghij@gmail.com>"));

  // For Root CA certificate verification (ESP8266 and ESP32 only)
  // config.certificate.cert_data = rootCACert;
  // or
  // config.certificate.cert_file = "/path/to/der/file";
  // config.certificate.cert_file_storage_type = esp_mail_file_storage_type_flash; // esp_mail_file_storage_type_sd
  // config.certificate.verify = true;

  // The WiFiNINA firmware the Root CA certification can be added via the option in Firmware update tool in Arduino IDE

  /* Connect to server with the session config */

  // Library will be trying to sync the time with NTP server if time is never sync or set.
  // This is 10 seconds blocking process.
  // If time reading was timed out, the error "NTP server time reading timed out" will show via debug and callback function.
  // You can manually sync time by yourself with NTP library or calling configTime in ESP32 and ESP8266.
  // Time can be set manually with provided timestamp to function smtp.setSystemTime.

  /* Set the TCP response read timeout in seconds */
  // smtp.setTCPTimeout(10);

  /* Connect to the server */
  if (!smtp.connect(&config)) {
    // MailClient.printf("Connection error, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
    return;
  }

  /** Or connect without log in and log in later

     if (!smtp.connect(&config, false))
       return;

     if (!smtp.loginWithPassword(AUTHOR_EMAIL, AUTHOR_PASSWORD))
       return;
  */

  if (!smtp.isLoggedIn()) {
    Serial.println("Not yet logged in.");
  } else {
    if (smtp.isAuthenticated())
      Serial.println("Successfully logged in.");
    else
      Serial.println("Connected with no Auth.");
  }

  /* Start sending Email and close the session */
  if (!MailClient.sendMail(&smtp, &message)) {  // MailClient.printf("Error, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
  }
}
/****************************************
   Main Functions
 ****************************************/

void setup() {
  Serial.begin(115200);

#ifdef USE_WIFI_MANAGER
  // wifiManager.setConfigPortalBlocking(true);
  // wifiManager.setConfigPortalTimeout(60);

  // Automatically connect using saved credentials or start configuration portal
  if (wifiManager.autoConnect("FallDetection", "password")) {
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Configportal running");
  }
#else
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
#endif

  initAccelerometer();
  initMax30100();
  // initPressureSensor();

  /*  Set the network reconnection option */
  MailClient.networkReconnect(true);
  smtp.debug(1);

  /* Set the callback function to get the sending results */
  smtp.callback(smtpCallback);


  /* Set the session config */
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_EMAIL;
  config.login.password = AUTHOR_PASSWORD;
  config.login.user_domain = F("127.0.0.1");
  config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
  config.time.gmt_offset = 3;
  config.time.day_light_offset = 0;


  client.setServer(mqttBroker, 1883);
  client.setCallback(callback);
  mqttConnectFlag = true;
}

void loop() {
  // unsigned long currentMillis = millis();

  pox.update();

  // Publish every 1 seconds
  if (millis() - lastPublishTime2 >= 2000) {
    heartRate = pox.getHeartRate();
    spo2 = pox.getSpO2();

    // Serial.print("Heart rate:");
    // Serial.print(heartRate);
    // Serial.print("bpm / SpO2:");
    // Serial.print(spo2);
    // Serial.println("%");

    if ((spo2 > 0) || (heartRate > 0)) {
      if (mqttConnectFlag == true) {
        // Prepare and publish the payload
        sprintf(payload, "%s", "");  //Cleans the payload
        sprintf(topic, "%s", "");

        sprintf(topic, "/v1.6/devices/%s", DEVICE_LABEL);
        sprintf(payload, "{\"heartRate\": {\"value\": %d}, \"spo2\": {\"value\": %d}}", heartRate, spo2);

        // Serial.print("Publishing to topic: ");
        // Serial.println(topic);
        // Serial.print("Payload: ");
        Serial.println(payload);

        client.publish(topic, payload);
      }
    }
    lastPublishTime2 = millis();
  }

#ifdef USE_WIFI_MANAGER
  wifiManager.process();
#endif
  adxl.readAccel(&accX, &accY, &accZ);

  // // Publish every 2 seconds
  if (millis() - lastPublishTime1 >= 1000) {

    // Prepare and publish the payload
    sprintf(payload, "%s", "");  //Cleans the payload
    sprintf(topic, "%s", "");

    sprintf(topic, "/v1.6/devices/%s", DEVICE_LABEL);
    //   //    sprintf(payload,
    //   //            "{\"accX\": {\"value\": %d}, \"accY\": {\"value\": %d}, \"accZ\": {\"value\": %d}, "
    //   //            "\"pressure\": {\"value\": %d}, \"altitude\": {\"value\": %d}, "
    //   //            "\"heartRate\": {\"value\": %d}, \"spo2\": {\"value\": %d}, \"fallDetect\": {\"value\": %d}}",
    //   //            accX, accY, accZ, pressure, altitude, heartRate, spo2, fallDetected);

    sprintf(payload, "{\"accX\": {\"value\": %d}, \"accY\": {\"value\": %d}, \"accZ\": {\"value\": %d}}", accX, accY, accZ);

    // Serial.print("Publishing to topic: ");
    Serial.println(topic);
    //   Serial.print("Payload: ");
    //   Serial.println(payload);

    client.publish(topic, payload);
    lastPublishTime1 = millis();
  }

  // // Publish every 4 seconds
  // if (millis() - lastPublishTime3 >= 2500) {

  //   pressure = bmp.readPressure();
  //   altitude = bmp.readAltitude();

  //   if ((pressure > 0) || (altitude > 0)) {
  //     // Prepare and publish the payload
  //     sprintf(payload, "%s", "");  //Cleans the payload
  //     sprintf(topic, "%s", "");

  //     sprintf(topic, "/v1.6/devices/%s", DEVICE_LABEL);
  //     sprintf(payload, "{\"pressure\": {\"value\": %d}, \"altitude\": {\"value\": %d}}", pressure, altitude);
  //     // Serial.print("Publishing to topic: ");
  //     // Serial.println(topic);
  //     // Serial.print("Payload: ");
  //     Serial.println(payload);

  //     client.publish(topic, payload);
  //     lastPublishTime3 = millis();
  //   }
  // }

  // if (!client.connected()) {
  //   reconnect();
  // }
  if (!client.connected()) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastReconnectAttempt >= reconnectInterval) {
      lastReconnectAttempt = currentMillis;  // Update the last reconnect attempt time
      reconnect();                           // Attempt to reconnect
    }
  }
  // else {
  //   client.loop();  // Handle MQTT communication when connected
  // }
  ADXL_ISR();
  client.loop();

  // }
}

/********************* ISR *********************/
/* Look for Interrupts and Triggered Action    */
void ADXL_ISR() {

  // getInterruptSource clears all triggered actions after returning value
  // Do not call again until you need to recheck for triggered actions
  byte interrupts = adxl.getInterruptSource();

  // Free Fall Detection
  if (adxl.triggered(interrupts, ADXL345_FREE_FALL)) {
    Serial.println("*** FREE FALL ***");
    // sendMail();
    fallDetected = true;
  } else {
    fallDetected = false;
  }

  // Inactivity
  if (adxl.triggered(interrupts, ADXL345_INACTIVITY)) {
    //    Serial.println("*** INACTIVITY ***");
    fallDetected = false;
    //add code here to do when inactivity is sensed
  }

  // Activity
  if (adxl.triggered(interrupts, ADXL345_ACTIVITY)) {
    //    Serial.println("*** ACTIVITY ***");
    //add code here to do when activity is sensed
  }

  // Double Tap Detection
  if (adxl.triggered(interrupts, ADXL345_DOUBLE_TAP)) {
    //    Serial.println("*** DOUBLE TAP ***");
    //add code here to do when a 2X tap is sensed
  }

  // Tap Detection
  if (adxl.triggered(interrupts, ADXL345_SINGLE_TAP)) {
    //    Serial.println("*** TAP ***");
    //add code here to do when a tap is sensed
  }
}

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status) {
  /* Print the current status */
  Serial.println(status.info());

  /* Print the sending result */
  if (status.success()) {
    // MailClient.printf used in the examples is for format printing via debug Serial port
    // that works for all supported Arduino platform SDKs e.g. SAMD, ESP32 and ESP8266.
    // In ESP8266 and ESP32, you can use Serial.printf directly.

    Serial.println("----------------");
    MailClient.printf("Message sent success: %d\n", status.completedCount());
    MailClient.printf("Message sent failed: %d\n", status.failedCount());
    Serial.println("----------------\n");

    for (size_t i = 0; i < smtp.sendingResult.size(); i++) {
      /* Get the result item */
      SMTP_Result result = smtp.sendingResult.getItem(i);

      // In case, ESP32, ESP8266 and SAMD device, the timestamp get from result.timestamp should be valid if
      // your device time was synched with NTP server.
      // Other devices may show invalid timestamp as the device time was not set i.e. it will show Jan 1, 1970.
      // You can call smtp.setSystemTime(xxx) to set device time manually. Where xxx is timestamp (seconds since Jan 1, 1970)

      // MailClient.printf("Message No: %d\n", i + 1);
      // MailClient.printf("Status: %s\n", result.completed ? "success" : "failed");
      // MailClient.printf("Date/Time: %s\n", MailClient.Time.getDateTimeString(result.timestamp, "%B %d, %Y %H:%M:%S").c_str());
      // MailClient.printf("Recipient: %s\n", result.recipients.c_str());
      // MailClient.printf("Subject: %s\n", result.subject.c_str());
    }
    // Serial.println("----------------\n");

    // You need to clear sending result as the memory usage will grow up.
    smtp.sendingResult.clear();
  }
}
