/* 
 * Project JP_MidTerm_Plant project "Space BioSphere"
 * Author: JP Funk
 * Date: 03/2/2024
 * Empty Cup Value: 3542
 * Submerged H2O Value: 1703
 * Dry Soil Value: 3571
 * Wet Soil Value: 1727
 */
// Include Particle Device OS APIs
#include "Particle.h"
#include "Air_quality_Sensor.h"
#include "credentials.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BME280.h"
#include "Adafruit_Sensor.h"
#include "IoTTimer.h"
#include "IoTClassroom_CNM.h"
#include "math.h"
#include "SpaceBioSphere.h"
// Capacaitive Soil Sensor
const int SOILSENSOR =A5;
int soilVal; // common usage soil value
int dryVal = 3500; // dry soil value
int wetVal = 1700; // wet soil value
// Water Pump
void pumpOn (int waterPumpPin);
const int PUMPDELAY = (1000);
// LED for Dashboard Button will be used for Plant water pump
const int LEDPIN = D7; // sharing LED pin with Pump
// Water Pump
const int PUMPIN = D6;
unsigned int last, lastTime;
float  pubValue;
int subValue;
// Button
const int BUTTONPIN = D3;
bool buttonState1;
// OLED
const int OLED_RESET=-1;
int rot;
// Millis
//int  waterPumpTimer;
IoTTimer startPumpTimer;
IoTTimer startSensorTimer;
IoTTimer startValueTimer;
IoTTimer startBMETimer;
int timerTime;
// Dust Sensor
const int DUST_SENSOR_PIN (D4);
const int SENSOR_READING_INTERVAL =30000;
int lastInterval, lowpulseOccupancy =0, last_lpo =0, duration;
float ratio =0, concentration =0;
// Air Quality Sensor
const int AQS_PIN (A2);
AirQualitySensor aqSensor(AQS_PIN);
String getAirQuality ();
// Adafruit OLED Display
Adafruit_SSD1306 display(OLED_RESET);
// Adafruit_BME280 bme code
Adafruit_BME280 bme;
const char degree = 0xf8;
const int delayTime =1000;
bool status;
int hexAddress, startime;
float tempC, pressPA, humidRH, tempF, inHG; 
// Date and Time String
String DateTime, TimeOnly; // String variable for Date and Time
// 
TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_Publish concentrationFeed = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME"/feeds/concentrationFeed");
Adafruit_MQTT_Publish qualityFeed = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME"/feeds/qualityFeed");
Adafruit_MQTT_Publish soilFeed = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME"/feeds/soilFeed");
Adafruit_MQTT_Publish tempFeed = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME"/feeds/tempFeed");
Adafruit_MQTT_Publish pressFeed = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME"/feeds/pressFeed");
Adafruit_MQTT_Publish humidFeed = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME"/feeds/humidFeed");
Adafruit_MQTT_Subscribe buttonFeed = Adafruit_MQTT_Subscribe(&mqtt,AIO_USERNAME"/feeds/buttononoff"); 
/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();
// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);
// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);

// setup() runs once, when the device is first turned on
void setup() {
  // Put initialization like pinMode and begin functions here
Serial.begin(9600);
waitFor(Serial.isConnected,10000);
 // Connect to Internet but not Particle Cloud
WiFi.on();
WiFi.connect();
  while(WiFi.connecting()) {
  Serial.printf(".");
  }
Serial.printf("\n\n");
// Setup MQTT subscription
mqtt.subscribe(&buttonFeed);
// Millis Timer set up
startPumpTimer.startTimer (600000);
//waterPumpTimer = millis();
startSensorTimer.startTimer (6000); 
startValueTimer.startTimer (6000);
startBMETimer.startTimer (6000);
// Button
pinMode(BUTTONPIN, INPUT);
// LED and Pump
pinMode (LEDPIN, OUTPUT);
pinMode (PUMPIN, OUTPUT);
//soilVal = wetVal;
soilVal = dryVal;
// Particle Time  
Time.zone (-7); // MST = -7, MDT = -6
Particle.syncTime (); // Sync time with Particle Cloud
pinMode(SOILSENSOR, INPUT); // Soil Sensor
display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // OLED Functions
display.display();
delay(2000);
// Space BioSphere Animated BMP startup Screen Sequence
rot = 2;
display.setRotation(rot);
 display.clearDisplay();
 display.drawBitmap (0, 0, SpaceBioSphere6, 128, 64, WHITE); 
 display.display();
 delay(10);
 display.clearDisplay();
 display.drawBitmap (0, 0, SpaceBioSphere5, 128, 64, WHITE); 
 display.display();
 delay(10);
 display.clearDisplay();
 display.drawBitmap (0, 0, SpaceBioSphere4, 128, 64, WHITE); 
 display.display();
 delay(10);
 display.clearDisplay();
 display.drawBitmap (0, 0, SpaceBioSphere3, 128, 64, WHITE); 
 display.display();
 delay(10);
 display.clearDisplay();
 display.drawBitmap (0, 0, SpaceBioSphere2, 128, 64, WHITE); 
 display.display();
 delay(10);
 display.clearDisplay();
 display.drawBitmap (0, 0, SpaceBioSphere, 128, 64, WHITE);
 display.display();
 delay(2000);
 // Invert Display
 display.invertDisplay(true);
 delay(1000);
 display.invertDisplay(false);
 delay(1000);
rot = 3; // OLED Display Rotation for Vertical viewing.
display.clearDisplay();
display.setRotation(rot);
display.setTextSize(1);
display.setTextColor(WHITE);
display.setCursor(0,0);

// Dust Sensor
pinMode(DUST_SENSOR_PIN, INPUT);
lastInterval = millis();
//void getDustSensorReadings(float ratio, float concentration);
// Air Quality Sensor
if (aqSensor.init()) {
  Serial.println("Air Quality Sensor ready.");
  }
    else {
      Serial.println("Air Quality Sensor ERROR!");
    }
    // initialize BME
      status = bme.begin(0x76);
      if (status==false);{
      Serial.printf("BME280 at address 0x%02X failed to start\n", 0x76); // Adafruit_BME280 bme code
      }
      startime = millis();
}
// loop() runs over and over again, as quickly as it can execute.
void loop() {
MQTT_connect();
MQTT_ping();

// OLED display functions with Time and Soil Sensor readings
  DateTime =Time.timeStr(); // Current Date and Time from Particle Time class
  TimeOnly =DateTime.substring (11,19); // Extract the Time from the DateTime String
  Serial.printf("Date and time is %s\n",DateTime.c_str());
  Serial.printf("Time is %s\n",TimeOnly.c_str());

  soilVal =analogRead(SOILSENSOR);
  //if  (millis() > startime + delayTime) {
    if (startValueTimer.isTimerReady()) {
    rot = 3;
    display.clearDisplay(); // Date Time functions
    display.setRotation(rot);
    display.setCursor(0,0);
    Serial.printf("SoilSensor%i\n",soilVal);
    display.printf("Time:\n%s\n",TimeOnly.c_str());
    display.printf("\nSoilSensor%i\n",soilVal);
    display.printf("\nLPO:%d\n", lowpulseOccupancy);
    display.printlnf("\nRatio: %f%%", ratio);
    display.printlnf("\nCncntrtion%f pcs/L", concentration);
    display.display();
    startValueTimer.startTimer (6000);
    //startime = millis();
    }
    //delay(6000);
  //}
 // BME functions with OLED display
  tempC = bme.readTemperature();
  tempF = map (tempC,0.0,100.0,32.0,212.0);
  pressPA = bme.readPressure();
  inHG = pressPA / 3386.0;
  humidRH = bme.readHumidity();
  if  (startBMETimer.isTimerReady()) {  //(millis() > startime + delayTime)
    rot = 3;
    display.clearDisplay();
    display.setRotation(rot);
    display.setCursor(0,0);
    display.printf("Temp F\n%0.2f\n", tempF);
    //display.printf("Temp %0.2f\n", tempC);
    display.printf("\nPressure\n%0.2f\n", pressPA);
    display.printf("\nHumidity\n%0.2f\n", humidRH);
    display.display();
    startBMETimer.startTimer (6000);
    // startime = millis();
    // delay(6000);
   }

// Adafruit Dashboard Button for LED/Water Pump
Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &buttonFeed) {
      subValue = atoi((char *)buttonFeed.lastread);
      Serial.printf("buttonFeed%i\n", subValue);
      }
  }
  buttonState1 = digitalRead(BUTTONPIN); //Button on Breadboard
  if (buttonState1) {
    pumpOn (LEDPIN);
    Serial.printf("Button is pressed \n");
  }
    if((millis()-lastTime > 60000)) {
      if(mqtt.Update()) {
        soilFeed.publish(soilVal);
        Serial.printf("soilFeed %i \n",soilVal); 
        } 
      lastTime = millis();
  }
// Soil Sensor Function for turning On/Off water pump
  if (startPumpTimer.isTimerReady()) {
    soilVal = analogRead (SOILSENSOR);

      if ((soilVal > wetVal)) {
        pumpOn (LEDPIN);
        pumpOn (PUMPIN);
        soilVal= wetVal;
      }
    startPumpTimer.startTimer (600000);
    //startPumpTimer.startTimer (1800000);
  }

  if (buttonState1 || subValue ) {  // ((soilVal > wetVal) || buttonState1 || subValue ) w/ soilVal= wetVal;
    pumpOn (LEDPIN);
    pumpOn (PUMPIN);
    buttonState1 = !buttonState1;
    subValue = 0 ;
  }

// Adafruit MQTT Publish functions
  Adafruit_MQTT_Publish *publish;
  // Dust Sensor readings and Display
  duration = pulseIn(DUST_SENSOR_PIN, LOW);
  lowpulseOccupancy =lowpulseOccupancy+duration;
  String quality = getAirQuality();  //Air Quality  
  if ((millis()-lastInterval)>SENSOR_READING_INTERVAL) { 
    //getDustSensorReadings();
    ratio = lowpulseOccupancy /(SENSOR_READING_INTERVAL*10.0);
    concentration = 1.1 *pow(ratio,3) -3.8 * pow(ratio,2) +520 *ratio +0.62;
    Serial.printlnf("LPO: %d", lowpulseOccupancy);
    Serial.printlnf("Ratio: %f%%", ratio);
    Serial.printlnf("Concentration; %f pcs/L", concentration);
    lowpulseOccupancy = 0;
    lastInterval = millis();

    if(mqtt.Update()) {
      concentrationFeed.publish(concentration);
      qualityFeed.publish(quality);
      //soilFeed.publish(val);
      tempFeed.publish(tempF);
      pressFeed.publish(pressPA);
      humidFeed.publish(humidRH);
      }
      Serial.printlnf("Air Quality: %s", quality.c_str());
  }
} // End Void Loop

// Air Quality Functions
String getAirQuality() {
int quality = aqSensor.slope();
String qual = "None";
  if (quality == AirQualitySensor::FORCE_SIGNAL) { 
  qual = "Danger,0"; 
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION) { 
  qual = "High Pollution,1"; 
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION) { 
  qual = "Low Pollution,2"; 
  }
  else if (quality == AirQualitySensor::FRESH_AIR) { 
  qual = "Fresh Air,3"; 
  }
  return qual;
}
// MQTT Adafruit.IO connections
void MQTT_connect() {
  int8_t ret;
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
    Serial.printf("Retrying MQTT connection in 5 seconds...\n");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}
bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;
  if ((millis()-last)>120000) {
    Serial.printf("Pinging MQTT \n");
    pingStatus = mqtt.ping();
    if(!pingStatus) {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
void pumpOn (int waterPumpPin) { // Water Pump OnOff function with serial print values
  digitalWrite(waterPumpPin, HIGH);
  Serial.printf("Pump On%i,%i\n", pumpOn, PUMPDELAY);
  rot = 3;
  display.clearDisplay();
  display.setRotation(rot);
  display.setCursor(0,0);
  display.printf("Pump On%i\n", pumpOn);
  delay (PUMPDELAY);
  digitalWrite(waterPumpPin, LOW);
  display.display();
  display.clearDisplay();
}
