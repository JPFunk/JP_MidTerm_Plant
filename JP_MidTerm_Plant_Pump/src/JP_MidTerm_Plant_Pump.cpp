/* 
 * Project JP_MidTerm_Plant_Pump
 * Author: JP Funk
 * Date: 02/23/2024
 * Empty Cup Value: 3542
 * Submerged H2O Value: 1703
 * Dry Soil Value: 3571
 * Wet Soil Value: 1727
 * 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */
// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_SPARK.h"
#include "credentials.h"
#include "IoTClassroom_CNM.h"
#include "IoTTimer.h"

// Capacaitive Soil Sensor
const int SOILSENSOR =A5;
int soilVal; // common usage soil value
int dryVal = 3671; // dry soil value
int wetVal = 1727; // wet soil value
// Water Pump
void pumpOn (int waterPumpPin);
const int PUMPDELAY = (1000);
// LED for Dashboard Button will be used for Plant water pump
const int LEDPIN = D7; // sharing LED pin with Pump
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
IoTTimer waterPumpTimer;
IoTTimer sensorTimer;
int timerTime;

Adafruit_SSD1306 display(OLED_RESET);
// Date and Time String
String DateTime, TimeOnly; // String variable for Date and Time

TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_Publish soilFeed = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME"/feeds/soilFeed");
Adafruit_MQTT_Subscribe buttonFeed = Adafruit_MQTT_Subscribe(&mqtt,AIO_USERNAME"/feeds/buttononoff"); 
// Let Device OS manage the connection to the Particle Cloud
void MQTT_connect();
bool MQTT_ping();

SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_INFO);
// setup() runs once, when the device is first turned on
void setup() {
  // Put initialization like pinMode and begin functions here
Serial.begin(9600);
waitFor(Serial.isConnected,10000);
WiFi.on();
WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
// Setup MQTT subscription
mqtt.subscribe(&buttonFeed);
// Millis Timer set up
waterPumpTimer.startTimer (60000);
sensorTimer.startTimer (60000); 
// Button
pinMode(BUTTONPIN, INPUT);
// LED and Pump
pinMode (LEDPIN, OUTPUT);
//Soil value
soilVal = wetVal;
// Particle Time
Time.zone (-7); // MST = -7, MDT = -6
Particle.syncTime (); // Sync time with Particle Cloud
pinMode(SOILSENSOR, INPUT); // Soil Sensor
display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // OLED Display Functions
display.display();
delay(2000);
rot = 1;
display.clearDisplay();
display.setRotation(rot);
display.setTextSize(1);
display.setTextColor(WHITE);
display.setCursor(0,0);
display.display();
}
// loop() runs over and over again, as quickly as it can execute.
void loop() {
MQTT_connect();
MQTT_ping();
  // OLED display functions
rot = 1;
display.clearDisplay(); // Date Time functions
display.setRotation(rot);
DateTime =Time.timeStr(); // Current Date and Time from Particle Time class
TimeOnly =DateTime.substring (11,19); // Extract the Time from the DateTime String
Serial.printf("Date and time is %s\n",DateTime.c_str());
Serial.printf("Time is %s\n",TimeOnly.c_str());
delay(10000);
//display.printf("Date, Time\n %s\n",DateTime.c_str());
display.printf("Time:%s\n",TimeOnly.c_str());
soilVal =analogRead(SOILSENSOR);
display.setCursor(0,0);
display.printf("SoilSensor%i\n",soilVal);
Serial.printf("SoilSensor%i\n",soilVal);
display.display();
delay(10000);

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
  if (waterPumpTimer.isTimerReady()) {
    soilVal = analogRead (SOILSENSOR);
    waterPumpTimer.startTimer (60000);
  }
  if ((soilVal < wetVal) || buttonState1 || subValue ) {
    pumpOn (LEDPIN);
    soilVal= wetVal;
    buttonState1 = !buttonState1;
    subValue = 0 ;
  }
} // End of Void Loop

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
  delay (PUMPDELAY);
  digitalWrite(waterPumpPin, LOW);
}