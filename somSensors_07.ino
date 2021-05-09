#define VERSIO " v0.7 "

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define LLINDAR_VERD    800
#define LLINDAR_VERMELL 1000
#define INTERVAL_MISSATGES_MQTT_EN_SEGONS 5
#define LLINDAR_MINIM_RECALIBRACIO 400
#define MINIM_TEMPS_CALIBRACIO 3600000

#include "Free_Fonts.h" // Include the header file attached to this sketch

#include "SPI.h"
#include "TFT_eSPI.h"

#define USING_WIFI

#include "MHZ19.h"
#include <SoftwareSerial.h>                                // Remove if using HardwareSerial or Arduino package without SoftwareSerial support

#define SW3 34
#define BT_IO0 0
#define LED_W 27
#define RGB_R 17
#define RGB_G 16
#define RGB_B 25
#define LDR 36
#define I2C_SDA 21
#define I2C_SCL 22
#define RX_PIN 35 /*10 */                                         // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 19 /*11 */                                         // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600

MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);
unsigned long myMHZ19Timer = 0;
bool bAutoCal_CO2 = false,bAutoCalibrationEnabled;
int CO2 = 400;

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
bool bHiHaBme680;

int   humidity_score, gas_score;
int   getgasreference_count = 0;
int   gas_lower_limit = 10000;  // Bad air quality limit
int   gas_upper_limit = 300000; // Good air quality limit
float hum_weighting = 0.25; // so hum effect is 25% of the total air quality score
float gas_weighting = 0.75; // so gas effect is 75% of the total air quality score
float gas_reference = 2500;
float hum_reference = 40;
double lfT, lfP , lfRH;
float air_quality_score;
bool bPrimeraLectura, bMqttsConnectat;

enum {hazardous, veryUnhealthy, unhealthy, unhealthySensitiveGroups, moderate, good, noReadings} enState;
String sJson;

bool bLedBlancEnces = false;

#ifdef USING_WIFI
#include <WiFi.h>
#include "IoT-02_wifiMng.h"
#include "IoT-02_mqttCredentials.h"
#include "IoT-02_mqttTopics.h"
#include <PubSubClient.h>
#include <ESPmDNS.h>
WiFiClientSecure espClient;
PubSubClient client(espClient);
#define MAC_SIZE 15
char sMac[MAC_SIZE];
String sIP;
bool bConsultaWiFi = false;
bool bMacIpWifiConsultada = false;
#endif         /* USING_WIFI */

#define MINIMUM_DELAY_ALLOWING_MULTITASKING 50
SemaphoreHandle_t xMutex;

// Use hardware SPI
TFT_eSPI tft = TFT_eSPI();

unsigned long drawTime = 0;

void setupDisplay() {
  tft.begin();
  tft.setRotation(1);
  tft.setTextDatum(MC_DATUM);
  // Set text colour to orange with black background
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);            // Clear screen
  tft.setFreeFont(FF31);                 // Select the font
  tft.drawString("SomSensors", 160, 100, GFXFF);// Print the string name of the font
  tft.setFreeFont(FF29);                 // Select the font
  tft.drawString(VERSIO, 160, 140, GFXFF);// Print the string name of the font
}

void vPresentaPantalla(int nCO2, double lfT, int nIAQ) {
  String szCO2 = "CO2: " + String(nCO2) + " ppm";
  String szT = "Temperatura sensor: " + String(lfT) + " ºC";
  String szIAQ = "IAQ: " + String(nIAQ) + " ";
  static int nPrevState = 0, nLastCO2 = nCO2, nLastIAQ = nIAQ;
  static double lfLastT = lfT;
  static bool bPrimerCop = true;

  if (nCO2 < LLINDAR_VERD) {
    if (bPrimerCop || bMacIpWifiConsultada || (nPrevState != 0) || (nLastIAQ != nIAQ)) {
      nPrevState = 0;
      bPrimerCop = true;
      nLastIAQ = nIAQ;
      tft.setTextColor(TFT_BLUE, TFT_GREEN);
      tft.fillScreen(TFT_GREEN);            // Clear screen
      tft.setFreeFont(FF31);                 // Select the font
      digitalWrite(RGB_R, LOW);
      digitalWrite(RGB_G, HIGH);
      digitalWrite(RGB_B, LOW);
    }
    if(nCO2 < LLINDAR_MINIM_RECALIBRACIO){
      bAutoCal_CO2 = true;
    }
  } else {
    if (nCO2 < LLINDAR_VERMELL) {
      if (bPrimerCop || bMacIpWifiConsultada || (nPrevState != 1) || (nLastIAQ != nIAQ)) {
        nPrevState = 1;
        bPrimerCop = true;
        nLastIAQ = nIAQ;
        tft.setTextColor(TFT_BLUE, TFT_CYAN);
        tft.fillScreen(TFT_CYAN);            // Clear screen
        tft.setFreeFont(FF31);                 // Select the font
        digitalWrite(RGB_R, LOW);
        digitalWrite(RGB_G, LOW);
        digitalWrite(RGB_B, HIGH);
      }
    } else {
      if (bPrimerCop || bMacIpWifiConsultada || (nPrevState != 2) || (nLastIAQ != nIAQ)) {
        nPrevState = 2;
        bPrimerCop = true;
        nLastIAQ = nIAQ;
        tft.setTextColor(TFT_YELLOW, TFT_RED);
        tft.fillScreen(TFT_RED);            // Clear screen
        tft.setFreeFont(FF31);                 // Select the font
        digitalWrite(RGB_R, HIGH);
        digitalWrite(RGB_G, LOW);
        digitalWrite(RGB_B, LOW);
      }
    }
  }
  if (bPrimerCop || (nLastCO2 != nCO2)) {
    nLastCO2 = nCO2;
    tft.drawString(szCO2.c_str(), 160, 60, GFXFF);// Print the string name of the font
    if (bHiHaBme680)
      tft.drawString(szIAQ.c_str(), 160, 180, GFXFF);// Print the string name of the font
  }
  if (bPrimerCop || (lfLastT != lfT)) {
    lfLastT = lfT;
    tft.setFreeFont(FF29);                 // Select the font
    tft.drawString(szT.c_str(), 160, 120, GFXFF);// Print the string name of the font
    tft.setFreeFont(FF31);
  }
  bPrimerCop = false;
  bMacIpWifiConsultada = false;
}

void vPresentaMacIp(String sMac, String sIP) {
  String szMac;
  String szIP;

  if (WiFi.status() == WL_CONNECTED) {
    szMac = "MAC: " + sMac;
    szIP = "IP: " + sIP;
  } else {
    szMac = "El sistema funciona";
    szIP = "sense WiFi";
  }


  tft.setTextColor(TFT_BLUE, TFT_GREEN);
  tft.fillScreen(TFT_GREEN);            // Clear screen
  tft.setFreeFont(FF30);                 // Select the font
  tft.drawString(szMac.c_str(), 160, 60, GFXFF);// Print the string name of the font
  tft.drawString(szIP.c_str(), 160, 120, GFXFF);// Print the string name of the font
  tft.drawString(sDarreraSsid().c_str(), 160, 180, GFXFF);// Nom de la xarxa
  tft.setFreeFont(FF31);
  bMacIpWifiConsultada = true;
}

void vPantallaHiHaWiFi() {
  tft.setTextColor(TFT_BLUE, TFT_GREEN);
  tft.fillScreen(TFT_GREEN);            // Clear screen
  tft.setFreeFont(FF31);                 // Select the font
  tft.drawString("Hi ha WiFi!", 160, 100, GFXFF);// Print the string name of the font
  bMacIpWifiConsultada = true;
}

//Starting BME680 functions {
void GetGasReference() {
  // Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
  //Serial.println("Getting a new gas reference value");
  int readings = 10;
  if (xMutex == NULL)
    xMutex = xSemaphoreCreateMutex();

  for (int i = 1; i <= readings; i++) { // read gas for 10 x 0.150mS = 1.5secs
    xSemaphoreTake(xMutex, portMAX_DELAY);
    gas_reference += bme.readGas();
    xSemaphoreGive(xMutex);
  }
  gas_reference = gas_reference / readings;
  //Serial.println("Gas Reference = "+String(gas_reference,3));
}

String CalculateIAQ(int score) {
  String IAQ_text = "air quality is ";
  score = (100 - score) * 5;
  if      (score >= 301)                  {
    IAQ_text += "Hazardous";
    enState = hazardous;
  }
  else if (score >= 201 && score <= 300 ) {
    IAQ_text += "Very Unhealthy";
    enState = veryUnhealthy;
  }
  else if (score >= 176 && score <= 200 ) {
    IAQ_text += "Unhealthy";
    enState = unhealthy;
  }
  else if (score >= 151 && score <= 175 ) {
    IAQ_text += "Unhealthy for Sensitive Groups";
    enState = unhealthySensitiveGroups;
  }
  else if (score >=  51 && score <= 150 ) {
    IAQ_text += "Moderate";
    enState = moderate;
  }
  else if (score >=  00 && score <=  50 ) {
    IAQ_text += "Good";
    enState = good;
  }
  //Serial.print("IAQ Score = " + String(score) + ", ");
  return IAQ_text;
}

int GetHumidityScore() {  //Calculate humidity contribution to IAQ index
  if (xMutex == NULL)
    xMutex = xSemaphoreCreateMutex();
  xSemaphoreTake(xMutex, portMAX_DELAY);
  float current_humidity = bme.readHumidity();
  xSemaphoreGive(xMutex);

  if (current_humidity >= 38 && current_humidity <= 42) // Humidity +/-5% around optimum
    humidity_score = 0.25 * 100;
  else
  { // Humidity is sub-optimal
    if (current_humidity < 38)
      humidity_score = 0.25 / hum_reference * current_humidity * 100;
    else
    {
      humidity_score = ((-0.25 / (100 - hum_reference) * current_humidity) + 0.416666) * 100;
    }
  }
  return humidity_score;
}

int GetGasScore() {
  //Calculate gas contribution to IAQ index
  gas_score = (0.75 / (gas_upper_limit - gas_lower_limit) * gas_reference - (gas_lower_limit * (0.75 / (gas_upper_limit - gas_lower_limit)))) * 100.00;
  if (gas_score > 75) gas_score = 75; // Sometimes gas readings can go outside of expected scale maximum
  if (gas_score <  0) gas_score = 0;  // Sometimes gas readings can go outside of expected scale minimum
  return gas_score;
}

bool bSetupBme680() {
  xMutex = NULL;
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    //vScreen24pixelText(0, 20, "NO BME680!");
    return false;
  } else {
    //vScreen24pixelText(0, 20, "Qualit AIRE");
    //vPantallaInicial(VERSIO);
    Serial.println("BME680 trobat!");
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms
  // Now run the sensor to normalise the readings, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
  // The sensor takes ~30-mins to fully stabilise
  GetGasReference();

  return true;
}

// } BME680 functions

boolean bPressedButton(int nWhichOne) {
  if (digitalRead(nWhichOne))
    return false;
  return true;
}

#ifdef USING_WIFI
void vConnectingMqttsTask( void *pvParameters );


String ip2Str(IPAddress ip) {
  String s = "";
  for (int i = 0; i < 4; i++) {
    s += i  ? "." + String(ip[i]) : String(ip[i]);
  }
  return s;
}

void receivedCallback(char* topic, byte* payload, unsigned int length) {
  float fTc, fRH, fP, fAlt;

  String szTopic = String(topic), szPayload = "";
  Serial.print("Topic: ");
  Serial.println(topic);

  Serial.print("payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    szPayload += (char)payload[i];
  }
  Serial.println();
  //Serial.print("Topic: "); Serial.println(szTopic);
  if (szTopic == TOPIC_REQUEST_MAC) {
    if (bPressedButton(BT_IO0)) {
      //szGetMac().toCharArray(sMac, MAC_SIZE);
      Serial.print(TOPIC_MAC); Serial.print(" : "); Serial.println(sMac);
      client.publish(TOPIC_MAC, sMac);
    }
  }

  if (szTopic == String("/" + String(sMac) + TOPIC_REQUEST_CAL).c_str()) {
    bAutoCal_CO2 = true;
    Serial.println("Autocalibració del CO2");
    //myMHZ19.autoCalibration();
  }

  if (szTopic == String("/" + String(sMac) + TOPIC_CHECK_WIFI).c_str()) {
    bConsultaWiFi = true;
    Serial.println("Rebut) Hi ha Wifi");
    //vPantallaHiHaWiFi();
  }

  if (szTopic == String("/" + String(sMac) + TOPIC_LED_W).c_str()) {
    bLedBlancEnces = true;
    Serial.println("Rebut) Petició d'encendre el led blanc");
    //vPantallaHiHaWiFi();
  }

}


void mqttconnect() {
  const TickType_t xDelay = 5000 / portTICK_PERIOD_MS; // 5 seconds

  /* Loop until reconnected */
  while (!client.connected()) {
    Serial.print("MQTT connecting ...");
    /* client ID */
    String clientId = "IoT-02_" + String(sMac); // <-------   Unique name in every device
    /* connect now */
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      //if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      /* subscribe topic */
      client.subscribe(TOPIC_REQUEST_MAC); // <-------   Subscription to MQTT(S) topic
      client.subscribe(String("/" + String(sMac) + TOPIC_LED_W).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_LED_R).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_LED_Y).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_LED_G).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_LATCHING_RELAY).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_LATCHING_RELAY_STATE).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_LDR).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_T).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_RH).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_P).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_ALT).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_SMALL_TEXT).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_MEDIUM_TEXT).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_BIG_TEXT).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_CAL).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_CHECK_WIFI).c_str());
    } else {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      //delay(5000);
      vTaskDelay(xDelay);
    }
  }
}

void vSetupMqtt() {
  /* set SSL/TLS certificate */
  espClient.setCACert(ca_cert);
  /* configure the MQTT server with IPaddress and port */
  client.setServer(mqtt_server, mqtt_port);
  /* this receivedCallback function will be invoked
    when client received subscribed topic */
  client.setCallback(receivedCallback);
}

void vReconnectWifiMqtt() {
  vSetupWifi();

  szGetMac().toCharArray(sMac, MAC_SIZE);
  /*
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  */
  vSetupMqtt();
  mqttconnect();

}

void vConnectingMqttsTask(void *parameter) {

  //Serial.println("0) vConnectingMqttsTask");

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      //Serial.println("1) vConnectingMqttsTask");
      bMqttsConnectat = false;
      sIP = "0.0.0.0";
      vReconnectWifiMqtt();
      if (WiFi.status() == WL_CONNECTED) {
        sIP = ip2Str(WiFi.localIP());
      }
      Serial.println("IP address: ");
      Serial.println(sIP);
    } else {
      if (client.connected()) {
        bMqttsConnectat = true;
        client.loop();
      } else {
        bMqttsConnectat = false;
        mqttconnect();
      }
    }

    /*
      if (WiFi.status() == WL_CONNECTED) {
       sIP = ip2Str(WiFi.localIP());
       szGetMac().toCharArray(sMac, MAC_SIZE);
      }
    */

    /*
      if (!client.connected())
      vReconnectWifiMqtt();
      else
      client.loop();
    */
    vTaskDelay(MINIMUM_DELAY_ALLOWING_MULTITASKING);
  }
  vTaskDelete(NULL); // There is an infinite loop before. This line will never be reached.
}

#endif         /* USING_WIFI */


void TaskSw3LedW(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  pinMode(SW3, INPUT);
  pinMode(LED_W, OUTPUT);

  for (;;) {
    // digitalWrite(LED_W, !digitalRead(SW3));
    if (bLedBlancEnces) {
      digitalWrite(LED_W, HIGH);
      vTaskDelay(1000);
      digitalWrite(LED_W, LOW);
      bLedBlancEnces = false;
    }
    vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
    /*
      digitalWrite(LED_W, HIGH);
      vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
      digitalWrite(LED_W, LOW);
      vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    */
  }
}

void setup(void) {
  Serial.begin(115200);

  pinMode(SW3, INPUT);
  pinMode(LED_W, OUTPUT);
  pinMode(RGB_R, OUTPUT);
  pinMode(RGB_G, OUTPUT);
  pinMode(RGB_B, OUTPUT);

  Serial.println(__FILE__);
  bMqttsConnectat = false;
  bPrimeraLectura = false;
  enState = noReadings;
  setupDisplay();
  delay(1000);

  bHiHaBme680 = bSetupBme680();
  (bHiHaBme680) ? Serial.println("El sensor VOC és connectat!") : Serial.println("No es detecta el sensor VOC");

  mySerial.begin(BAUDRATE);
  myMHZ19.begin(mySerial);
  myMHZ19.autoCalibration(true); 
  bAutoCalibrationEnabled = true;

  xTaskCreatePinnedToCore(
    TaskSw3LedW
    ,  "TaskSw3LedW"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL
    ,  ARDUINO_RUNNING_CORE);

#ifdef USING_WIFI
  if (digitalRead(SW3)) {
    xTaskCreatePinnedToCore(
      vConnectingMqttsTask,    /* Task function. */
      "Connecting MQTTS Task", /* name of task. */
      //10000,                   /* Stack size of task */
      8192,  /* Stack size of task */
      NULL,  /* parameter of the task */
      2,     /* priority of the task */
      NULL,
      ARDUINO_RUNNING_CORE);
  }
#endif         /* USING_WIFI */
}

void loop() {
  static unsigned long int ulLastMillis = millis(), n = 0;
  unsigned long int ulNow = millis();
  unsigned long int ulDif = ulNow - ulLastMillis;

  static int nCO2 = 400; // Primer cop
  static int8_t nTemp = 10; // Primer cop

  int nIAQ = 0;
  double lfT, lfP, lfRH;

  if (xMutex == NULL)
    xMutex = xSemaphoreCreateMutex();

  if (bHiHaBme680) {
    xSemaphoreTake(xMutex, portMAX_DELAY);
    lfT = bme.readTemperature();
    lfP = bme.readPressure();
    lfRH = bme.readHumidity();
    xSemaphoreGive(xMutex);

    humidity_score = GetHumidityScore();
    gas_score      = GetGasScore();

    //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
    air_quality_score = humidity_score + gas_score;
    //Serial.println(" comprised of " + String(humidity_score) + "% Humidity and " + String(gas_score) + "% Gas");
    if ((getgasreference_count++) % 5 == 0) GetGasReference();
    //Serial.println(CalculateIAQ(air_quality_score));
    CalculateIAQ(air_quality_score);
    nIAQ = (100 - air_quality_score) * 5;
  }
  if (millis() - myMHZ19Timer >= 2000)  {
    //Serial.println("---------------- -");
    CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)
    if (CO2)
      nCO2 = CO2;

    Serial.print("CO2 (ppm): ");
    Serial.println(CO2);

    nTemp = myMHZ19.getTemperature();                     // Request Temperature (as Celsius)
    Serial.print("Temperature (C): ");
    Serial.println(nTemp);

    myMHZ19Timer = millis();
    if (bAutoCal_CO2) {
      bAutoCal_CO2 = false;
      if(ulNow > MINIM_TEMPS_CALIBRACIO){
        Serial.println("Fent l'autocalibració del CO2");
        myMHZ19.calibrateZero();
      }else{
        Serial.println("Per recalibrar cal esperar un mínim de 5 minuts des del darrer reset");
      }
    }
  }

  if (bHiHaBme680) {
    sJson = "{ \"LDR\": "    + String(analogRead(LDR))
            + ", \"T\": "    + String(lfT, 2)
            + ", \"RH\": "   + String(lfRH, 1)
            + ", \"P\": "    + String(lfP / 100.0F)
            + ", \"IAQ\": "  + String(nIAQ)
            + ", \"G\": "    +  String(gas_reference)
            + ", \"Gs\": "    +  String(gas_score)
            + ", \"RHs\": "    +  String(humidity_score)
            + ", \"Estat\": "    + String(int(enState))
            + ", \"CO2_ppm\": "    + String(nCO2)
            + ", \"CO2_T\": "    + String(nTemp)
            + ", \"Llengua\": "  + "\"ca\""
            + "}";
  } else {
    sJson = "{  \"LDR\": "    + String(analogRead(LDR))
            + ", \"CO2_ppm\": "    + String(nCO2)
            + ", \"CO2_T\": "    + String(nTemp)
            + ", \"Llengua\": "  + "\"ca\""
            + "}";
  }

  if (ulDif >= 1000) {
    n++;
    ulLastMillis = ulNow;
    vPresentaPantalla(nCO2, (bHiHaBme680) ? lfT : (double)nTemp, nIAQ);
    Serial.print("json: "); Serial.println(sJson);
    if (n > INTERVAL_MISSATGES_MQTT_EN_SEGONS) {
      if (bMqttsConnectat) {
        Serial.print("Tema: "); Serial.println(String("/" + String(sMac) + TOPIC_JSON_DATA).c_str());
        Serial.print("Contingut: "); Serial.println(sJson.c_str());
        client.publish( String("/" + String(sMac) + TOPIC_JSON_DATA).c_str(), sJson.c_str());
      } else {
        Serial.print("No hi ha WiFi. El json seria: "); Serial.println(sJson);
      }
      n = 0;
    }
  }

  if (!bConsultaWiFi) {
    if (digitalRead(SW3))
      //vScreenIAQ(int(enState), nIAQ, nCO2);
      vPresentaPantalla(nCO2, (bHiHaBme680) ? lfT : (double)nTemp, nIAQ);
    else
      vPresentaMacIp(sMac, sIP);
  } else {
    bConsultaWiFi = false;
    Serial.println("Pantalla) Hi ha Wifi");
    vPantallaHiHaWiFi();
  }

  bPrimeraLectura = true;
  vTaskDelay(MINIMUM_DELAY_ALLOWING_MULTITASKING);
}

#ifndef LOAD_GLCD
//ERROR_Please_enable_LOAD_GLCD_in_User_Setup
#endif

#ifndef LOAD_GFXFF
ERROR_Please_enable_LOAD_GFXFF_in_User_Setup!
#endif
