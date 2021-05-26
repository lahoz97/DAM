/**********************************
 * CODI: AirQuality_V02_Lahoz
 * Original: Jordi Binefa (SomSensors_S8)
 * Modificacions: Albert Lahoz
 * Data: 26/05/2021
 **********************************/

#define VERSIO " v0.2 (A.Lahoz) "

/*****************CONSTANTS**************/

 #define USING_MH_Z19

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
#define ONE_SECOND 1000

#ifdef USING_SENSEAIR_S8
#define MEASUREMENT_INTERVAL 5000
#endif
#ifdef USING_MH_Z19
#define MEASUREMENT_INTERVAL 2000
#endif

#include "Free_Fonts.h"

#include "SPI.h"
#include "TFT_eSPI.h"

#define USING_WIFI

#ifdef USING_MH_Z19
  #include "MHZ19.h"
#endif

#include <SoftwareSerial.h>       

#define SW3 34
#define BT_IO0 0
#define LED_W 27
#define RGB_R 17
#define RGB_G 16
#define RGB_B 25
#define LDR 36
#define I2C_SDA 21
#define I2C_SCL 22
#define RX_PIN 35                                         // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 19                                         // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600

#ifdef USING_MH_Z19
  MHZ19 myMHZ19;                                             // Constructor for library
#endif
SoftwareSerial mySerial(RX_PIN, TX_PIN);
unsigned long myCO2SensorTimer = 0;
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

#include <WiFiClient.h>
#include <WebServer.h>
#include <Update.h>

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

WebServer server(80);

/*********************************************** OTA WEB SERVER ***********************************************/
/* Style */
String style =
  "<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
  "input{background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}"
  "#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
  "#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
  "form{background:#fff;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
  ".btn{background:#3498db;color:#fff;cursor:pointer}</style>";

/* Login page */
String loginIndex =
  "<form name=loginForm>"
  "<h1>ESP32 Login</h1>"
  "<input name=userid placeholder='User ID'> "
  "<input name=pwd placeholder=Password type=Password> "
  "<input type=submit onclick=check(this.form) class=btn value=Login></form>"
  "<script>"
  "function check(form) {"
  "if(form.userid.value=='admin' && form.pwd.value=='admin')"
  "{window.open('/serverIndex')}"
  "else"
  "{alert('Error Password or Username')}"
  "}"
  "</script>" + style;

/* Server Index Page */
String serverIndex =
  "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
  "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
  "<input type='file' name='update' id='file' onchange='sub(this)' style=display:none>"
  "<label id='file-input' for='file'>   Choose file...</label>"
  "<input type='submit' class=btn value='Update'>"
  "<br><br>"
  "<div id='prg'></div>"
  "<br><div id='prgbar'><div id='bar'></div></div><br></form>"
  "<script>"
  "function sub(obj){"
  "var fileName = obj.value.split('\\\\');"
  "document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
  "};"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  "$.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "$('#bar').css('width',Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!') "
  "},"
  "error: function (a, b, c) {"
  "}"
  "});"
  "});"
  "</script>" + style;
/**************************************************************************************************************/

/******************************************PANTALLA TFT********************************************************/
void setupDisplay() {
  tft.begin();
  tft.setRotation(1);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);            //Neteja Pantalla   
  tft.setFreeFont(FF31);                
  tft.drawString("AirQuality", 160, 100, GFXFF);
  tft.setFreeFont(FF29);                 
  tft.drawString(VERSIO, 160, 140, GFXFF);
}//Setup Display

void vPresentaPantalla(int nCO2, double lfT, int nIAQ) {
  /************ Estbalim la base*******************/
  String szCO2 = "CO2: " + String(nCO2) + " ppm";
  String szT = "Temperatura sensor: " + String(lfT) + " ºC";
  String szIAQ = "IAQ: " + String(nIAQ) + " ";
  static int nPrevState = 0, nLastCO2 = nCO2, nLastIAQ = nIAQ;
  static double lfLastT = lfT;
  static bool bPrimerCop = true;
  
  /************ Printem segons color de fons*******************/
  if (nCO2 < LLINDAR_VERD) {
    if (bPrimerCop || bMacIpWifiConsultada || (nPrevState != 0) || (nLastIAQ != nIAQ)) {
      nPrevState = 0;
      bPrimerCop = true;
      nLastIAQ = nIAQ;
      tft.setTextColor(TFT_BLUE, TFT_GREEN);
      tft.fillScreen(TFT_GREEN);            //Neteja Pantalla   
      tft.setFreeFont(FF31);                 
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
        tft.fillScreen(TFT_CYAN);            //Neteja Pantalla   
        tft.setFreeFont(FF31);                 
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
        tft.fillScreen(TFT_RED);                //Neteja Pantalla        
        tft.setFreeFont(FF31);                 
        digitalWrite(RGB_R, HIGH);
        digitalWrite(RGB_G, LOW);
        digitalWrite(RGB_B, LOW);
      }
    }
  }
  if (bPrimerCop || (nLastCO2 != nCO2)) {
    nLastCO2 = nCO2;
    tft.drawString(szCO2.c_str(), 160, 60, GFXFF);
    if (bHiHaBme680)
      tft.drawString(szIAQ.c_str(), 160, 180, GFXFF);
  }
  if (bPrimerCop || (lfLastT != lfT)) {
    lfLastT = lfT;
    tft.setFreeFont(FF29);             
    tft.drawString(szT.c_str(), 160, 120, GFXFF);
    tft.setFreeFont(FF31);
  }
  bPrimerCop = false;
  bMacIpWifiConsultada = false;

  
}//Presenta Pantalla

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

}//MAC IP


void vPantallaHiHaWiFi() {
  tft.setTextColor(TFT_BLUE, TFT_GREEN);
  tft.fillScreen(TFT_GREEN);            // Clear screen
  tft.setFreeFont(FF31);                 // Select the font
  tft.drawString("Hi ha WiFi!", 160, 100, GFXFF);// Print the string name of the font
  bMacIpWifiConsultada = true;
  
}//Hi Ha Wifi




/*************************************FUNCIONS BME680***********************************************************/
void GetGasReference() {
  // Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
  int readings = 10;
  if (xMutex == NULL)
    xMutex = xSemaphoreCreateMutex();

  for (int i = 1; i <= readings; i++) {
    xSemaphoreTake(xMutex, portMAX_DELAY);
    gas_reference += bme.readGas();
    xSemaphoreGive(xMutex);
  }
  gas_reference = gas_reference / readings;
  
}//GasReference


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
  
}//IAQ 


int GetHumidityScore() {  //Puntuacio de la humitat sobre el calcul IAQ
  if (xMutex == NULL)
    xMutex = xSemaphoreCreateMutex();
  xSemaphoreTake(xMutex, portMAX_DELAY);
  float current_humidity = bme.readHumidity();
  xSemaphoreGive(xMutex);

  if (current_humidity >= 38 && current_humidity <= 42) 
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
  
}//HUMIDITY Score


int GetGasScore() {
                  //Contribucio del CO2 al IAQ
  gas_score = (0.75 / (gas_upper_limit - gas_lower_limit) * gas_reference - (gas_lower_limit * (0.75 / (gas_upper_limit - gas_lower_limit)))) * 100.00;
  if (gas_score > 75) gas_score = 75; // Sometimes gas readings can go outside of expected scale maximum
  if (gas_score <  0) gas_score = 0;  // Sometimes gas readings can go outside of expected scale minimum
  return gas_score;
}//GasScore

bool bSetupBme680() {
  xMutex = NULL;
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!bme.begin()) {
    Serial.println("No s'ha trobat BME680 Vàlid");
    return false;
  } else {
    Serial.println("BME680 trobat!");
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C for 150 ms

  //Despres de inicialitzar sensors... Llegim
  GetGasReference();

  return true;
}//Gas Setup BME

/****************************************FINAL BME680***********************************************************/



boolean bPressedButton(int nWhichOne) {
  if (digitalRead(nWhichOne))
    return false;
  return true;
}//bPressed



/****************************************TASCA MQTT***********************************************************/
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
      client.subscribe(String("/" + String(sMac) + TOPIC_REQUEST_CAL).c_str());
      client.subscribe(String("/" + String(sMac) + TOPIC_CHECK_WIFI).c_str());
    } else {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      vTaskDelay(xDelay);
    }
  }
}//MQTT connnect

void vSetupMqtt() {
  /* Especifa certificat SSL/TLS */
  espClient.setCACert(ca_cert);
  /* configura el server MQTT amb IP i port */
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(receivedCallback);
}//MQTT setup

void vReconnectWifiMqtt() {
  vSetupWifi();

  szGetMac().toCharArray(sMac, MAC_SIZE);
  
  vSetupMqtt();
  mqttconnect();

}//reconect wifi

void vSetupOtaWebServer(){
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();  
}

void vConnectingMqttsTask(void *parameter) {

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      bMqttsConnectat = false;
      sIP = "0.0.0.0";
      vReconnectWifiMqtt();
      if (WiFi.status() == WL_CONNECTED) {
        sIP = ip2Str(WiFi.localIP());
      }
      Serial.println("IP address: ");
      Serial.println(sIP);
      vSetupOtaWebServer();
    } else {
      if (client.connected()) {
        bMqttsConnectat = true;
        client.loop();
        server.handleClient(); // OTA OPEN
      } else {
        bMqttsConnectat = false;
        mqttconnect();
      }
    }

    vTaskDelay(MINIMUM_DELAY_ALLOWING_MULTITASKING);
  }
  vTaskDelete(NULL);
}

#endif         /* USING_WIFI */

/****************************************FINAL MQTT***********************************************************/



/****************************************TASCA LED RGB***********************************************************/
void TaskSw3LedW(void *pvParameters)
{
  (void) pvParameters;

  pinMode(SW3, INPUT);
  pinMode(LED_W, OUTPUT);

  for (;;) {
    if (bLedBlancEnces) {
      digitalWrite(LED_W, HIGH);
      vTaskDelay(1000);
      digitalWrite(LED_W, LOW);
      bLedBlancEnces = false;
    }
    vTaskDelay(10);
   
  }
}
/****************************************FINAL LED RGB***********************************************************/



/****************************************S8 Lectura CO2**************************************************************/
#ifdef USING_SENSEAIR_S8
const byte s8_co2[8] = {0xfe, 0x04, 0x00, 0x03, 0x00, 0x01, 0xd5, 0xc5};
const byte s8_fwver[8] = {0xfe, 0x04, 0x00, 0x1c, 0x00, 0x01, 0xe4, 0x03};
const byte s8_id_hi[8] = {0xfe, 0x04, 0x00, 0x1d, 0x00, 0x01, 0xb5, 0xc3};
const byte s8_id_lo[8] = {0xfe, 0x04, 0x00, 0x1e, 0x00, 0x01, 0x45, 0xc3};

static byte buf[10];
//uint16_t co2;

bool myread(int n) {
  int i;
  for(i = 0; i < 10;i++) buf[i] = 0;
  i = 0;
    while(mySerial.available() > 0) {
      buf[i] = mySerial.read();
      i++;
      if(i==n)
        break;
      vTaskDelay(5);
    }

  Serial.print("Bytes llegits: ");Serial.println(i);
  return (i==n);
}

// Compute the MODBUS RTU CRC
uint16_t ModRTU_CRC(byte* buf, int len){
  uint16_t crc = 0xFFFF;
  
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
  
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;  
}

uint16_t readco2() {
  uint16_t crc, got, co2;

  Serial.println("1) readco2()");
  mySerial.write(s8_co2, 8);
  Serial.println("2) readco2()");
  if(myread(7)){
    Serial.println("3) readco2()");
    co2 = (uint16_t)buf[3] * 256 + (uint16_t)buf[4];
    Serial.println("4) readco2()");
    crc = ModRTU_CRC(buf, 5);
    Serial.println("5) readco2()");
    got = (uint16_t)buf[5] + (uint16_t)buf[6] * 256;
    Serial.println("6) readco2()");
    if(crc != got) {
      Serial.print("Invalid checksum.  Expect: ");
      Serial.print(crc, HEX);
      Serial.print("  Got: ");
      Serial.println(got, HEX);
    } else {
      Serial.print("CO2: ");
      Serial.println(co2);
      // Serial.printf("%02x %02x %02x %02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
    }
  }else
    co2 = 0;
  return co2;
}

void vS8setup(){
  Serial.print("Sensor ID: ");
  mySerial.write(s8_id_hi, 8);
  myread(7);
  Serial.println(String(buf[3]) + String(buf[4]));
  mySerial.write(s8_id_lo, 8);
  myread(7);
  Serial.println(String(buf[3]) + String(buf[4]));
  Serial.println("");

  mySerial.write(s8_fwver, 8);
  myread(7);
  Serial.println("Firmware: " + String(buf[3]) + String(buf[4]));
  Serial.println();  
}

#endif
/****************************************END S8 Lectura CO2***********************************************************/



/********************************************************************************************************************************/
/********************************************************************************************************************************/
/**************************************** MAIN ARDUINO (SETUP I LOOP) ***********************************************************/

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
  delay(10);

#ifdef USING_MH_Z19
  myMHZ19.begin(mySerial);
  myMHZ19.autoCalibration(true); 
#endif
#ifdef USING_SENSEAIR_S8
  vS8setup();
#endif

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
}//end setup

/********************************************************************************************************************************/
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

    air_quality_score = humidity_score + gas_score;
    if ((getgasreference_count++) % 5 == 0) GetGasReference();
    CalculateIAQ(air_quality_score);
    nIAQ = (100 - air_quality_score) * 5;
  }
  
  if (millis() - myCO2SensorTimer >= MEASUREMENT_INTERVAL)  {
#ifdef USING_MH_Z19    

    CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)

    if (CO2)
      nCO2 = CO2;

    Serial.print("CO2 (ppm): ");
    Serial.println(CO2);
    nTemp = myMHZ19.getTemperature();                     // Request Temperature (as Celsius)
    Serial.print("Temperature (C): ");
    Serial.println(nTemp);

    if (bAutoCal_CO2) {
      bAutoCal_CO2 = false;
      if(ulNow > MINIM_TEMPS_CALIBRACIO){
        Serial.println("Fent l'autocalibració del CO2");
        myMHZ19.calibrateZero();
      }else{
        Serial.println("Per recalibrar cal esperar un mínim de 5 minuts des del darrer reset");
      }
    }
#endif
#ifdef USING_SENSEAIR_S8
    CO2 = readco2();                             // Request CO2 (as ppm)

    if (CO2)
      nCO2 = CO2;
#endif
    myCO2SensorTimer = millis();
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
#ifdef USING_MH_Z19    
           + ", \"CO2_T\": "    + String(nTemp)
#endif           
            + ", \"Llengua\": "  + "\"ca\""
            + "}";
  } else {
    sJson = "{  \"LDR\": "    + String(analogRead(LDR))
            + ", \"CO2_ppm\": "    + String(nCO2)
#ifdef USING_MH_Z19    
            + ", \"CO2_T\": "    + String(nTemp)
#endif
            + ", \"Llengua\": "  + "\"ca\""
            + "}";
  }

  if (ulDif >= ONE_SECOND) {
    n++;
    ulLastMillis = ulNow;
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
      vPresentaPantalla(nCO2, (bHiHaBme680) ? lfT : (double)nTemp, nIAQ);
    else
      vPresentaMacIp(sMac, sIP);
  } else {
    bConsultaWiFi = false;
    Serial.println("Pantalla) Hi ha Wifi");
    vPantallaHiHaWiFi();
  }

  bPrimeraLectura = true;
  Serial.print("*");
  vTaskDelay(MINIMUM_DELAY_ALLOWING_MULTITASKING);
}//END setup
/********************************************************************************************************************************/
#ifndef LOAD_GLCD
//ERROR_Please_enable_LOAD_GLCD_in_User_Setup
#endif

#ifndef LOAD_GFXFF
ERROR_Please_enable_LOAD_GFXFF_in_User_Setup!
#endif
