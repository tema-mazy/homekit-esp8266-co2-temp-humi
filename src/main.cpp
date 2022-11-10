/*
 * CO2 / Pressure / Temperature / Humitidty Sensor
 (c) 2019 Mazy
*/

#include <Arduino.h>
#include <arduino_homekit_server.h>
#include <Ticker.h>
// sensor CO2
#include <SoftwareSerial.h>
#include <Wire.h>

#include <ESP8266WiFi.h>
#include <WiFiManager.h>

#define LOG_D(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);
#define SENSOR_T_CORRECTION -1.9;

//#define BLINK_LEDS

#define SENSOR_HTU
#ifdef SENSOR_HTU
#include <HTU21D.h>
/*
HTU21D(resolution)

resolution:
HTU21D_RES_RH12_TEMP14 - RH: 12Bit, Temperature: 14Bit, by default
HTU21D_RES_RH8_TEMP12  - RH: 8Bit,  Temperature: 12Bit
HTU21D_RES_RH10_TEMP13 - RH: 10Bit, Temperature: 13Bit
HTU21D_RES_RH11_TEMP11 - RH: 11Bit, Temperature: 11Bit
*/
HTU21D            htu(HTU21D_RES_RH12_TEMP14);
#endif


#define TRIGGER_PIN D3 // @reset config on button "flash"
#define LED_ESP D4 // esp onboard GPIO2
#define LED_MCU D0 // nodemcu onboard GPIO16
#define LED_B   D4 //
#define LED_G   D7 //
#define LED_R   D8 //


#define MH_Z19_RX D5
#define MH_Z19_TX D6

#define INTERVAL 15.0       // read interval, s;


volatile long previousMillis = 0;

volatile  float  avg_ppm  = 440;
volatile  int  alert  = 1200;
volatile  int  s = 0; 
volatile  float temp, humi;
volatile  bool configMode = false;
volatile  bool bmeEnabled = false;

uint32_t next_heap_millis = 0;



Ticker timerWifiReconnect;


 // define MH-Z19 sensor pins
SoftwareSerial co2Serial(MH_Z19_RX, MH_Z19_TX);

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

// Network credentials
String ssid { "ssid" };

//==============================
// HomeKit setup
//==============================

extern "C" homekit_characteristic_t cha_name;
extern "C" homekit_characteristic_t cha_temperature;
extern "C" homekit_characteristic_t cha_humidity;
extern "C" homekit_characteristic_t cha_co2;
extern "C" homekit_characteristic_t cha_co2_alert;

extern "C" homekit_server_config_t config;


void bl() {
  digitalWrite(LED_ESP, !digitalRead(LED_ESP));
}
void blR() {
  digitalWrite(LED_R, !digitalRead(LED_R));
}
void blG() {
  digitalWrite(LED_G, !digitalRead(LED_G));
}


void ledOff() {
    digitalWrite(LED_MCU, HIGH);
    digitalWrite(LED_ESP, HIGH);
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_R, LOW);
}

int readMH() {
    byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    byte response[9]; // for answer
    co2Serial.write(cmd, 9); //request PPM CO2

    // The serial stream can get out of sync. The response starts with 0xff, try to resync.
    while (co2Serial.available() > 0 && (unsigned char)co2Serial.peek() != 0xFF) {
        co2Serial.read();
    }

    memset(response, 0, 9);
    co2Serial.readBytes(response, 9);

    if (response[1] != 0x86) {
        Serial.println(F("### Invalid response from co2 sensor!"));
        return -1;
    }

    byte crc = 0;
    for (int i = 1; i < 8; i++) {
        crc += response[i];
    }
    crc = 255 - crc + 1;

    if (response[8] == crc) {
        int responseHigh = (int) response[2];
        int responseLow = (int) response[3];
        int ppm = (256 * responseHigh) + responseLow;
        return ppm;
    } else {
        Serial.println(F("### CRC error!"));
        return -1;
    }
    yield();
}


void factoryReset() {
    Serial.println(F("@@@ Resetting to factory settings"));
    WiFiManager wifiManager;
    wifiManager.resetSettings();
    homekit_storage_reset();
    ESP.reset();
}


void readCO2() {
#ifdef BLINK_LEDS    
    digitalWrite(LED_ESP,LOW);
#endif
    int ppm = readMH();
    avg_ppm = (ppm+avg_ppm)/2;
    Serial.print(F("raw PPM = "));
    Serial.print(String(ppm));
    Serial.print(F(" Avg PPM = "));
    Serial.println(String(avg_ppm));
    cha_co2.value.float_value = avg_ppm;
    homekit_characteristic_notify(&cha_co2, cha_co2.value);
    cha_co2_alert.value.bool_value = avg_ppm > alert;
    homekit_characteristic_notify(&cha_co2_alert, cha_co2_alert.value);
    LOG_D("Sensor CO2 ppm: %.1f, a: %u", avg_ppm, cha_co2_alert.value.bool_value);
    digitalWrite(LED_ESP,HIGH);
}



void readTemp() {
#ifdef BLINK_LEDS    
    digitalWrite(LED_MCU,LOW);
#endif    
    temp = htu.readTemperature();
    cha_temperature.value.float_value = temp;
    homekit_characteristic_notify(&cha_temperature, cha_temperature.value);
    LOG_D("Sensor temp: %.1f", temp);
    digitalWrite(LED_MCU,HIGH);
}
void readHumi() {
#ifdef BLINK_LEDS    
    digitalWrite(LED_MCU,LOW);
#endif
    humi = htu.readHumidity();
    cha_humidity.value.float_value = humi;
    homekit_characteristic_notify(&cha_humidity, cha_humidity.value);
    LOG_D("Sensor humi: %.1f", humi);
    digitalWrite(LED_MCU,HIGH);
    yield();
}

void readSensor() {
    switch(s++) {
       case 0:
          readCO2();
          break;
       case 1:
          readTemp();
          break;
       case 2:
          readHumi();
          break;   
       default: 
          s = 0;
    }    
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
    Serial.println(F("Disconnected from Wi-Fi."));
    delay(100);
    WiFi.begin();
}
void setupWiFi() {
    WiFiManager wifiManager;
    wifiManager.setTimeout(60);

    if (!wifiManager.autoConnect(ssid.c_str())) {
        Serial.println(F("failed to connect and hit timeout"));
        configMode = true;
    }
    if (!configMode) {
        WiFi.setAutoReconnect(true);
        WiFi.begin();
        Serial.println(F(">>> WiFi connected"));
        Serial.print(F(">>> IP address: "));
        Serial.println(WiFi.localIP());
     
        wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
 
    }
    
}


void setup() {
    Serial.begin(9600);
    Serial.setRxBufferSize(32);
    Serial.setDebugOutput(false);    
    pinMode(TRIGGER_PIN, INPUT);
    pinMode(LED_ESP,OUTPUT);
    pinMode(LED_MCU,OUTPUT);
    pinMode(LED_R,OUTPUT);
    pinMode(LED_G,OUTPUT);
    pinMode(LED_B,OUTPUT);
    co2Serial.begin(9600);
    delay(100);

    ssid = "MIoT-CO2-"+String(ESP.getChipId(),16);
    LOG_D("\n@@@ %s booting", ssid.c_str());


    Serial.println(F("### Sleep for 5 sec. Press and hold  FLASH to clear settings "));
    delay(5000);
    if ( digitalRead(TRIGGER_PIN) == LOW) { //debounce
        factoryReset();
    }
 
    setupWiFi();
  
    cha_name.value = HOMEKIT_STRING_CPP(&ssid[0]);
 
    delay(500);
    arduino_homekit_setup(&config);

    
#ifdef SENSOR_HTU
    bmeEnabled = htu.begin(D2,D1); // SDA/SCL
    if (!bmeEnabled) {
        Serial.println(F("### Could not find a valid HTU21D sensor, check wiring!"));
    } else {
        Serial.print(F(">>> HTU21D sensor found. fw ver "));
        Serial.println(htu.readFirmwareVersion());
    }
#endif
    
    next_heap_millis = millis()+60*1000;
    Serial.println(F("@@@ setup finished"));
    ledOff();
}



void loop() {
    arduino_homekit_loop();
    yield(); 
    uint32_t time = millis();
    if (time > next_heap_millis) {
        next_heap_millis = time + INTERVAL*1000;
        if (WiFi.isConnected() && homekit_is_paired() ) {  
            LOG_D(">>> [% d] loop", time);
            readSensor();
        }
        INFO(">>> heap: %d, sockets: %d", ESP.getFreeHeap(), arduino_homekit_connected_clients_count());
    }
    yield();  
}


