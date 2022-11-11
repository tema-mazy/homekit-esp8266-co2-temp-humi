/*
 * CO2 / Pressure / Temperature / Humitidty Sensor
 *
*/

#include <ArduinoOTA.h>
#include <arduino_homekit_server.h>
#include <Ticker.h>
// sensor CO2
#include <SoftwareSerial.h>
#include <Wire.h>

#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <watchdog.h>

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


#define TRIGGER_PIN D3 // reset config on button "flash"
#define LED_ESP D4     // esp onboard GPIO2
#define LED_MCU D0     // nodemcu onboard GPIO16

#define MH_Z19_RX D5
#define MH_Z19_TX D6

#define INTERVAL 30.0       // read interval, s;

volatile uint32_t next_heap_millis = 0;
volatile uint32_t buttlastSent = 0;
volatile unsigned int  s = 0; 
volatile unsigned int pressCnt = 0;
volatile float  avg_ppm  = 440;
volatile float temp, humi;
volatile bool configMode = false;
volatile bool bmeEnabled = false;


Ticker timerWifiReconnect;

SoftwareSerial co2Serial(MH_Z19_RX, MH_Z19_TX);

WiFiEventHandler wifiDisconnectHandler;
WiFiManager wifiManager;

String ssid { "ssid" };

extern "C" homekit_characteristic_t cha_name;
extern "C" homekit_characteristic_t cha_sn;
extern "C" homekit_characteristic_t cha_temperature;
extern "C" homekit_characteristic_t cha_humidity;
extern "C" homekit_characteristic_t cha_co2;
extern "C" homekit_characteristic_t cha_co2_alert;
extern "C" homekit_server_config_t config;


void bl() {
  digitalWrite(LED_MCU, !digitalRead(LED_MCU));
}

void ledOff() {
    digitalWrite(LED_MCU, HIGH);
    digitalWrite(LED_ESP, HIGH);
}

int readMH() {
    
    byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    byte response[9]; // for answer
    
    watchdog_disable_all();
    
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
    watchdog_enable_all();
}


void factoryReset() {
    Serial.println(F("@@@ Resetting to factory settings"));
    wifiManager.resetSettings();
    homekit_storage_reset();
    SPIFFS.format();
    ESP.reset();
}

void _iototasetup() {
  ArduinoOTA.setHostname(ssid.c_str());
  ArduinoOTA.setPassword(OTAKEY);
  ArduinoOTA.onStart([]() { digitalWrite(LED_MCU, LOW); });
  ArduinoOTA.onEnd([]() {
    digitalWrite(D0, LOW);
    for (int i = 0; i < 20; i++)
       {
         digitalWrite(D0, HIGH);
         delay(i * 2);
         digitalWrite(D0, LOW);
         delay(i * 2);
       }
       digitalWrite(D0, HIGH);
       ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    digitalWrite(D0, total % 1);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("### Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
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
    cha_co2_alert.value.bool_value = avg_ppm > ALERT;
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
    Serial.println(F("### Disconnected from Wi-Fi."));
    delay(100);
    WiFi.begin();
}
void setupWiFi() {
    wifiManager.setTimeout(60);
    wifiManager.setDebugOutput(false);
    if (!wifiManager.autoConnect(ssid.c_str())) {
        Serial.println(F("### failed to connect and hit timeout"));
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

void IRAM_ATTR button_ISR() {
    unsigned long edelay = millis() - buttlastSent;
    if (edelay > 3000) { pressCnt = 0; }
    if (edelay >= 30) { // remove jitter
//	     Serial.println(F(" ** Button pressed "));
	     buttlastSent = millis();
	     pressCnt++;
    }

    if (pressCnt > 10) {
      pressCnt = 0;
      factoryReset();
    }

}

void setup() {
    Serial.begin(9600);
    Serial.setRxBufferSize(32);
    Serial.setDebugOutput(false);    
    pinMode(TRIGGER_PIN, INPUT);
    
    pinMode(LED_ESP,OUTPUT);
    pinMode(LED_MCU,OUTPUT);

    co2Serial.begin(9600);
    delay(100);

    ssid = "MIoT-CO2-"+String(ESP.getChipId(),16);
    LOG_D("\*** %s booting", ssid.c_str());


    Serial.println(F(">>> Press 10 times to reset settings "));
    Serial.println(F(">>> Button attach "));
    attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), button_ISR , RISING);

    Serial.println(F(">>> WiFi setup "));
    setupWiFi();
    Serial.println(F(">>> OTA setup "));
    _iototasetup();
    Serial.print(F(">>> Connecting "));
    
    for (int i=0;i<10000; i++){
        if(!WiFi.isConnected()) { 
            bl();
            Serial.print(".");
            delay(500);
            
        }
        else break;
    }
    Serial.println(F(""));
    ledOff();
    
    cha_name.value = HOMEKIT_STRING_CPP(&ssid[0]);
    cha_sn.value = HOMEKIT_STRING_CPP(&ssid[0]);
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
    Serial.println(F("*** Wunderwaffel setup finished"));
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
    
    ArduinoOTA.handle();
    yield();  
}


