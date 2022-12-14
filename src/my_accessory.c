
#define ACCESSORY_NAME  ("THCSensor")
#define ACCESSORY_SN  ("MIoT:DEADBEEF")  
#define ACCESSORY_MANUFACTURER ("Mazy's Wünderwafle")
#define ACCESSORY_MODEL  ("ESP/THC")
#define HOMEKIT_SOCKET_KEEPALIVE_IDLE_SEC 5 // Instead of 180
#define HOMEKIT_SOCKET_KEEPALIVE_INTERVAL_SEC 3 //Instead of 30
#define HOMEKIT_SOCKET_KEEPALIVE_IDLE_COUNT 1 //Instead of 4

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

void my_accessory_identify(homekit_value_t _value) {}

homekit_characteristic_t cha_name = HOMEKIT_CHARACTERISTIC_(NAME, ACCESSORY_NAME);
homekit_characteristic_t cha_sn = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, ACCESSORY_SN);
homekit_characteristic_t cha_temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t cha_humidity  = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);
homekit_characteristic_t cha_co2  = HOMEKIT_CHARACTERISTIC_(CARBON_DIOXIDE_LEVEL, 0);
homekit_characteristic_t cha_co2_alert  = HOMEKIT_CHARACTERISTIC_(CARBON_DIOXIDE_DETECTED, false);


homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            &cha_name,
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, ACCESSORY_MANUFACTURER),
            &cha_sn,
            HOMEKIT_CHARACTERISTIC(MODEL, ACCESSORY_MODEL),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, my_accessory_identify),
            NULL
        }),
        HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Temperature"),
            &cha_temperature,
            NULL
        }),
        HOMEKIT_SERVICE(HUMIDITY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Humidity"),
            &cha_humidity,
            NULL
        }),
        HOMEKIT_SERVICE(CARBON_DIOXIDE_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "CO2 ppm"),
            &cha_co2,
            &cha_co2_alert,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
		.accessories = accessories,
		.password = "111-11-111"
};


