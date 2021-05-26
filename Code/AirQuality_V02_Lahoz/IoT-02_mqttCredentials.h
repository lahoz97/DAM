#ifndef IOT_02_MQTT_CREDENTIALS
#define IOT_02_MQTT_CREDENTIALS

#define NODE_UNIQUE_NAME szOwnMac /* Directly embedded in the code*/

const char* mqtt_server = "iot.yourbroker.com"; // Your MQTTS broker
const int mqtt_port = 8883; 
const char* mqtt_user = "Usr";
const char* mqtt_password = "YourPASSWD";

const char* ca_cert = \ 
                        "-----BEGIN CERTIFICATE-----\n" \
                        
                        "-----END CERTIFICATE-----\n";

