#ifndef IOT_02_MQTT_CREDENTIALS
#define IOT_02_MQTT_CREDENTIALS

#define NODE_UNIQUE_NAME szOwnMac /* Directly embedded in the code*/

const char* mqtt_server = "yourbroker.com"; // Your MQTTS broker
const int mqtt_port = 0000; 
const char* mqtt_user = "your_usr";
const char* mqtt_password = "your_pswd";

const char* ca_cert = \ 
                        "-----BEGIN CERTIFICATE-----\n" \
                        "-----END CERTIFICATE-----\n";

#endif // IOT_02_MQTT_CREDENTIALS
