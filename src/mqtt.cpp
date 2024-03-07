#include "Arduino.h"
#include <WiFi.h>
#include "main.h"
#include <PubSubClient.h>

extern String topic, payload,  mainTopic, clientId ;
extern pvValue upv;
extern WiFiClient wifiClient;
extern PubSubClient mqttClient;

void mqttConfig(void) {
  //---------------------- Discovery topic -------------------------------
    Serial.print("ESP Board MAC Address:  ");
    Serial.println(WiFi.macAddress());
    clientId  = "isida_" + WiFi.macAddress();
    clientId .replace(":","");
    Serial.print("identifiers:  ");
    Serial.println(clientId );
    topic = "homeassistant/sensor/" + clientId  + "_temper1/config";

    Serial.println(); Serial.print("Discovery topic: "); Serial.println(topic);

    payload = "{\"name\": \"Temper1\",\"state_topic\": \"ISIDA/Node1/temper1\",\"device_class\": \"temperature\",\"value_template\": \"{{ value | round(2) }}\",\"unique_id\":\"";
    payload += clientId ;
    payload += "_temper1\",\"device\": {\"name\": \"ISIDA\",\"identifiers\": [\"" + clientId  + "\"]}}";
    // std::string payload = "{\"name\": \"Temper1\","
    //                  "\"state_topic\": \"" + std::string(mainTopic.c_str()) + "/temper1\","
    //                  "\"device_class\": \"temperature\","
    //                  "\"value_template\": \"{{ value | round(2) }}\","
    //                  "\"unique_id\":\"" + std::string(clientId .c_str()) + "_temper1\","
    //                  "\"device\": {"
    //                  "\"name\": \"ISIDA\","
    //                  "\"identifiers\": [\"" + std::string(clientId .c_str()) + "\"]"
    //                  "}}";
    
    Serial.println(payload.c_str()); Serial.println();
}

void sendMqttBroker(void) {
    topic = mainTopic + "/model";
    payload = String(upv.pv.model);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/node";
    payload = String(upv.pv.node);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/temper1";
    payload = String((double)upv.pv.pvT[0]/10,1);
    // Serial.println("MESH -> MQTT: Forward message to MQTT broker, to " + topic + " = " + payload);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/temper2";
    payload = String((double)upv.pv.pvT[1]/10,1);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/temper3";
    payload = String((double)upv.pv.pvT[2]/10,1);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/humidity";
    payload = String(upv.pv.pvRH);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/carbon";
    payload = String(upv.pv.pvCO2);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/timer";
    payload = String(upv.pv.nextTurn);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/fan";
    payload = String(upv.pv.fan);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/flap";
    payload = String(upv.pv.flap);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/power";
    payload = String(upv.pv.power);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    //------------- fuses -------------
    topic = mainTopic + "/fuses";
    payload = String(upv.pv.fuses & 0x7F);    // bit 7 -  Состояние дверей
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/door";
    if(upv.pv.fuses & 0x80) payload = "open"; else payload = "closed";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    //---------------------------------
    topic = mainTopic + "/errors";
    payload = String(upv.pv.errors);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/warning";
    payload = String(upv.pv.warning);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/set1";
    payload = String((double)upv.pv.spT[0]/10,1);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/set2";
    payload = String((double)upv.pv.spT[1]/10,1);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/set3";
    payload = String(upv.pv.spRH[1]);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    //---------- state -------------
    topic = mainTopic + "/state";
    payload = String(upv.pv.state & 0x83);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/incubation";
    if(upv.pv.state & 0x01) payload = "on"; else payload = "off";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/heater_1";
    if(upv.pv.state & 0x01) payload = "heat"; else payload = "off";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/just_turn";
    if(upv.pv.state & 0x80) payload = "turn"; else payload = "off";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/heater_2";
    if(upv.pv.state & 0x80) payload = "heat"; else payload = "off";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/humidifier/action";
    if(upv.pv.state & 0x80) payload = "off"; else payload = "humidifying";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/humidifier/state";
    if(upv.pv.state & 0x80) payload = "0"; else payload = "1";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    //------------------------------
    topic = mainTopic + "/extendmode";
    payload = String(upv.pv.extendMode);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/relaymode";
    payload = String(upv.pv.relayMode);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/programm";
    payload = String(upv.pv.programm);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}
// https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_esp8266/mqtt_esp8266.ino
void connectMqttBroker() {
    Serial.println("Attempting MQTT connection...");
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("Connected to MQTT broker!");
      mqttClient.subscribe("ISIDA/N_1/set");
      // mqttClient.subscribe("topic2");
    } else {
      Serial.print("Failed to connect to MQTT broker, MQTT_");
      switch (mqttClient.state())
      {
      case MQTT_CONNECTION_TIMEOUT:
        Serial.print("CONNECTION_TIMEOUT");
        break;
      case MQTT_CONNECTION_LOST:
        Serial.print("CONNECTION_LOST");
        break;
      case MQTT_CONNECT_FAILED:
        Serial.print("CONNECT_FAILED");
        break;
      case MQTT_DISCONNECTED:
        Serial.print("DISCONNECTED");
        break;
      case MQTT_CONNECT_BAD_PROTOCOL:
        Serial.print("CONNECT_BAD_PROTOCOL");
        break;
      case MQTT_CONNECT_BAD_CLIENT_ID:
        Serial.print("CONNECT_BAD_CLIENT_ID");
        break;
      case MQTT_CONNECT_UNAVAILABLE:
        Serial.print("CONNECT_UNAVAILABLE");
        break;
      case MQTT_CONNECT_BAD_CREDENTIALS:
        Serial.print("CONNECT_BAD_CREDENTIALS");
        break;
      case MQTT_CONNECT_UNAUTHORIZED:
        Serial.print("CONNECT_UNAUTHORIZED");
        break;
      default:
        break;
      }
      Serial.println(" Retrying in 5 seconds...");
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
