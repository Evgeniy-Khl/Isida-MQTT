//************************************************************
// this is a simple example that uses the easyMesh library
//
// 1. blinks led once for every node on the mesh
// 2. blink cycle repeats every BLINK_PERIOD
// 3. sends a silly message to every node on the mesh at a random time between 1 and 5 seconds
// 4. prints anything it receives to Serial.print
//
//
//************************************************************
#include "Arduino.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "main.h"
#include "BluetoothSerial.h"
#include "esp_bt.h"
#include "multiserial.h"
#include "commands.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

void receivedCallback(const uint32_t &from, const String &msg);
void mqttCallback(char *topic, byte *payload, unsigned int length);
void newConnectionCallback(const uint32_t &id);
void connectMqttBroker(void);
void sendMqttBroker(void);

void receivedCallback(uint32_t from, String & msg);
void receiveUCSerial(void);
void sendBufferNow(void);
void receiveBlueTooth(void);

String sendMsg = BT_NAME;
String mainTopic = MAIN_TOPIC;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
const char* mqttServer = "192.168.1.25";    //192.168.1.25 //public.mqtthq.com
const int mqttPort = 1883;
const char* mqttUsername = "your_mqtt_username";
const char* mqttPassword = "your_mqtt_password";
const char* macaddres = "4tcd099d51f0";

uint8_t seconds=0, minuts=0, hour=0;
String topic, payload;
union {
  uint8_t data[7];
  struct {
    uint8_t ip[4];    // 4 байт ind=41;ind=42; ind=43;ind=44; IP MQTT broker [192.168.100.100]
    uint16_t port;    // 2 байт ind=45;ind=46; [1883]		 Port MQTT broker
    uint8_t group;    // 1 байт ind=47;        группа узлов MQTT broker
  } ip;
} ipport;
bool ifconfig = false;
bool ifdatacomplit = false;
// ------------------  Multiserial  ------------------------------------
BluetoothSerial SerialBT;
HardwareSerial UCSerial(1);
MultiSerial CmdSerial;

char BT_CTRL_ESCAPE_SEQUENCE[] = {'\4', '\4', '\4', '!'};
uint8_t BT_CTRL_ESCAPE_SEQUENCE_LENGTH = sizeof(BT_CTRL_ESCAPE_SEQUENCE)/sizeof(BT_CTRL_ESCAPE_SEQUENCE[0]);

double dbTemp = 199.9;

unsigned long lastSend = 0, lastSendMqtt = 0;
unsigned long blinkLed = 0;
bool isConnected = false;
bool btKeyHigh = false;
bool btReady = false;
bool escIsEnabled = false;
bool mqttsend = false;
String sendBuffer;
String commandBuffer;

int8_t escapeSequencePos = 0;
unsigned long lastEscapeSequenceChar = 0;

bool bridgeInit = false;
bool ucTx = false;
pvValue upv;
int indData = 0;
bool calc_delay = false;
bool onFlag = false;

void setup() {
  Serial.begin(115200);
  upv.pvdata[indData]=0;
  // pinMode(LED, OUTPUT);
  // ------------------  Multiserial  ------------------------------------
    pinMode(BT_KEY, INPUT_PULLDOWN);
    pinMode(PIN_WIFI_CONECTED, OUTPUT);
    digitalWrite(PIN_WIFI_CONECTED, LOW);  // WiFi NO conected!
    pinMode(PIN_CONNECTED, OUTPUT);
    digitalWrite(PIN_CONNECTED, LOW);
    pinMode(UC_NRST, INPUT);

    
    UCSerial.begin(9600, SERIAL_8N1, UC_RX, UC_TX);
    UCSerial.setRxBufferSize(1024);

    CmdSerial.addInterface(&Serial);
    CmdSerial.addInterface(&UCSerial);

    sendBuffer.reserve(MAX_SEND_BUFFER);
    commandBuffer.reserve(MAX_CMD_BUFFER);

    setupCommands();

    Serial.println("Waiting for STM32 transmission.");
    //----- Wait BT_Name -----
    indData = 0; 
    while(!ifconfig)
    {
        if(UCSerial.available()) {
            receiveUCSerial();
        }
    }
    upv.pv.cellID = 1;
    indData = 0;
    sendMsg = "ISIDA-" + String(upv.pv.cellID);
    mainTopic += String(upv.pv.cellID);
    Serial.print("New name bluetooth:"); Serial.println(sendMsg);
    Serial.print("New name topic:"); Serial.println(mainTopic);
    SerialBT.begin(sendMsg);
    CmdSerial.addInterface(&SerialBT);

    while(CmdSerial.available()) {
        CmdSerial.read();
    }
    CmdSerial.disableInterface(&SerialBT);
    CmdSerial.disableInterface(&UCSerial);

    Serial.print("escapeIsEnabled(): ");
    Serial.println(escapeIsEnabled());

    Serial.print("monitorBridgeEnabled(): ");
    Serial.println(monitorBridgeEnabled());

    Serial.print("Serial Bridge Ready: ");
    Serial.println(sendMsg);

    commandPrompt();

    digitalWrite(PIN_MONITOR, LOW);
    pinMode(PIN_MONITOR, OUTPUT);
//----------------------------------------------------------------------------------------------------
    digitalWrite(PIN_READY, HIGH);
    pinMode(PIN_READY, OUTPUT);
    Serial.println();
    Serial.println("WiFi.begin(Andrew_2023)");
    // Инициализация Wi-Fi и подключение к сети
    WiFi.begin("Andrew_2023", "graviton19630301");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
     
    Serial.print("WiFi conected IP: ");
    Serial.println(WiFi.localIP());
    // Настройка MQTT клиента
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqttCallback);
    
    // Подключение к MQTT брокеру
    Serial.println("setup()->Connecting to MQTT ...");
    connectMqttBroker();
}

void loop() {
  // digitalWrite(LED, !onFlag);
  // ------------------  Multiserial  ------------------------------------
    commandLoop();

    bool _connected = SerialBT.hasClient();

    if(isConnected != _connected) {
        isConnected = _connected;
        if(isConnected) {
            Serial.println("<Client Connected>");
        } else {
            Serial.println("<Client Disconnected>");
            unescape();
        }
        digitalWrite(PIN_CONNECTED, isConnected);
    }
    if(isConnected){                    // подключен к BT горит постоянно
        digitalWrite(PIN_READY, HIGH);
    } else if(escIsEnabled){            // в режиме Esc последовательности медлено мигает
        if(millis() - blinkLed > MAX_SEND_WAIT*20){
            blinkLed = millis();
            if(btReady){
                digitalWrite(PIN_READY, HIGH);
                btReady = false;
            } else {
                digitalWrite(PIN_READY, LOW);
                btReady = true;
            }
        }
    } else if(millis() - blinkLed > MAX_SEND_WAIT*2){// в режиме моста быстро мигает
        blinkLed = millis();
        if(btReady){
            digitalWrite(PIN_READY, HIGH);
            btReady = false;
        } else {
            digitalWrite(PIN_READY, LOW);
            btReady = true;
        }
    }

    bool _btKeyHigh = digitalRead(BT_KEY) == HIGH;

    if(btKeyHigh != _btKeyHigh) {
        btKeyHigh = _btKeyHigh;

        if(btKeyHigh) {
            Serial.println("<BtKey High>");
        } else {
            Serial.println("<BtKey Low>");
        }
    }

    if(UCSerial.available()) {
        receiveUCSerial();
    } else if (millis() - lastSend > MAX_SEND_WAIT) {
        sendBufferNow();
    }

    if(!escapeIsEnabled()) {        // режим моста
        if(SerialBT.available()) {
            receiveBlueTooth();
        }
    }

//---------- Подключение к MQTT брокеру, если соединение разорвано ------------
  
  // Обработка MQTT сообщений
  mqttClient.loop();
  // Отправка MQTT сообщений
  if(millis() - lastSendMqtt > MQTT_SEND_WAIT){
    lastSendMqtt = millis();
    if (mqttClient.connected()) {
        sendMqttBroker();
    } else {
        connectMqttBroker();
    }
  }
}//***************** END loop() ****************************************
void receivedCallback(uint32_t from, String &msg) {
  // Обработка полученных сообщений от других узлов в сети mesh
  // ...
}

void sendMqttBroker(void) {
    ++seconds;
    minuts = seconds%2;
    hour = seconds%5;
    topic = "homeassistant/sensor/";
    topic +=  macaddres;
    topic += "_temper1/config";

  payload = "{\"name\": \"Temper1\",\"state_topic\": \"ISIDA/Node-1/temper1\",\"device_class\": \"temperature\",\"value_template\": \"{{ value | round(2) }}\",\"unique_id\":\"";
  payload += macaddres;
//   payload += "_temper1\"}";
  payload += "_temper1\",\"device\": {\"name\": \"ISIDA\",\"identifiers\": [\"";
  payload += macaddres;
  payload += "\"]}}";

  Serial.println("-------------------------------------------------------------");
  Serial.println(topic);
  Serial.println(payload.c_str());
//   mqttsend = mqttClient.publish(topic.c_str(), payload.c_str(), false);  
//   Serial.println(mqttsend);  
    mqttClient.beginPublish(topic.c_str(),payload.length(),false);
    mqttClient.print(payload.c_str());
    mqttsend = mqttClient.endPublish();
    Serial.println(mqttsend);

    mainTopic = MAIN_TOPIC + String(1);
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
    topic = mainTopic + "/flap";
    payload = String(upv.pv.pvFlap);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/power";
    payload = String(upv.pv.power);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/fuses";
    payload = String(upv.pv.fuses);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/warning";
    payload = String(upv.pv.warning);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/errors";
    payload = String(upv.pv.errors);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    mainTopic = MAIN_TOPIC + String(2);
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
    topic = mainTopic + "/flap";
    payload = String(upv.pv.pvFlap);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/power";
    payload = String(upv.pv.power);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/fuses";
    payload = String(upv.pv.fuses);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/warning";
    payload = String(upv.pv.warning);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
    topic = mainTopic + "/errors";
    payload = String(upv.pv.errors);
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Обработка полученного сообщения от MQTT брокера
  // ...
}

void connectMqttBroker() {
//   while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (mqttClient.connect("ESP8266Client")) {
      Serial.println("Connected to MQTT broker!");
      mqttClient.subscribe("topic1");
      mqttClient.subscribe("topic2");
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
//   }
}


// ------------------  Multiserial  ------------------------------------
void receiveUCSerial(){
    int read = UCSerial.read();
    if(read != -1) {
        if(btKeyHigh) {
            // The uC is trying to send us a command; let's process
            // it as such.
            commandByte(read);
        } else if(isConnected) {
            if(monitorBridgeEnabled()) {
                digitalWrite(PIN_MONITOR, HIGH);
                if(!ucTx || bridgeInit == false) {
                    Serial.println();
                    Serial.print("UC> ");
                    ucTx = true;
                    bridgeInit = true;
                }
                Serial.print((char)read);
            } else {
                digitalWrite(PIN_MONITOR, LOW);
            }

            sendBuffer += (char)read;
            if(
                ((char)read == '\n') 
                || sendBuffer.length() >= (MAX_SEND_BUFFER - 1)
            ) {
                sendBufferNow();
            }
        } else {    //------------- передача масива для MQTT -------------------------------
            if(!ifconfig){
               ipport.data[indData++] = read;
                if(ipport.data[indData-1]==10 && ipport.data[indData-2]==13) {
                    indData = 0;
                    Serial.println();
                    String tempstr = String(ipport.ip.ip[0])+'.'+String(ipport.ip.ip[1])+'.'+String(ipport.ip.ip[2])+'.'+String(ipport.ip.ip[3]);
                    Serial.print("MQTT IP:"); Serial.println(tempstr);
                    tempstr = String(ipport.ip.port);
                    Serial.print("MQTT Port:"); Serial.println(tempstr);
                    tempstr = String(ipport.ip.group);
                    Serial.print("MQTT Group:"); Serial.println(tempstr);
                    ifconfig =true;
                    digitalWrite(PIN_WIFI_CONECTED, HIGH);  // WiFi conected!
                } 
            }else {
                upv.pvdata[indData++] = read;
                if(upv.pvdata[indData-1]==10 && upv.pvdata[indData-2]==13) {
                    indData = 0;
                    // Serial.println("Reseive upv.pvdata!");
                }
            }
            
        }
    }
}
//-------------------------------------------------------------------------
void receiveBlueTooth(){
    int read = SerialBT.read();
    if(isConnected){
    if(read != -1) {
        if(monitorBridgeEnabled()) {
            if(ucTx || bridgeInit == false) {
                Serial.println();
                Serial.print("BT> ");
                ucTx = false;
                bridgeInit = true;
            }
            Serial.print((char)read);
            // Serial.print(";");
        }
        UCSerial.write((char)read);
        if(
            read == BT_CTRL_ESCAPE_SEQUENCE[escapeSequencePos]
            && (
                millis() > (
                    lastEscapeSequenceChar + BT_CTRL_ESCAPE_SEQUENCE_INTERCHARACTER_DELAY
                )
            )
        ) {
            lastEscapeSequenceChar = millis();
            escapeSequencePos++;
        } else {
            escapeSequencePos = 0;
        }
        if(escapeSequencePos == BT_CTRL_ESCAPE_SEQUENCE_LENGTH) {
            enableEscape();
        }
    }
    }
}
//-------------------------------------------------------------------------
void sendBufferNow() {
    int sentBytes = 0;
    if(isConnected) {
        if(sendBuffer.length() > 0) {
            while(sentBytes < sendBuffer.length()) {
                sentBytes += SerialBT.write(
                    &(((const uint8_t*)sendBuffer.c_str())[sentBytes]),
                    sendBuffer.length() - sentBytes
                );
            }
        }
    }
    sendBuffer = "";
    lastSend = millis();
}
