//************************************************************
// this is a simple example that uses the easyMesh library
//
// 1. blinks led once for every node on the mesh
// 2. blink cycle repeats every BLINK_PERIOD
// 3. sends a silly message to every node on the mesh at a random time between 1 and 5 seconds
// 4. prints anything it receives to Serial.print
// Flash: [=======   ]  71.6% (used 1501589 bytes from 2097152 bytes)
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
void mqttConfig(void);

void receivedCallback(uint32_t from, String & msg);
void receiveUCSerial(void);
void sendBufferNow(void);
void sendBufferUC(const uint8_t *buffer, size_t size);
void receiveBlueTooth(void);


String sendMsg = BT_NAME;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
const char* mqttServer = "192.168.1.25";    //192.168.1.25 //public.mqtthq.com
const int mqttPort = 1883;
const char* mqttUsername = "your_mqtt_username";
const char* mqttPassword = "your_mqtt_password";

uint8_t seconds=0;
String topic, payload,  mainTopic = MAIN_TOPIC, clientId ;
union {
  uint8_t data[6];
  struct {
    uint8_t model;   // 1 байт ind=0  модель прибора
    uint8_t node;    // 1 байт Port MQTT broker
    uint8_t ip[4];   // 4 байт IP MQTT broker [192.168.1.25]
  } ip;
} ipcnf;
bool ifconfig = false;
bool datacomplit = false;
// ------------------  Multiserial  ------------------------------------
BluetoothSerial SerialBT;
HardwareSerial UCSerial(1);
MultiSerial CmdSerial;
uint8_t tmpdata[MQTT_SEND_BUFFER+2];
uint8_t UC_SEQUENCE[] = {1,9,4,0,87,0,128,0};
char BT_CTRL_ESCAPE_SEQUENCE[] = {'\4', '\4', '\4', '!'};
uint8_t BT_CTRL_ESCAPE_SEQUENCE_LENGTH = sizeof(BT_CTRL_ESCAPE_SEQUENCE)/sizeof(BT_CTRL_ESCAPE_SEQUENCE[0]);

double dbTemp = 199.9;

unsigned long lastSend = 0, lastSendMqtt = 0, lastReceiveUC = 0;
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
    pinMode(UC_NRST, INPUT);
    pinMode(BT_KEY, INPUT_PULLDOWN);

    pinMode(PIN_WIFI_CONECTED, OUTPUT);
    digitalWrite(PIN_WIFI_CONECTED, LOW);  // WiFi NO conected!
    pinMode(PIN_CONNECTED, OUTPUT);
    digitalWrite(PIN_CONNECTED, LOW);
    pinMode(PIN_MONITOR, OUTPUT);
    digitalWrite(PIN_MONITOR, LOW);
    pinMode(PIN_READY, OUTPUT);
    digitalWrite(PIN_READY, HIGH);
    
    
    UCSerial.begin(9600, SERIAL_8N1, UC_RX, UC_TX);
    UCSerial.setRxBufferSize(1024);

    CmdSerial.addInterface(&Serial);
    CmdSerial.addInterface(&UCSerial);

    sendBuffer.reserve(MAX_SEND_BUFFER);
    commandBuffer.reserve(MAX_CMD_BUFFER);

    setupCommands();

    Serial.println("Waiting for STM32 transmission.");
    //----- Wait MQTT data -----
    indData = 0;
    while(!ifconfig)
    {
        if(UCSerial.available()) {
            receiveUCSerial();
        }
    }
    mainTopic = MAIN_TOPIC + String(ipcnf.ip.node);
    Serial.println();
    Serial.print("MQTT Main topic:"); Serial.println(mainTopic);
    // clientId  = String(upv.pv.ip[0])+'.'+String(upv.pv.ip[1])+'.'+String(upv.pv.ip[2])+'.'+String(upv.pv.ip[3]);
    // Serial.print("MQTT IP:"); Serial.println(clientId );

    //------- New name bluetooth -------
    clientId  = "ISIDA-" + String(ipcnf.ip.node);
    Serial.print("New name bluetooth:"); Serial.println(clientId );
    SerialBT.begin(clientId );
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
    //------------- Инициализация Wi-Fi и подключение к сети -------------
    Serial.println();
    Serial.println("WiFi.begin(Andrew_2023)");

    WiFi.begin("Andrew_2023", "graviton19630301");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
     
    Serial.print("WiFi conected IP: ");
    Serial.println(WiFi.localIP());
    //---------------------- Настройка MQTT клиента -----------------------
    // https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_esp8266/mqtt_esp8266.ino
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(mqttCallback);
    
    //---------------------- Подключение к MQTT брокеру --------------------
    connectMqttBroker();
    mqttConfig();

    mqttClient.beginPublish(topic.c_str(),payload.length(),false);
    mqttClient.print(payload.c_str());
    mqttsend = mqttClient.endPublish();
    if(mqttsend) Serial.println("Discovery topic sent successfully.");
    else Serial.println("Discovery topic not sent!!");
    //--------------------------------------------------------------------
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
// ------------- blue LED flashing -------------------------------------------
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
//-------------------------------------------------------------------------------
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
  
  // Обработка MQTT сообщений
  // https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_esp8266/mqtt_esp8266.ino
  mqttClient.loop();
  // Отправка MQTT сообщений каждые 5 сек.
  if(millis() - lastSendMqtt > MQTT_SEND_WAIT){
    lastSendMqtt = millis();
    if (mqttClient.connected()) {
        sendMqttBroker();
        //-----------------------------
        sendBufferUC(UC_SEQUENCE,8);
        //-----------------------------
    } else {
        connectMqttBroker();
    }
  }
}//***************** END loop() ****************************************

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
            if(!ifconfig){  // передача первичных данных
               ipcnf.data[indData++] = read;
                if(ipcnf.data[indData-1]==10 && ipcnf.data[indData-2]==13) {
                    indData = 0;
                    ifconfig = true;
                    digitalWrite(PIN_WIFI_CONECTED, HIGH);  // WiFi conected!
                    Serial.println();
                    String tempstr = String(ipcnf.ip.model);
                    Serial.print("Model:"); Serial.println(tempstr);
                    tempstr = String(ipcnf.ip.node);
                    Serial.print("Node:"); Serial.println(tempstr);
                    tempstr = String(ipcnf.ip.ip[0])+'.'+String(ipcnf.ip.ip[1])+'.'+String(ipcnf.ip.ip[2])+'.'+String(ipcnf.ip.ip[3]);
                    Serial.print("IP:"); Serial.println(tempstr);
                } 
            }else {         // передача текущих данных
                if(millis() - lastReceiveUC > MAX_SEND_WAIT*10){
                    indData = 0;
                    // Serial.print("lastReceiveUC:"); Serial.println(millis() - lastReceiveUC);
                    lastReceiveUC = millis();
                }
                tmpdata[indData++] = read;
                if(tmpdata[indData-1]==10 && tmpdata[indData-2]==13) {
                    uint16_t crc=0, crcsum;
                    for(int8_t i=0;i<indData-4;i++) {
                        crc += tmpdata[i];
                        crc ^= (crc>>2);
                    }
                    crcsum = tmpdata[indData-3]*256 + tmpdata[indData-4];
                    // Serial.print("IND="); Serial.print(indData-3); Serial.print("  dt0="); Serial.print(tmpdata[indData-4]); Serial.print("  dt1="); Serial.println(tmpdata[indData-3]);
                    // Serial.print("CRC="); Serial.print(crc); Serial.print("  SUM="); Serial.println(crcsum);
                    if(crcsum==crc) memcpy(&upv.pvdata,&tmpdata,MQTT_SEND_BUFFER);
                    indData = 0;
                }
            
            
            }
        }
    }
}
//--------------- режим моста BT -------------------------------------------------
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
    } else {
        if(read != -1) {
        if(monitorBridgeEnabled()) {
            if(ucTx || bridgeInit == false) {
                Serial.println();
                Serial.print("WiFi> ");
                ucTx = false;
                bridgeInit = true;
            }
            Serial.print((char)read);
            Serial.print(";");
        }
        UCSerial.write((char)read);
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
//-------------------------------------------------------------------------
void sendBufferUC(const uint8_t *buffer, size_t size) {
    union {
        uint8_t data[4] = {0,0,13,10};
        struct {
            uint16_t crc;
        } str;   
    } un;
    UC_SEQUENCE[1]++;
    un.str.crc=0;
    for(int8_t i=0;i<8;i++) {
        un.str.crc += buffer[i];
        un.str.crc ^= (un.str.crc>>2);
    }
    UCSerial.write(buffer, size);
    UCSerial.write(un.data, 4);
    lastSend = millis();
}
