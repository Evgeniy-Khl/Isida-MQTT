#ifndef main_h
#define main_h

#include "Arduino.h"
#include "BluetoothSerial.h"
#include "multiserial.h"

// some gpio pin that is connected to an LED...
// on my rig, this is 5, change to the right number of your LED.
// #ifdef LED_BUILTIN
//     #define LED LED_BUILTIN
// #else
//     #define LED 2
// #endif
// ------------------  Multiserial  ------------------------------------
// This is the name that this device will appear under during discovery
#define BT_NAME "esp32-bridge"
// At least this number of ms must elapse between each
// character of the escape sequence for it to be counted; this
// is done to prevent legitimate occurrences of the escape sequence
// occurring in binary causing us to enter the bridge mode.
#define BT_CTRL_ESCAPE_SEQUENCE_INTERCHARACTER_DELAY 250
// For communicating with the microcontroller, we will transmit and
// receive on the following pins.  Do not set these to match the pins
// used by defualt for the ESP32 unit's UART1 (IO35 and IO34).  I've also
// heard, but not confirmed, that the IO pin used for TX must be a higher
// IO number than the pin used for RX.  If you can confirm or deny this,
// I'd love to hear a definitive answer.
#define UC_TX 27
#define UC_RX 14
// Этот контакт будет переведен в ВЫСОКИЙ уровень (если он определен), когда устройство будет готово к подключению.
#define PIN_READY 2
#define PIN_MONITOR 5
#define PIN_WIFI_CONECTED 18
// This pin will be pulled HIGH when a client is connected over bluetooth.
#define PIN_CONNECTED 4
// Если ваш микроконтроллер выведет этот вывод на ВЫСОКИЙ уровень, он сможет отправлять команды непосредственно на модуль ESP32.
#define BT_KEY 16
// Подключите его к линии сброса микроконтроллера, чтобы вы могли сбросить микроконтроллер по своему желанию.
#define UC_NRST 17
#define MAX_SEND_WAIT 50
#define MAX_CMD_BUFFER 128
#define MAX_SEND_BUFFER 128
#define MQTT_SEND_BUFFER 36
#define MQTT_SEND_WAIT 5000
//------------------------------------------------------------------------

#define   BLINK_PERIOD    3000 // milliseconds until cycle repeat
#define   BLINK_DURATION  100  // milliseconds LED is on for

#define   MAIN_TOPIC      "ISIDA/Node-"

// Prototypes
void sendMessage(); 
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback(); 
void nodeTimeAdjustedCallback(int32_t offset); 
void delayReceivedCallback(uint32_t from, int32_t delay);

union pvValue{
  uint8_t pvdata[MQTT_SEND_BUFFER];
  struct rampv {
    uint8_t model;                // 1 байт ind=0         сетевой номер и тип прибора
    uint8_t node;                 // 1 байт ind=1         сетевой номер и тип прибора
    int16_t pvT[3];               // 6 байт ind=2-ind=7   значения 3 датчиков температуры
    uint8_t pvRH;                 // 1 байт ind=8         значение датчика относительной влажности
    uint8_t pvCO2;                // 1 байт ind=9         значения датчика CO2
    uint8_t pvTimer;              // 1 байт ind=10        значение таймера
    uint8_t pvFan;                // 1 байт ind=11        
    uint8_t pvFlap;               // 1 байт ind=12        положение заслонки 
    uint8_t power;                // 1 байт ind=13        мощность подаваемая на тены
    uint8_t fuses;                // 1 байт ind=14        короткие замыкания 
    uint8_t errors;               // 1 байт ind=15        ошибки и предупреждения
    uint8_t warning;              // 1 байт ind=16        ошибки и предупреждения
    uint8_t other0;               // 1 байт ind=17        не используется !!!!!!!!!!!!!!!!!!!!!!!!!
    uint8_t other1;               // 1 байт ind=18        не используется !!!!!!!!!!!!!!!!!!!!!!!!!
    uint8_t other2;               // 1 байт ind=19        не используется !!!!!!!!!!!!!!!!!!!!!!!!!
    int16_t spT[2];     // 4 байт ind=20-ind=23   Уставка температуры sp[0].spT->Сухой датчик; sp[1].spT->Влажный датчик
    int8_t  spRH[2];    // 2 байт ind=24;ind=25   spRH[0]->ПОДСТРОЙКА HIH-5030; spRH[1]->Уставка влажности Датчик HIH-5030
    uint8_t state;      // 1 байт ind=26;        состояние камеры (ОТКЛ. ВКЛ. ОХЛАЖДЕНИЕ, и т.д.)
    uint8_t extendMode; // 1 байт ind=27;        расширенный режим работы  0-СИРЕНА; 1-ВЕНТ. 2-Форс НАГР. 3-Форс ОХЛЖД. 4-Форс ОСУШ. 5-Дубляж увлажнения
    uint8_t relayMode;  // 1 байт ind=28;        релейный режим работы  0-НЕТ; 1->по кан.[0] 2->по кан.[1] 3->по кан.[0]&[1] 4->по кан.[1] импульсный режим
    uint8_t programm;   // 1 байт ind=29;        работа по программе
    uint8_t ip[4];      // 4 байт ind=30;ind=31;ind=32;ind=33; IP MQTT broker [192.168.100.100]
  } pv;// ------------------ ИТОГО 30 bytes -------------------------------
} ;

extern MultiSerial CmdSerial;
extern HardwareSerial UCSerial;
extern BluetoothSerial SerialBT;

#endif /* _ESP32_CORE_MAIN_H_ */