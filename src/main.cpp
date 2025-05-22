#define G433_SPEED 3000 
#include <Arduino.h>
#include <GyverHub.h>
//==========================
#include <Gyver433.h>
Gyver433_RX<5, 16> rx;
//=============================
#define led 2            //D4
#define press 4          //D2
#define led_brite 14     //D5
#define led_beck 12      //D6

///////////////=====================================
uint32_t time_ds01;
uint32_t time_ds02;
//--------------------------
uint32_t time_sist;         //время системы
uint32_t  t_on;             //время вкл аквариум
uint32_t  t_off;            //время выкл аквариум
uint8_t sw_stat;            //положение перекл аквариума
uint8_t sw_press;           //положение переключателя компрессора
uint8_t sw_lbstate;         // точечные светильники
uint8_t sw_lbeckstate;      //выключатель подсветки

uint32_t timer_led;         //

//=========================
struct DataPack {
  int32_t deviceID;
  uint32_t counter;
  float voltage;
  float temper;
};
DataPack data;
//==============================
//////////////======================================

GyverHub hub("MyDev", "дорога", "f63b");  // имя сети, имя устройства, иконка
WiFiClient espClient;

///   WI-FI  ///////////
const char* ssid = "TTK-24";
const char* password = "79811231";

//const char* ssid = "srvrn";
//const char* password = "2155791975";

//   MQTT  /////////////
const char* mqtt_server = "m4.wqtt.ru";
const int mqtt_port = 9478;
const char* mqtt_user = "u_5A3C2X";
const char* mqtt_password = "HilZPRjD";
//=============================
ICACHE_RAM_ATTR void isr() {            // тикер вызывается в прерывании
  rx.tickISR();
}
//===================================
void onunix(uint32_t stamp) {                //получаем дату время. ра6отает!!!
    time_sist = (stamp + 10800 + 21600) % 86400;    //получаем только время и корректируем часовой пояс +3 часа
}

void setup_wifi() {

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void sw_f(){                      //функция вкл-выкл свет аквариум
  digitalWrite(led, !sw_stat);
}
void sw_presss(){                 //компрессор
  digitalWrite(press, !sw_press);
}
void sw_svet(){
  digitalWrite(led_brite, !sw_lbstate);
}
void sw_becksvet(){
  digitalWrite(led_beck, !sw_lbeckstate);
}

void radio(){
  digitalWrite(led, LOW);
  timer_led = millis();
  
  if (rx.readData(data)) {                          // переписываем данные в неё  
    Serial.print("Device ");                        // если данные подходят - выводим
    Serial.println(data.deviceID);
    Serial.println(data.counter);
    Serial.println(data.voltage);
    Serial.println(data.temper);
  } 
  else {
    Serial.println("Wrong data");
  }
  Serial.print("RSSI: ");
  Serial.println(rx.getRSSI());
  Serial.println();

  switch (data.deviceID) {
  case 1:
    time_ds01 = time_sist;
    hub.sendUpdate (F("time_ds01"));
    hub.sendUpdate (F("volt_ds01"));
    hub.sendUpdate (F("count_ds01"));
    hub.sendUpdate (F("temper_ds01"));
    break;

  case 2:
    time_ds02 = time_sist;
    hub.sendUpdate (F("time_ds02"));
    hub.sendUpdate (F("volt_ds02"));
    hub.sendUpdate (F("count_ds02"));
    hub.sendUpdate (F("temper_ds02"));
    break;
  }

  
}

void build(gh::Builder& b){

  if(b.beginRow()){
  b.Time_(F("time"), &time_sist).label(F("время")).color(gh::Colors::Blue);
  b.Display_(F("count"), data.counter).label("").color(gh::Colors::Blue);
  b.Display_(F("voltage"), data.temper).label(F("температура")). color(gh::Colors::Blue);       //сюда шлет свою версию прибор из туалета
  //b.Button_(F("supd"));                                               //по нажатию, все удаленные устройства ищут обновы.
  b.endRow();
  }
  
  //=============================================первая строка. датчик DS 01
  if(b.beginRow()){
  b.Time_(F("time_ds01"), &time_ds01).label(F("обновилось")).color(gh::Colors:: Aqua).click();
  b.Display_(F("volt_ds01"), data.voltage).label(F("напряжение")).color(gh::Colors::Blue);
  b.Display_(F("count_ds01"), data.counter).label(F("count")).color(gh::Colors::Blue);
  b.Display_(F("temper_ds01"), data.temper).label(F("температура")). color(gh::Colors::Blue);
  b.endRow();
}
 
  //=============================================вторая строка. датчик DS 02
  if(b.beginRow()){
  b.Time_(F("time_ds02"), &time_ds02).label(F("обновилось")).color(gh::Colors:: Aqua).click();
  b.Display_(F("volt_ds02"), data.voltage).label(F("напряжение")).color(gh::Colors::Blue);
  b.Display_(F("count_ds02"), data.counter).label(F("count")).color(gh::Colors::Blue);
  b.Display_(F("temper_ds02"), data.temper).label(F("температура")). color(gh::Colors::Blue);
  b.endRow();
}
//====================================================================
  if(b.beginRow()){
    b.Time_(F("t_on"), &t_on).label(F("вкл")).color(gh::Colors::Red).click();
    b.Time_(F("t_off"), &t_off).label(F("выкл")).color(gh::Colors::Green);
    b.Switch_(F("Swit"), &sw_stat).label(F("акваСвет")).attach(sw_f);
    b.Switch_(F("Swit_press"), &sw_press).label(F("компрессор")).attach(sw_presss);
    b.endRow();
  }
  
  if(b.beginRow()){
    b.Switch_(F("led_brite"),&sw_lbstate).label(F("свет")).attach(sw_svet);
    b.Switch_(F("led_beck"),&sw_lbeckstate).label(F("подсветка")).attach(sw_becksvet);
    b.endRow();
  }


}

void setup(){
  Serial.begin(74880);
  Serial.println("");
  Serial.println("Hello");
  Serial.println("версия 0.3");

  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  pinMode(press, OUTPUT);
  digitalWrite(press, LOW);

  pinMode(led_brite, OUTPUT);
  digitalWrite(led_brite, LOW);

  pinMode(led_beck, OUTPUT);
  digitalWrite(led_beck, LOW);
  
  setup_wifi();

  hub.mqtt.config(mqtt_server, mqtt_port, mqtt_user, mqtt_password);
  hub.setVersion("Srvrn1/Road_test@0.3");
  hub.onUnix(onunix);
  hub.onBuild(build);                        // подключаем билдер
  hub.begin();   

  attachInterrupt(5, isr, CHANGE);  // взводим прерывания по CHANGE
}

void loop(){
  hub.tick();

  if (rx.gotData()) radio();

  if(millis() - timer_led > 300) digitalWrite(led, HIGH);

  static GH::Timer tmr(1000);                    //запускаем таймер
  if(tmr){
    time_sist++;
    if(time_sist >= 86400) time_sist = 0;                   //время в Unix формате с6расываем в 00 часов
    hub.sendUpdate(F("time"));

    if(time_sist == t_on){                                 //6удильник включаем Switch
      sw_stat = 1;
      sw_f();
      sw_press = 1;
      sw_presss();

      hub.sendUpdate(F("Swit"));
      hub.sendUpdate(F("Swit_press"));
    }
    if(time_sist == t_off){
      sw_stat = 0;
      sw_f();
      sw_press = 0;
      sw_presss();

      hub.sendUpdate(F("Swit"));
      hub.sendUpdate(F("Swit_press"));
    }
  }

}