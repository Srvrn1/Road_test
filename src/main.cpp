
#include <Arduino.h>
#include <GyverHub.h>
//==========================
#include <Gyver433.h>
Gyver433_RX<5, 4> rx;
//=============================
#define led 2            //D4
#define press 4          //D2
#define led_brite 14     //D5
#define led_beck 12      //D6

///////////////=====================================
uint32_t time_sist;         //время системы
uint32_t  t_on;             //время вкл аквариум
uint32_t  t_off;            //время выкл аквариум
uint8_t sw_stat;            //положение перекл аквариума
uint8_t sw_press;           //положение переключателя компрессора
uint8_t sw_lbstate;         // точечные светильники
uint8_t sw_lbeckstate;      //выключатель подсветки
uint8_t Rsw_svet;           //выкл Риты
uint8_t Rsw_roz;            //ритина розетка

uint8_t sw_mg;              //положение переключателя в туалете
const char*  vers_mg = "0";             //версия прошивы туалетного контроллера
 
//=========================
struct DataPack {
  byte counter = 0;
  byte randomNum;
  uint16_t analog;
  //uint32_t time;
};
DataPack data;
//==============================
//////////////======================================

GyverHub hub("MyDev", "дорога", "f0ad");  // имя сети, имя устройства, иконка
WiFiClient espClient;

///   WI-FI  ///////////
//const char* ssid = "RT-WiFi-0FBE";
//const char* password = "YeNu5VAyeY";
//const char* ssid = "srvrn";
//const char* password = "2155791975";
const char* ssid = "srvrn";
const char* password = "2155791975";

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
    time_sist = (stamp + 10800) % 86400;    //получаем только время и корректируем часовой пояс +3 часа
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
    if (rx.readData(data)) {                          // переписываем данные в неё
        // если данные подходят - выводим
      Serial.println(data.counter);
      Serial.println(data.randomNum);
      Serial.println(data.analog);
      //Serial.println(data.time);
      Serial.println();
    } 
    else {
      Serial.println("Wrong data");
    }
    Serial.print("RSSI: ");
    Serial.println(rx.getRSSI());
  
}

void build(gh::Builder& b){
  if(b.beginRow()){
  b.Time_(F("time"), &time_sist).label(F("время")).color(gh::Colors::Blue);
  b.Display(F("V1.6.6")).label(F("Releases")).color(gh::Colors::Blue);
  b.Display_(F("vers")).label(F("версия")). color(gh::Colors::Blue);                      //сюда шлет свою версию прибор из туалета
  b.Button_(F("supd"));                                               //по нажатию, все удаленные устройства ищут обновы.
  b.endRow();
  }

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

  if(b.beginRow()){
    b.Display_(F("hvs")).label(F("ХВС")).color(gh::Colors::Aqua);
    b.Display_(F("gvs")).label(F("ГВС")).color(gh::Colors::Orange);
    b.Switch_(F("mg"), &sw_mg).label(F("М/Ж")).color(gh::Colors::Red);
    b.endRow();
  }

  b.Title(F("Рита"));
  if(b.beginRow()){
    b.Switch_(F("Rsvet"), &Rsw_svet).label(F("парта"));    //.attach(sw_svet);
    b.Switch_(F("Rroz"), &Rsw_roz).label(F("розетка"));    //.attach(sw_becksvet);
    b.endRow();
  }
}

void setup(){
  Serial.begin(74880);
  Serial.println("");
  Serial.println("Hello");
  Serial.println("версия 0.1");

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
  hub.setVersion("Srvrn1/Lipa@0.1");
  hub.onUnix(onunix);
  hub.onBuild(build);                        // подключаем билдер
  hub.begin();   

  attachInterrupt(5, isr, CHANGE);  // взводим прерывания по CHANGE
}

void loop(){
  hub.tick();

  if (rx.gotData()) radio();

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