

#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>

#include "WiFiEsp.h"
// #include <WiFiEspAT.h>

#include "WiFiProvider.h"

#define STRIP_PIN 3     // пин ленты
#define NUMLEDS 8      // кол-во светодиодов
#define COLOR_DEBTH 3
#include <microLED.h>   // подключаем библу

microLED<NUMLEDS, STRIP_PIN, MLED_NO_CLOCK, LED_WS2812, ORDER_GRB, CLI_HIGH, SAVE_MILLIS> strip;

#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(6, 7);  // RX, TX
#endif


char ssid[] = "TP-LINK_92";
char pass[] = "O247HY54ru";
char ipaddr[] = "192.168.0.150";
uint16_t port = 3939;

WiFiEspClient wifi;
HttpClient client = HttpClient(wifi, ipaddr, port);


int status = WL_IDLE_STATUS;
IPAddress ip;

const uint8_t pins[2][3] = {
  {9,8,31},
  {11,12,30},
};

const uint8_t CountSpaces = (sizeof(pins) / sizeof(pins[0]));
const uint8_t LedLength = NUMLEDS / CountSpaces;

Parkov_Mesto Spaces[CountSpaces];


bool isStarted = false;
bool isRequest = false;

int GetReservedCounter = 0;

void setup() {
  strip.fill(mRGB(255, 0, 0));  // заливаем черным
  strip.setBrightness(255);
  strip.show(); 
  Serial.begin(9600);
  Serial1.begin(9600);
  connectWIFI();
  client.setTimeout(0);
  client.setHttpResponseTimeout(2000);
  delay(500);

  sendStartupMessage();

  delay(500);
  FillsSpaces();
}

void loop() {
  StaticJsonDocument<400> doc;
  JsonObject states = doc.createNestedObject("states");

  if(GetReservedCounter == 2){
    GetReservedCounter = 0;
    String json = getReserved();

    StaticJsonDocument<200> reservedDoc;
    DeserializationError error = deserializeJson(reservedDoc, json);
  // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }

    Serial.println();
    for(uint8_t id=0; id<CountSpaces; id++){
      Spaces[id].isReserved = false;
    }
    for(int index=0; index < reservedDoc["res"].size(); index++ ){
      Spaces[reservedDoc["res"][index].as<int>()].isReserved = true;
    }
    
  }
  GetReservedCounter++;
  struct StatesForLed ledstates[CountSpaces];


  int8_t Counter = 0;
  for(uint8_t id=0; id<CountSpaces; id++){
    struct UpdateResp state = Spaces[id].Update();
    states[String(id)] = state.state;
    if(state.isChanged){
      
        if(state.last_state != 0 && state.last_state != 3){
          if(state.state != 3){
            Serial.print("Blinking state=");
            Serial.print(state.state);
            Serial.print(" last=");
            Serial.println(state.last_state);
            Blink(state.last_state, id);
          }
        }

        ledstates[Counter].id = id;
        ledstates[Counter].prevstate = state.last_state;
        ledstates[Counter].newstate = state.state;

        Counter++;
      
    }
  }

  if(Counter > 0){
    String statesJson;
    serializeJson(doc, statesJson);
    Serial.println(statesJson);

    sendStates(statesJson);
    for(uint8_t id=0; id<CountSpaces; id++){
      Spaces[id].AttachServo();
    }

    Shutdown(ledstates, Counter);
    Wakeup(ledstates, Counter);

    for(uint8_t id=0; id<CountSpaces; id++){
      Spaces[id].DeatachServo();
    }
  }
  

  doc.clear();
  delay(500);
}



void Shutdown(struct StatesForLed states[], uint8_t size){
  for(int i=255; i>0; i--){
    for(int ind=0; ind<size; ind++){
      if(states[ind].prevstate==3 && states[ind].newstate != 3){
        Spaces[states[ind].id].servo.write(map(i, 0, 255, 0,90));
      }
      strip.fill( ((states[ind].id) * (LedLength)), ((states[ind].id)*(LedLength))+LedLength-1, ( (states[ind].prevstate==3) ? mRGB(0,0,i) : ((states[ind].prevstate == 1) ? mRGB(0,i,0) : mRGB(i, 0, 0)) ) );
    }
    // strip.fill(start_pos, start_pos+LedLength-1, ( (prev == 1) ? mRGB(0,i,0) : mRGB(i, 0, 0) ) );
    strip.show();
    delay(4);
  }
}

void Wakeup(struct StatesForLed states[], uint8_t size){
  for(int i=0; i<255; i++){
    for(int ind=0; ind<size; ind++){
      if(states[ind].newstate == 3 && states[ind].prevstate != 3){
        Spaces[states[ind].id].servo.write(map(i, 0, 255, 0,90));
      }
      strip.fill( ((states[ind].id) * (LedLength)), ((states[ind].id)*(LedLength))+LedLength-1, ( (states[ind].newstate==3) ? mRGB(0,0,i) : ((states[ind].newstate == 1) ? mRGB(0,i,0) : mRGB(i, 0, 0)) ) );
    }
    // strip.fill(start_pos, start_pos+LedLength-1, ( (state == 1) ? mRGB(0,i,0) : mRGB(i, 0, 0) ) );
    strip.show();
    delay(4);
  }
}

void Blink(int state, int index) {
  uint8_t start_pos = LedLength * index;
  for(int i=255; i>32; i--){
    strip.fill(start_pos, start_pos+LedLength-1,  ( (state==3) ? mRGB(0,0,i) : ((state == 1) ? mRGB(0,i,0) : mRGB(i, 0, 0)) ) );
    strip.show();
    delay(3);
  }
  for(int i=32; i<255; i++){
    strip.fill(start_pos, start_pos+LedLength-1, ( (state==3) ? mRGB(0,0,i) : ((state == 1) ? mRGB(0,i,0) : mRGB(i, 0, 0)) )  );
    strip.show();
    delay(3);
  }
}

String getReserved(){
  Serial.println("Getting ReservedSpaces");

  client.beginRequest();
  client.post("/getReserved", "application/plain-text", "");
  client.endRequest();

  delay(50);

  int16_t statusCode1 = client.responseStatusCode();
  String response1 = client.responseBody();

  Serial.print("Response reserved status=" + String(statusCode1));
  Serial.println(" response=" + response1);


  // client.skipResponseHeaders();
  client.flush();
  client.stop();
  // client.clearWriteError();


  return response1;
}


void sendStates(String statesJson){
  Serial.println("Sending states");

  client.beginRequest();
  client.post("/updateStates", "application/json", statesJson);
  client.endRequest();

  delay(50);

  int16_t statusCode1 = client.responseStatusCode();
  String response1 = client.responseBody();

  Serial.print("Response states status=" + String(statusCode1));
  Serial.println(" response=" + response1);

  // client.skipResponseHeaders();
  client.flush();
  client.stop();
  // client.clearWriteError();

}

void sendStartupMessage() {
  Serial.println("Sending startup message!");
  StaticJsonDocument<50> doc;

  String ipString = "" + String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
  doc["ip"] = "" + ipString;
  doc["spc"] = String(CountSpaces);

  String out;
  ArduinoJson6113_00000::serializeJson(doc, out);
  Serial.println(out);

  client.beginRequest();
  client.post("/hello", "application/json", out);
  client.endRequest();

  delay(100);

  int16_t statusCode1 = client.responseStatusCode();
  String response1 = client.responseBody();

  Serial.print("Response startup msg status=" + String(statusCode1));
  Serial.println(" response=" + response1);


  // client.skipResponseHeaders();
  client.flush();
  client.stop();
  // client.clearWriteError();
}

void connectWIFI() {
  WiFi.init(&Serial1);
  // WiFi.init(Serial1);
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    while (true)
      ;
  }
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
  }
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}

void FillsSpaces() {
  for(uint8_t i=0; i < CountSpaces; i++){
    Spaces[i].begin(pins[i][0], pins[i][1], pins[i][2], i);
    Serial.print("Initializing Space id=");
    Serial.print(i);
    Serial.print(" ECHO=");
    Serial.print(pins[i][0]);
    Serial.print(" TRIG=");
    Serial.print(pins[i][1]);
    Serial.print(" SERVO=");
    Serial.println(pins[i][2]);
  }
}
