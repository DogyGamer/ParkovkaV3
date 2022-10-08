// 0-не обновлено
// 1-свободно
// 2-занято
// 3-забронированно


struct UpdateResp {
  bool isChanged = false;
  int8_t last_state;
  int8_t state;
};

struct StatesForLed {
  uint8_t id;
  uint8_t prevstate;
  uint8_t newstate;
};





class Parkov_Mesto {
public:
  uint8_t state = 0;
  uint8_t last_state = 0;
  bool isReserved = false;
  void begin(uint8_t _ECHO_pin, uint8_t _TRIG_pin, uint8_t _SERVO_pin, uint8_t _id) {
    ECHO_pin = _ECHO_pin;
    TRIG_pin = _TRIG_pin;
    SERVO_pin = _SERVO_pin;
    id = _id;
    pinMode(ECHO_pin, INPUT);
    pinMode(TRIG_pin, OUTPUT);
    pinMode(SERVO_pin, OUTPUT);
  }


  //Returns struct 
  struct UpdateResp Update() {
    struct UpdateResp result;
    

    last_state = state;
    if(isReserved){
      state = 3;
    }
    else { 
      long dist = GetDistance();
      state = (dist < treshold_distance) ?  2 : 1;
    }


    result.state = state;
    result.last_state = last_state;

    if(state != last_state){
      Serial.print("Mesto state changed id=");
      Serial.print(id);
      Serial.print(" value=");
      Serial.println(state);
      result.isChanged = true;
    }

    return result;
  }

  void ReserveParkSpace(){;
  }

  void UnlockParkSpace(){
  }

  long GetDistance() {
    long summ = 0;
    long values[3];
    for (uint8_t i = 0; i < 3; i++) {
      digitalWrite(TRIG_pin, LOW);
      delayMicroseconds(5);
      digitalWrite(TRIG_pin, HIGH);

      // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
      delayMicroseconds(10);
      digitalWrite(TRIG_pin, LOW);

      //  Время задержки акустического сигнала на эхолокаторе.
      long duration = pulseIn(ECHO_pin, HIGH);
      values[i] = (duration / 2) / 29.1;

      delay(50);
    }
    //Возвращаем значение по медиане чтобы избежать всяких ложноположительных пиков
    long result = (values[0] < values[1]) ? ((values[1] < values[2]) ? values[1] : ((values[2] < values[0]) ? values[0] : values[2])) : ((values[0] < values[2]) ? values[0] : ((values[2] < values[1]) ? values[1] : values[2]));
    Serial.print("Distance id=");
    Serial.print(id);
    Serial.print(" dist=");
    Serial.println(result);
    return result;
  }

private:
  const uint8_t treshold_distance = 10;
  uint8_t ECHO_pin, TRIG_pin, SERVO_pin, id;

};