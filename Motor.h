
enum direction { FORWARD, BACK };

class Motor {

  uint8_t ENA = 3, ENB = 9;
  uint8_t IN1 = 4, IN2 = 5, IN3 = 6, IN4 = 7;

  int power_ = 0; // 0...255

public:

  Motor() {}

  Motor(uint8_t ENA, uint8_t ENB, uint8_t IN1, 
        uint8_t IN2, uint8_t IN3, uint8_t IN4)
  : ENA(ENA), ENB(ENB)
  , IN1(IN1), IN2(IN2), IN3(IN3), IN4(IN4)
  , power_(0)
  {}

  // Инициализация моторов в setup()
  void initialize() const {
    // Установка всех управляющих пинов в режим выхода
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    // Команда остановки двум моторам
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }//initialize()

  void set_power(uint8_t power) { power_ = constrain(power, 0, 255); }
  uint8_t get_power() const { return power_; }

  void run(bool direction){
    if(direction){
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENA, power_);
      analogWrite(ENB, power_);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENA, power_);
      analogWrite(ENB, power_);
    }//if
  }//run
};