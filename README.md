# IR-Remote-Control-For-Home-Setup.
This lesson introduces children to building a simple Smart Home System using Arduino. 

Arduino Code

#include <IRremote.hpp>
#include <Servo.h>

#define IR_PIN      11
#define IN1         3
#define IN2         4
#define ENA         5
#define SERVO_PIN   6
#define BUZZER_PIN  7
#define RED_PIN     8
#define YELLOW_PIN  9
#define GREEN_PIN   10

#define BTN_MOTOR_TOGGLE  0x45
#define BTN_MOTOR_DIR     0x46
#define BTN_SERVOO        0x47
#define BTN_SERVOC        0x44
#define BTN_BUZZER        0x40
#define BTN_TRAFFIC       0x43

class SmartHome {
  private:
    Servo windowServo;
    bool motorState;
    bool motorDir;

  public:
    SmartHome() : motorState(false), motorDir(false) {}

    void begin() {
      pinMode(IN1, OUTPUT);
      pinMode(IN2, OUTPUT);
      pinMode(ENA, OUTPUT);
      pinMode(BUZZER_PIN, OUTPUT);
      pinMode(RED_PIN, OUTPUT);
      pinMode(YELLOW_PIN, OUTPUT);
      pinMode(GREEN_PIN, OUTPUT);
      stopMotor();
      digitalWrite(BUZZER_PIN, LOW);
      windowServo.attach(SERVO_PIN);
      windowServo.write(0);
      IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK);
      Serial.println("Smart Home Ready with L298N!");
    }

    void startMotor() { if (!motorState) { motorState = true; runMotor(); }}
    void stopMotor() { motorState = false; digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0); }
    void toggleMotor() { if (motorState) stopMotor(); else startMotor(); }
    void changeMotorDir() { motorDir = !motorDir; if (motorState) runMotor(); }
    void runMotor() {
      if (motorDir) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
      else { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
      analogWrite(ENA, 200);
    }

    void openWindow() { windowServo.write(90); }
    void closeWindow() { windowServo.write(0); }
    void alarm() { tone(BUZZER_PIN, 1000, 500); }
    void trafficSequence() {
      digitalWrite(RED_PIN, HIGH); delay(2000); digitalWrite(RED_PIN, LOW);
      digitalWrite(GREEN_PIN, HIGH); delay(2000); digitalWrite(GREEN_PIN, LOW);
      digitalWrite(YELLOW_PIN, HIGH); delay(1000); digitalWrite(YELLOW_PIN, LOW);
    }

    void handleRemote() {
      if (IrReceiver.decode()) {
        uint16_t cmd = IrReceiver.decodedIRData.command;
        if (cmd == BTN_MOTOR_TOGGLE) toggleMotor();
        else if (cmd == BTN_MOTOR_DIR) changeMotorDir();
        else if (cmd == BTN_SERVOO) openWindow();
        else if (cmd == BTN_SERVOC) closeWindow();
        else if (cmd == BTN_BUZZER) alarm();
        else if (cmd == BTN_TRAFFIC) trafficSequence();
        IrReceiver.resume();
      }
    }
};

SmartHome myHome;
void setup() { Serial.begin(9600); myHome.begin(); }
void loop() { myHome.handleRemote(); }

