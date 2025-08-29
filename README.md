Smart Home System with Arduino and L298N Motor Driver

This lesson introduces children to building a simple Smart Home System using Arduino. The project uses an IR remote control, L298N motor driver for controlling a DC motor, a servo motor for simulating a window, a buzzer as an alarm, and a traffic lights module to simulate driveway signals. This project teaches children about automation, safety, and object-oriented programming concepts in Arduino.



Components Required

Component	Quantity

Arduino Uno/Nano	1

IR Remote + IR Receiver	1 set

L298N Motor Driver	1

DC Motor	1

Servo Motor (SG90)	1

Buzzer	1

Traffic Light Module (3 LEDs)	1

Jumper Wires	Several

External Power Supply (9V/12V Battery)	1



Connection Instructions

IR Receiver OUT	Pin 11	VCC → 5V, GND → GND

L298N IN1	Pin 3	Motor control input 1

L298N IN2	Pin 4	Motor control input 2

L298N ENA	Pin 5 (PWM)	Enable motor speed (HIGH = ON)

Servo Motor	Pin 6	Signal wire to pin 6

Buzzer	Pin 7	Active buzzer

Traffic Light Red	Pin 8	Red LED

Traffic Light Yellow	Pin 9	Yellow LED

Traffic Light Green	Pin 10	Green LED




Teacher’s Notes

- The L298N Motor Driver allows Arduino to safely control DC motors with higher current.
- Always connect the GND of Arduino and L298N together.
- Using a class (SmartHome) helps children think of the house as an object with abilities.
- Encourage children to experiment with motor speed by changing PWM values on ENA pin.
- This project demonstrates automation, safety, and structured coding.





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

