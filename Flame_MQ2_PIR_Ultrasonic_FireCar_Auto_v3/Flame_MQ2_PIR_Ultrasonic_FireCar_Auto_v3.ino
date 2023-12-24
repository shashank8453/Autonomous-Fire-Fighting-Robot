#define BLYNK_TEMPLATE_ID "TMPL3QMJHmEYV"
#define BLYNK_TEMPLATE_NAME "Fire Fighting Robot"
#define BLYNK_AUTH_TOKEN "2c8M1wkId2Mm3JwJyaOGXzsA6wnGIYdW"
#define BLYNK_PRINT Serial

// Libraries:
#include <WiFi.h>
/* Add JSON Package https://dl.espressif.com/dl/package_esp32_index.json
  and Install ESP32 Board */

#include <BlynkSimpleEsp32.h>
/* Install: Blynk by Volodymyr Shymanskyy */

#include <ESP32Servo.h>
/* Built-In Library */

// Pin Numbers:
#define OnBoardLED_Pin 2

// Pin Numbers: L298N Motor Driver
#define RightMtr_Pin1 22   // (Output) Connect to IN1 of L298N, Right Motor Pin1
#define RightMtr_Pin2 23   // (Output) Connect to IN2 of L298N, Right Motor Pin2
#define LeftMtr_Pin1 16    // (Output) Connect to IN3 of L298N, Left Motor Pin1
#define LeftMtr_Pin2 17    // (Output) Connect to IN4 of L298N, Left Motor Pin2

// Pin Numbers: 5Ch Flame Sensor
#define RightEnd_Pin 35   // (Input) Connect to A1 pin of flame sensor.
#define Right_Pin 34      // (Input) Connect to A2 pin of flame sensor.
#define Mid_Pin 32        // (Input) Connect to A3 pin of flame sensor.
#define Left_Pin 33       // (Input) Connect to A4 pin of flame sensor.
#define LeftEnd_Pin 39    // (Input) Connect to A5 pin of flame sensor.

// Pin Numbers: Flame
#define PIR_Pin 27
#define MQ2_Pin 36

// Pin Numbers: Ultrasonic Sensor
#define US_Trig_Pin 18   // (OUTPUT) Connect to Trig pin
#define US_Echo_Pin 19   // (INPUT)  Connect to Echo pin

// Pin Numbers: Relay Module
#define PumpRelay_Pin 21    // (Output) Connect to the IN1 pin of Relay Module

// Pin Numbers: Servo
#define ServoPan_Pin 13   // Connect to Servo Motor

// Pin Numbers: Blynk Virtual Pins
#define MQ2Indicator_vPin V0        // Type: Integer, Range: 0-1
#define ObstacleIndicator_vPin V1   // Type: Integer, Range: 0-1
#define MotionIndicator_vPin V2
#define MQ2Value_vPin V3
#define RightEnd_vPin V4
#define Right_vPin V5
#define Mid_vPin V6
#define Left_vPin V7
#define LeftEnd_vPin V8
#define Message_vPin V9

// Objects: Blynk Widget
WidgetLED wdMQ2LED(MQ2Indicator_vPin);
WidgetLED wdObstacleLED(ObstacleIndicator_vPin);
WidgetLED wdMotionLED(MotionIndicator_vPin);

// Configuration: Blynk Project Authentication Key
const char auth[] = BLYNK_AUTH_TOKEN;

// Configuration: WiFi Credentials
const char ssid[] = "Soma";        // Name of your network (Hotspot or Router Name)
const char pass[] = "soma1234";   // Corresponding Password

// Configurations: Threshold Value
const int MQ2_thValue = 2000;
const int dist_thValue = 30;

// Objects:
Servo servoPan;

// Variables:
bool Flame_State;
bool PIR_State;
int MQ2_Value;
int distance;

// Variables:
int RightEnd_Value;
int Right_Value;
int Mid_Value;
int Left_Value;
int LeftEnd_Value;
bool isFlame = false;

// Variables:
int srvLeftPos = 180;
int srvMidPos = 90;
int srvRightPos = 0;
int leftDist;
int rightDist;

void setup() {
  // Pin Mode Configuration:
  pinMode(OnBoardLED_Pin, OUTPUT);
  pinMode(US_Trig_Pin, OUTPUT);
  pinMode(US_Echo_Pin, INPUT);
  pinMode(PIR_Pin, INPUT);

  pinMode(LeftMtr_Pin1, OUTPUT);
  pinMode(LeftMtr_Pin2, OUTPUT);
  pinMode(RightMtr_Pin1, OUTPUT);
  pinMode(RightMtr_Pin2, OUTPUT);
  StopTheCar();

  pinMode(PumpRelay_Pin, OUTPUT);
  digitalWrite(PumpRelay_Pin, LOW);

  /* Establish serial communication with the PC */
  Serial.begin(9600);

  digitalWrite(OnBoardLED_Pin, LOW);
  /* Begin communication with Blynk App */
  Blynk.begin(auth, ssid, pass);
  digitalWrite(OnBoardLED_Pin, HIGH);

  wdMQ2LED.on();
  wdObstacleLED.on();
  wdMotionLED.on();
  ActuateServo(servoPan, ServoPan_Pin, srvMidPos);
}

void loop() {
  Blynk.run();

  PIR_State = digitalRead(PIR_Pin);
  Serial.print("PIR State = ");
  Serial.print(PIR_State);
  if (PIR_State == 1) {
    Blynk.setProperty(MotionIndicator_vPin, "color", "#D3435C"); // Red Color
    Serial.print(": Motion Detected!");
  } else  {
    Blynk.setProperty(MotionIndicator_vPin, "color", "#41E1AC"); // Green Color
  }
  Serial.println();
  delay(100);

  MQ2_Value = analogRead(MQ2_Pin);
  Serial.print("MQ-2 Value = ");
  Serial.print(MQ2_Value);
  Blynk.virtualWrite(MQ2Value_vPin, MQ2_Value);

  if (MQ2_Value > MQ2_thValue) {
    Serial.print(": LPG/Smoke Detected!");
    Blynk.setProperty(MQ2Indicator_vPin, "color", "#D3435C"); // Red Color
  } else {
    Blynk.setProperty(MQ2Indicator_vPin, "color", "#41E1AC"); // Green Color
  }
  Serial.println();
  delay(100);

  distance = GetUltrasonicDist(US_Trig_Pin, US_Echo_Pin);
  Serial.print("Distance: ");
  Serial.print(distance);
  if (distance < dist_thValue) {
    Blynk.setProperty(ObstacleIndicator_vPin, "color", "#41B1E1"); // Blue Color
    Serial.print(": Obstacle Detected!");
    StopTheCar();
    if (isFlame) {
      ActuateServo(servoPan, ServoPan_Pin, srvLeftPos);
      leftDist = GetUltrasonicDist(US_Trig_Pin, US_Echo_Pin);
      Blynk.virtualWrite(Message_vPin, "Obstacle: Left Dist " + String(leftDist));
      delay(1000);

      ActuateServo(servoPan, ServoPan_Pin, srvRightPos);
      rightDist = GetUltrasonicDist(US_Trig_Pin, US_Echo_Pin);
      Blynk.virtualWrite(Message_vPin, "Obstacle: Right Dist " + String(rightDist));
      delay(1000);

      ActuateServo(servoPan, ServoPan_Pin, srvMidPos);

      if (rightDist > dist_thValue || leftDist > dist_thValue) {
        if (rightDist > leftDist) {
          Serial.println("Right Turn");
          Blynk.virtualWrite(Message_vPin, "Obstacle: Right Turn");
          TurnRight();
          delay(1000);
          Serial.println("GoForward");
          Blynk.virtualWrite(Message_vPin, "Obstacle: GoForward");
          GoForward();
          delay(1000);
          Serial.println("TurnLeft");
          Blynk.virtualWrite(Message_vPin, "Obstacle: TurnLeft");
          TurnLeft();
        } else {
          Serial.println("Left Turn");
          Blynk.virtualWrite(Message_vPin, "Obstacle: Left Turn");
          TurnLeft();
          delay(1000);
          Serial.println("GoForward");
          Blynk.virtualWrite(Message_vPin, "Obstacle: GoForward");
          GoForward();
          delay(1000);
          Serial.println("TurnRight");
          Blynk.virtualWrite(Message_vPin, "Obstacle: TurnRight");
          TurnRight();
        }
        delay(1000);
        Serial.println("GoForward");
        Blynk.virtualWrite(Message_vPin, "Obstacle: GoForward");
        GoForward();
        delay(1000);
        Serial.println("Stop");
        Blynk.virtualWrite(Message_vPin, "Obstacle: Stop");
        StopTheCar();
        isFlame = false;
      }
    }
  } else {
    Blynk.setProperty(ObstacleIndicator_vPin, "color", "#41E1AC"); // Green Color
  }
  Serial.println();
  Serial.println();
  delay(10);

  RightEnd_Value = analogRead(RightEnd_Pin);
  delay(10);
  Right_Value = analogRead(Right_Pin);
  delay(10);
  Mid_Value = analogRead(Mid_Pin);
  delay(10);
  Left_Value = analogRead(Left_Pin);
  delay(10);
  LeftEnd_Value = analogRead(LeftEnd_Pin);
  delay(10);

  Blynk.virtualWrite(RightEnd_vPin, RightEnd_Value);
  Blynk.virtualWrite(Right_vPin, Right_Value);
  Blynk.virtualWrite(Mid_vPin, Mid_Value);
  Blynk.virtualWrite(Left_vPin, Left_Value);
  Blynk.virtualWrite(LeftEnd_vPin, LeftEnd_Value);

  //  Serial.print("Values: ");
  //  Serial.print(RightEnd_Value);
  //  Serial.print(" ");
  //  Serial.print(Right_Value);
  //  Serial.print(" ");
  //  Serial.print(Mid_Value);
  //  Serial.print(" ");
  //  Serial.print(Left_Value);
  //  Serial.print(" ");
  //  Serial.print(LeftEnd_Value);
  //  Serial.print(" : ");


  if (Right_Value <= 400 && RightEnd_Value <= 400 && Mid_Value <= 400 && Left_Value <= 400 && LeftEnd_Value <= 400) {
    Blynk.virtualWrite(Message_vPin, "Stop");
    Serial.println("Stop");
    StopTheCar();
  } else if (Mid_Value >= 3000) {
    Blynk.virtualWrite(Message_vPin, "Fire Extinguish");
    Serial.println("GoBackward, Stop and Pump On");
    FireExtinguish();
  } else {
    isFlame = true;
    if (Mid_Value <= 300 && Right_Value >= 400 || RightEnd_Value >= 400) {
      Blynk.virtualWrite(Message_vPin, "Right");
      Serial.println("Right");
      TurnRight();
    } else if (Mid_Value <= 300 && Left_Value >= 400 || LeftEnd_Value >= 400) {
      Blynk.virtualWrite(Message_vPin, "Left");
      Serial.println("Left");
      TurnLeft();
    } else if (Mid_Value >= 400 && Right_Value <= 300 && RightEnd_Value <= 300 && Left_Value <= 300 && LeftEnd_Value <= 300) {
      Blynk.virtualWrite(Message_vPin, "Forward 1");
      Serial.println("Forward 1");
      GoForward();
    } else if (Mid_Value >= 400) {
      Blynk.virtualWrite(Message_vPin, "Forward 2");
      Serial.println("Forward 2");
      GoForward();
    }
  }
}


int GetUltrasonicDist(int trigPin, int echoPin) {
  int distance;
  /* Clear the trig pin by setting it LOW */
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);

  /* Trigger the sensor by setting the trig pin HIGH for 10 microseconds */
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  /* Read the echoPin. pulseIn() returns the duration (length of the pulse) in microseconds */
  long duration = pulseIn(echoPin, HIGH);

  /* Calculate the distance: Distance = Speed * Time
     Speed of sound is 343 metres per second = 0.0343cm per micro second */
  distance = 0.0343 * (duration / 2);

  return distance;
}

void FireExtinguish() {
  GoBackward();
  delay(300);
  StopTheCar();
  digitalWrite(PumpRelay_Pin, HIGH);
  delay(3000);
  digitalWrite(PumpRelay_Pin, LOW);
  isFlame = false;
}

// Transmission Control: Turn Off
void MotorOFF(int pin1, int pin2) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
}

// Direction Control: Forward (CW)
void ForwardDir(int pin1, int pin2) {
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
}

// Direction Control: Backward (CCW)
void BackwardDir(int pin1, int pin2) {
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
}

void StopTheCar() {
  MotorOFF(LeftMtr_Pin1, LeftMtr_Pin2);
  MotorOFF(RightMtr_Pin1, RightMtr_Pin2);
}

void GoForward() {
  if (GetUltrasonicDist(US_Trig_Pin, US_Echo_Pin) > 30) {
    ForwardDir(LeftMtr_Pin1, LeftMtr_Pin2);
    ForwardDir(RightMtr_Pin1, RightMtr_Pin2);
  } else  {
    StopTheCar();
  }
}

void GoBackward() {
  BackwardDir(LeftMtr_Pin1, LeftMtr_Pin2);
  BackwardDir(RightMtr_Pin1, RightMtr_Pin2);
}

void TurnLeft() {
  BackwardDir(LeftMtr_Pin1, LeftMtr_Pin2);
  ForwardDir(RightMtr_Pin1, RightMtr_Pin2);
}

void TurnRight() {
  ForwardDir(LeftMtr_Pin1, LeftMtr_Pin2);
  BackwardDir(RightMtr_Pin1, RightMtr_Pin2);
}

void ActuateServo(Servo &objServo, int Servo_Pin, int srvPos) {
  objServo.attach(Servo_Pin);
  objServo.write(srvPos);
  delay(2000);
}
