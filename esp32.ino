#include "BluetoothSerial.h"
#include "esp_bt_device.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//===== BUTTONS DEF=====
const int button1pin = 18; //button 1 pin
const int button2pin = 19; //button 2 pin
const int button3pin = 21; //button 3 pin
const int button4pin = 22;
const int button5pin = 23;
const int button6pin = 5;
//followings are for debounce
int button1state;
int button2state;
int button3state;
int button4state;
int button5state;
int button6state;
int lastbutton1state = LOW;
int lastbutton2state = LOW;
int lastbutton3state = LOW;
int lastbutton4state = LOW;
int lastbutton5state = LOW;
int lastbutton6state = LOW;
unsigned long lastbutton1DebounceTime = 0;
unsigned long lastbutton2DebounceTime = 0;
unsigned long lastbutton3DebounceTime = 0;
unsigned long lastbutton4DebounceTime = 0;
unsigned long lastbutton5DebounceTime = 0;
unsigned long lastbutton6DebounceTime = 0;
unsigned long debounceDelay = 25;
////===== BUTTONS DEF END=====

//===== JOYSTICK DEF =====
const int xAxis = 4;         // joystick X axis
const int yAxis = 13;         // joystick Y axis
// parameters for reading the joystick:
int range = 30;               // output range of X or Y movement
int responseDelay = 25;        // response delay of the mouse, in ms
int threshold = 5;    // resting threshold
int xCenter = 58;     // resting position value
int yCenter = 58;

int xMoved;
int yMoved;
//===== JOYSTICK DEF END =====

void setup() {
  Serial.begin(115200);
  SerialBT.begin("INNOSPORT_controller"); //Bluetooth device name

  Serial.println();
  pinMode(button1pin, INPUT);
  pinMode(button2pin, INPUT);
  pinMode(button3pin, INPUT);
//
//  calibrateAxis();
}

void loop() {
  bool sent = false;
  //========== button 1 trigger ==========
  int button1reading = digitalRead(button1pin);
  if (button1reading != lastbutton1state) {
    lastbutton1DebounceTime = millis();
  }
  if ((millis() - lastbutton1DebounceTime) > debounceDelay) {
    if (button1reading != button1state) {
      button1state = button1reading;
      if (button1state == HIGH) {
        Serial.println("Button 1");
        delay(1);
        SerialBT.print("A*");
        sent = true;
      }
//      else if (button1state == LOW) {
//        Serial.println("-Button 1");
//        delay(1);
//        SerialBT.print("-A*");
//      }
    }
  }
  lastbutton1state = button1reading;
  //========== button 1 trigger END ==========

  //========== button 2 trigger ==========
  int button2reading = digitalRead(button2pin);
  if (button2reading != lastbutton2state) {
    lastbutton2DebounceTime = millis();
  }
  if ((millis() - lastbutton2DebounceTime) > debounceDelay) {
    if (button2reading != button2state) {
      button2state = button2reading;
      if (button2state == HIGH) {
        Serial.println("Button 2");
        delay(1);
        SerialBT.print("B*");
        sent = true;
      }
//      else if (button2state == LOW) {
//        Serial.println("-Button 2");
//        delay(1);
//        SerialBT.print("-B*");
//      }
    }
  }
  lastbutton2state = button2reading;
  //========== button 2 trigger END ==========

  //========== button 3 trigger ==========
  int button3reading = digitalRead(button3pin);
  if (button3reading != lastbutton3state) {
    lastbutton3DebounceTime = millis();
  }
  if ((millis() - lastbutton3DebounceTime) > debounceDelay) {
    if (button3reading != button3state) {
      button3state = button3reading;
      if (button3state == HIGH) {
        Serial.println("Button 3");
        delay(1);
        SerialBT.print("C*");
        sent = true;
      }
//      else if (button3state == LOW) {
//        Serial.println("-Button 3");
//        delay(1);
//        SerialBT.print("-C*");
//      }
    }
  }
  lastbutton3state = button3reading;
  //========== button 3 trigger END ==========

  //========== button 4 trigger ==========
  int button4reading = digitalRead(button4pin);
  if (button4reading != lastbutton4state) {
    lastbutton4DebounceTime = millis();
  }
  if ((millis() - lastbutton4DebounceTime) > debounceDelay) {
    if (button4reading != button4state) {
      button4state = button4reading;
      if (button4state == HIGH) {
        Serial.println("Button 4");
        delay(1);
        SerialBT.print("D*");
        sent = true;
      }
//      else if (button4state == LOW) {
//        Serial.println("-Button 4");
//        delay(1);
//        SerialBT.print("-D*");
//      }
    }
  }
  lastbutton4state = button4reading;
  //========== button 4 trigger END ==========

  //========== button 5 trigger ==========
  int button5reading = digitalRead(button5pin);
  if (button5reading != lastbutton5state) {
    lastbutton5DebounceTime = millis();
  }
  if ((millis() - lastbutton5DebounceTime) > debounceDelay) {
    if (button5reading != button5state) {
      button5state = button5reading;
      if (button5state == HIGH) {
        Serial.println("Button 5");
        delay(1);
        SerialBT.print("E*");
        sent = true;
      }
//      else if (button5state == LOW) {
//        Serial.println("-Button 5");
//        delay(1);
//        SerialBT.print("-E*");
//      }
    }
  }
  lastbutton5state = button5reading;
  //========== button 5 trigger END ==========

  //========== button 6 trigger ==========
  int button6reading = digitalRead(button6pin);
  if (button6reading != lastbutton6state) {
    lastbutton6DebounceTime = millis();
  }
  if ((millis() - lastbutton6DebounceTime) > debounceDelay) {
    if (button6reading != button6state) {
      button6state = button6reading;
      if (button6state == HIGH) {
        Serial.println("Button 6");
        delay(1);
        SerialBT.print("F*");
        sent = true;
      }
//      else if (button6state == LOW) {
//        Serial.println("-Button 6");
//        delay(1);
//        SerialBT.print("-F*");
//      }
    }
  }
  lastbutton6state = button6reading;
  //========== button 6 trigger END ==========

  //========== JOYSTICK ==========
  int xReading = readAxis(xAxis);
  int yReading = readAxis(yAxis);
  if (xReading != 0) {
    Serial.print("X: ");
    Serial.println(xReading);
    delay(1);
    SerialBT.print("X:" + String(xReading) + "*");
    sent = true;
  }
  if (yReading != 0) {
    Serial.print("Y: ");
    Serial.println(yReading);
    delay(3);
    SerialBT.print("Y:" + String(yReading) + "*");
    sent = true;
  }
  if (sent == false) {
    delay(1);
    SerialBT.print("NULL*");
  }
  delay(responseDelay);
  //========== JOYSTICK END ==========
}

//void calibrateAxis() {
//  int xValue = 0;
//  int yValue = 0;
//  for (int i = 0; i < 10; i++){
//    xValue += analogRead(xAxis);
//    yValue += analogRead(yAxis);
//  }
//  xCenter = xValue / 10;
//  yCenter = yValue / 10;
//}

int readAxis(int thisAxis) {
  int reading = analogRead(thisAxis);
  reading = map(reading, 0, 1023, 0, range);

  int distance;
  if (thisAxis == xAxis) {
    distance = reading - xCenter;
  }
  else {
    distance = reading - yCenter;
  }
  if (abs(distance) < threshold) {
    distance = 0;
  }
  return distance;
}
