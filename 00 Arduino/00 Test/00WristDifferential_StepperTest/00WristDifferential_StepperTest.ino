#include <AccelStepper.h>

// FULL4WIRE -> Full step control
// AccelStepper st1(AccelStepper::FULL4WIRE, 8, 10, 9, 11);
// AccelStepper st2(AccelStepper::FULL4WIRE, 4, 6, 5, 7);

// HALF4WIRE -> Half step control (More precise)
AccelStepper st1(AccelStepper::HALF4WIRE, 8, 10, 9, 11);
AccelStepper st2(AccelStepper::HALF4WIRE, 4, 6, 5, 7);


const long rotationAmount = 800;
const float speed = 1000;
const float accel = 800;


String msg;
bool moving = false;

void setup() {
  Serial.begin(9600);

  st1.setMaxSpeed(speed);
  st1.setAcceleration(accel);
  st2.setMaxSpeed(speed);
  st2.setAcceleration(accel);

  st1.setCurrentPosition(0);
  st2.setCurrentPosition(0);

  st1.moveTo(0);
  st2.moveTo(0);

  Serial.println("CMD Ready");
}

void loop() {
  st1.run();
  st2.run();

  // Read CMD when it's ready
  if (moving) {
    if (st1.distanceToGo() == 0 && st2.distanceToGo() == 0) {
      moving = false;
      while (Serial.available() > 0) Serial.read();
    }
    return;
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (msg.length() > 0) {
        int cmd = msg.toInt();
        msg = "";

        Serial.print("CMD = ");
        Serial.println(cmd);

        actionSeq(cmd);
        moving = true;
      }
    } else {
      msg += c;
    }
  }
}

void actionSeq(int cases) {
  // Memorize position
  long p1 = st1.currentPosition();
  long p2 = st2.currentPosition();

  switch (cases) {
    case 0:
      // Total reset
      st1.moveTo(0);
      st2.moveTo(0);
      break;

    case 1:
      st1.moveTo(p1 + rotationAmount);
      st2.moveTo(p2 + rotationAmount);
      break;

    case 2:
      st1.moveTo(p1 - rotationAmount);
      st2.moveTo(p2 - rotationAmount);
      break;

    case 3:
      st1.moveTo(p1 + rotationAmount);
      st2.moveTo(p2 - rotationAmount);
      break;

    case 4:
      st1.moveTo(p1 - rotationAmount);
      st2.moveTo(p2 + rotationAmount);
      break;

    case 5:
      st1.moveTo(p1 + rotationAmount*2);
      st2.moveTo(p2);
      break;

    case 6:
      st1.moveTo(p1);
      st2.moveTo(p2 - rotationAmount*2);
      break;

    default:
      Serial.println("Invalid CMD");
      moving = false;
      break;
  }
}