#include <AccelStepper.h>

const int stepperCount = 2;
const int dirPin[stepperCount] = {2,4};
const int stepPin[stepperCount] = {3,5};

// Motor Enable/Disable purpose
// Can add motor disable mode with this
const int enPin[stepperCount] = {6,7};
const int stepControlPin = 8;

const int fullStepRev = 200;
// Microstepping setting
// Fullstep -> 1
// Halfstep -> 2
// ...
// 1/16 step -> 16
const int microStepping = 16;
const int stepsPerRevolution = fullStepRev * microStepping;


// Higher number = faster
const int speeds[2] = {20*microStepping, 600*microStepping};

// Acceleration
const int accelSpeed = 800*microStepping;

AccelStepper* st[stepperCount];

String msg;
bool moving = false;

long revToSteps(float rev){
  return lround(rev * (float)stepsPerRevolution);
}

// Enable/Disable motor
void enableMotor(bool activate){
  for(int i = 0; i < stepperCount; i++){
    digitalWrite(enPin[i], activate ? LOW : HIGH);
  }
}

void setProfileMotor(float maxSpeed, float accelSpeeds){
  for(int i = 0; i < stepperCount; i++){
    st[i]->setMaxSpeed(maxSpeed);
    st[i]->setAcceleration(accelSpeeds);
  }
}

void setProfileMotorSep(int index, float maxSpeed, float accelSpeeds){
  st[index]->setMaxSpeed(maxSpeed);
  st[index]->setAcceleration(accelSpeeds);
  Serial.println("Motor: " + (String)index + " - Speed: " + (String)maxSpeed + "/Accel: " + (String)accelSpeeds);
}

int dirSet(bool dir, bool sep, int index){
  int base = dir ? +1 : -1;
  if(!sep) return base;

  if(index == 0) return base;
  if(index == 1) return -base;
  return base;
}

void stepperRot(bool dir, bool sep, float rev_L, float rev_R, int speed){
  long moveSteps_L = labs(revToSteps(rev_L));
  long moveSteps_R = labs(revToSteps(rev_R));
  if(moveSteps_L <= 0 || moveSteps_R <= 0) return;

  // enableMotor(true);
  // setProfileMotor(speed, accelSpeed);

  // Num. 0 => Left
  // Num. 1 => Right

  if(rev_L > rev_R){
    float adjustSpeed = rev_L/rev_R*speed ;
    float adjustAccel = rev_L/rev_R*accelSpeed ;
    setProfileMotorSep(0,adjustSpeed, adjustAccel);
    setProfileMotorSep(1,speed, accelSpeed);
  }else if(rev_L < rev_R){
    int adjustSpeed = rev_R/rev_L * speed ;
    float adjustAccel = rev_R/rev_L * accelSpeed ;
    setProfileMotorSep(1, adjustSpeed, adjustAccel);
    setProfileMotorSep(0,speed, accelSpeed);
  }else{
    setProfileMotor(speed, accelSpeed);
  }

  for(int i =0; i< stepperCount; i++){
    int dSet = dirSet(dir, sep, i);
    if(i == 0){
      long target = st[i]->currentPosition() + (long)dSet * moveSteps_L;
      st[i]->moveTo(target);
    }else{
      long target = st[i]->currentPosition() + (long)dSet * moveSteps_R;
      st[i]->moveTo(target);
    }
  }

  bool anyRunning;
  do{
    anyRunning = false;
    for(int i =0; i< stepperCount; i++){
      if(st[i]->distanceToGo() != 0){
        st[i]->run();
        anyRunning = true;
      }
    }
  }
  while (anyRunning);
}

void actionSeq(int cases){
  switch(cases){
    case 0:
      stepperRot(true, false, 0.5f,0.5f, speeds[0]);
      break;
    case 1:
      stepperRot(false, false, 0.5f,0.5f, speeds[0]);
      break;
    case 2:
      stepperRot(true, true, 0.25f,0.25f, speeds[0]);
      break;
    case 3:
      stepperRot(false, true, 0.25f,0.25f, speeds[0]);
      break;
    case 4:
      stepperRot(true, false, 0.5f,0.25f, speeds[0]);
      break;
    case 5:
      stepperRot(false, false, 0.5f,0.25f, speeds[0]);
      break;
    // Action Seq
    case 6:
      stepperRot(true, true, 0.25f, 0.25f, speeds[0]);
      stepperRot(false, true, 0.5f, 0.5f, speeds[1]);
      stepperRot(true, true, 0.25f, 0.25f, speeds[1]);
      stepperRot(true, false, 0.5f, 0.25f, speeds[0]);
      stepperRot(false, false, 0.5f, 0.25f, speeds[0]);
      stepperRot(true, false, 0.5f, 0.5f, speeds[0]);
      stepperRot(false, false, 0.5f, 0.5f, speeds[1]);
      break;
  
    default:
      Serial.println("CMD out fo range");
      moving = false;
      break;

  }
}


void setup() {
  Serial.begin(9600);



  for(int i=0; i<stepperCount; i++){
    pinMode(dirPin[i], OUTPUT);
    pinMode(stepPin[i], OUTPUT);
    if (enPin[i] != 255) {
      pinMode(enPin[i], OUTPUT);
      digitalWrite(enPin[i], LOW);
    }
  }
  pinMode(stepControlPin, OUTPUT);
  digitalWrite(stepControlPin, HIGH);

  for(int i=0; i<stepperCount; i++){
    st[i] = new AccelStepper(AccelStepper::DRIVER, stepPin[i], dirPin[i]);
    // Safe Pulse width for A4988
    st[i]->setMinPulseWidth(3);
    st[i]->setMaxSpeed(speeds[0]);
    st[i]->setAcceleration(accelSpeed);

  }  

  // This for changing the step mode
  // pinMode(ms1, OUTPUT);
  // pinMode(ms2, OUTPUT);
  // pinMode(ms3, OUTPUT);
  // digitalWrite(ms1,HIGH);
  // digitalWrite(ms2,HIGH);

  delay(2000);

    Serial.println("CMD Ready");



}

void loop() {

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

