// motors
const int DIR_MOTOR_L = 7;
const int DIR_MOTOR_R = 8;
const int SPEED_MOTOR_L = 9;
const int SPEED_MOTOR_R = 10;

// encoders
const int ENCODER_L_A = 4;
const int ENCODER_L_B = 2;
const int ENCODER_R_A = 3;
const int ENCODER_R_B = 5;
volatile int encoderCount = 0;

void setup() {
  Serial.begin(9600);

  pinMode(DIR_MOTOR_L, OUTPUT);
  pinMode(DIR_MOTOR_R, OUTPUT);
  pinMode(SPEED_MOTOR_L, OUTPUT);
  pinMode(SPEED_MOTOR_R, OUTPUT);

  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_B), readEncoder, CHANGE);
  removeInterrupt();
}

void loop() {
  // go forward
  digitalWrite(DIR_MOTOR_L, HIGH);
  analogWrite(SPEED_MOTOR_L, 150);

  digitalWrite(DIR_MOTOR_R, LOW);
  analogWrite(SPEED_MOTOR_R, 150);
}

void setMotors(int dir, int /Users/gyryk/Downloads/micromouse.inospeed){
  analogWrite(SPEED_MOTOR_L, speed);
  analogWrite(SPEED_MOTOR_L, speed);

  if(dir == 1){
    fast_write_pin();
  }
  if(dir == 2){

  }
}

void readEncoder() {
  if(digitalRead() == HIGH) {
    encoderCount++;
  }
  else {
    encoderCount--;
  }
}
