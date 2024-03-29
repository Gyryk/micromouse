/** DO NOT CHANGE THE FOLLOWING DEFINITONS - From UKMARS MazeRunner GitHub **/
/** =============================================================================================================== **/
#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
#define __digitalPinToPortReg(P) (((P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define __digitalPinToDDRReg(P) (((P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define __digitalPinToPINReg(P) (((P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define __digitalPinToBit(P) (((P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P)-8 : (P)-14))

// general macros/defines
#if !defined(BIT_READ)
#define BIT_READ(value, bit) ((value) & (1UL << (bit)))
#endif
#if !defined(BIT_SET)
#define BIT_SET(value, bit) ((value) |= (1UL << (bit)))
#endif
#if !defined(BIT_CLEAR)
#define BIT_CLEAR(value, bit) ((value) &= ~(1UL << (bit)))
#endif
#if !defined(BIT_WRITE)
#define BIT_WRITE(value, bit, bitvalue) (bitvalue ? BIT_SET(value, bit) : BIT_CLEAR(value, bit))
#endif

#define fast_write_pin(P, V) BIT_WRITE(*__digitalPinToPortReg(P), __digitalPinToBit(P), (V));
#define fast_read_pin(P) ((int)(((BIT_READ(*__digitalPinToPINReg(P), __digitalPinToBit(P))) ? HIGH : LOW)))

#else
#define fast_write_pin(P, V) fast_write_pin(P, V)
#define fast_read_pin(P) digitalRead(P)
#endif

// this is what we added
#define MATRIX_SIZE 8
#define QUEUE_CAPACITY 64
#define WALL_THRESHOLD 32
#define END_POSITIONS_SIZE 4
#define CELL_SIZE 108

struct Cell {
  int x, y, distance;
};

enum Direction {
  NORTH,
  EAST,
  SOUTH,
  WEST
};

class Queue {
private:
  Cell arr[QUEUE_CAPACITY];
  int front, rear, count;

public:
  Queue() : front(0), rear(-1), count(0) {}

  bool isEmpty() { return count == 0; }

  bool isFull() { return count == QUEUE_CAPACITY; }

  void enqueue(Cell item) {
    if (isFull()) {
      // Serial.println("Queue Overflow");
      return;
    }
    rear = (rear + 1) % QUEUE_CAPACITY;
    arr[rear] = item;
    count++;
  }

  Cell dequeue() {
    if (isEmpty()) {
      // Serial.println("Queue Underflow");
      return {-1, -1}; // Return invalid point
    }
    Cell item = arr[front];
    front = (front + 1) % QUEUE_CAPACITY;
    count--;
    return item;
  }
};

/** =============================================================================================================== **/

/** DEFINE OUR PINS AND WHICH COMPONENTS THEY ARE CONNECTED TO **/
/** _______________________________________________________________________________________________________________ **/
const int ENCODER_R_A = 3; // ENCODER RIGHT A (ticks first when motor forward)
const int ENCODER_R_B = 5; // ENCODER RIGHT B (ticks first when motor backward) 

const int ENCODER_L_A = 4; // ENCODER LEFT A (ticks first when motor forward)
const int ENCODER_L_B = 2; // ENCODER LEFT B (ticks first when motor backward)

const int SPEED_MOTOR_L = 9; // PWM MOTOR LEFT 
const int SPEED_MOTOR_R = 10; // PWM MOTOR RIGHT 

const int DIR_MOTOR_L = 7; // DIRECTION MOTOR LEFT 
const int DIR_MOTOR_R = 8; // DIRECTION MOTOR RIGHT 

// 4 Way switch and push button
const int DIP_SWITCH = A6; 
/** _______________________________________________________________________________________________________________ **/

/* GLOBAL VARIABLES */

volatile int rightEncoderPos = 0; // Counts for right encoder ticks
volatile int leftEncoderPos = 0; // Counts for left encoder ticks

// Variables to help us with our PID
int prevTime = 0;
int prevError;
int errorIntegral;
bool switchOn;


// Define the start position as a constant
const Cell START = {7, 0, -1};
// Define the end positions as constants
const Cell END[] = {{2, 5, 0}, {2, 6, 0}, {1, 5, 0}, {1, 6, 0}};

// Phototransistors
const int RIGHT_SENSOR = A0;
const int FRONT_SENSOR = A1;
const int LEFT_SENSOR = A2;

// LEDs
const int EMITTERS = 12;

// The maze
Cell currentPosition;
Direction currentDirection;
int matrix[MATRIX_SIZE][MATRIX_SIZE];
bool hWalls[MATRIX_SIZE + 1][MATRIX_SIZE];
bool vWalls[MATRIX_SIZE][MATRIX_SIZE + 1];

void setup() {
  Serial.begin(9600);

  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);

  pinMode(SPEED_MOTOR_L, OUTPUT);
  pinMode(SPEED_MOTOR_R, OUTPUT);
  pinMode(DIR_MOTOR_L, OUTPUT);
  pinMode(DIR_MOTOR_R, OUTPUT);

  pinMode(EMITTERS, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_B), readEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), readEncoderRight, CHANGE);

  // Initialize horizontal walls
    for (int i = 0; i <= MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            hWalls[i][j] = false;
            if (i == 0 || i == MATRIX_SIZE) { 
                // Top and bottom boundary walls
                hWalls[i][j] = true;
            } else {
                hWalls[i][j] = false;
            }
        }
    }
    // Initialize vertical walls
    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j <= MATRIX_SIZE; j++) {
            vWalls[i][j] = false;
            if (j == 0 || j == MATRIX_SIZE) { 
                // Left and right boundary walls
                vWalls[i][j] = true;
            } else {
                vWalls[i][j] = false;
            }
        }
    }

  floodFill(matrix, hWalls, vWalls);
  currentPosition = START;
  currentDirection = NORTH;
}

/** INTERRUPT SERVICE ROUTINES FOR HANDLING ENCODER COUNTING USING STATE TABLE METHOD **/
void readEncoderLeft() {
  static uint8_t prevState = 0; 
  static uint8_t currState = 0; 
  static unsigned long lastTime = 0; 
  
  currState = (fast_read_pin(ENCODER_L_B) << 1) | fast_read_pin(ENCODER_L_A);
  
  unsigned long currentTime = micros();
  unsigned long deltaTime = currentTime - lastTime;
  lastTime = currentTime;
  
  // direction based on prev state
  uint8_t direction = (prevState << 2) | currState;
  switch(direction) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      leftEncoderPos++;
      break;
    case 0b0010:
    case 0b1100:
    case 0b0101:
    case 0b1011:
      leftEncoderPos--;
      break;

    default:
      break;
  }

  prevState = currState;
}
void readEncoderRight() {
  static uint8_t prevState = 0; 
  static uint8_t currState = 0; 
  static unsigned long lastTime = 0; 
  
  currState = (fast_read_pin(ENCODER_R_B) << 1) | fast_read_pin(ENCODER_R_A);
  
  unsigned long currentTime = micros();
  unsigned long deltaTime = currentTime - lastTime;
  lastTime = currentTime;
  
  uint8_t direction = (prevState << 2) | currState;
  switch(direction) {
    case 0b0100:
    case 0b1010:
    case 0b0111:
    case 0b1001:
      rightEncoderPos++;
      break;
    case 0b1000:
    case 0b0110:
    case 0b1101:
    case 0b0011:
      rightEncoderPos--;
      break;

    default:
      break;
  }
  
  prevState = currState;
}

/** Function to set motor speed and direction for BOTH motors
    @params dir - can either be HIGH or LOW for clockwise / counter clockwise rotation
    @params speed - analogWrite() value between 0-255
**/
void setMotors(int dir, int speed){
  analogWrite(SPEED_MOTOR_L, speed);
  analogWrite(SPEED_MOTOR_R, speed);
  
  if(dir == 1){
    fast_write_pin(DIR_MOTOR_L, HIGH);
    fast_write_pin(DIR_MOTOR_R, LOW);
  } else if (dir == -1){
    fast_write_pin(DIR_MOTOR_L, LOW);
    fast_write_pin(DIR_MOTOR_R, HIGH);
  } else{
    analogWrite(SPEED_MOTOR_L, 0);
    analogWrite(SPEED_MOTOR_R, 0);
  }
}

/** Function to make the robot travel for a certain amount of encoder ticks, calls upon setMotors at end
    @params dir - setPoint: The target value for how far we want to go (in encoder ticks)
    @params speed - analogWrite() value between 0-255
    @params kp - proportional gain, this is the main one you should be changing
    @params ki - intergral gain, use this for steady state errors
    @params kd - derivative gain, use this for overshoot and oscillation handling 
**/
void motorPID(int setPoint, float kp, float ki, float kd){
  int currentTime = micros();
  int deltaT = ((float)(currentTime - prevTime)) / 1.0e6; // time difference between ticks in seconds
  prevTime = currentTime; // update prevTime each loop 
  
  int error = setPoint - rightEncoderPos;
  int errorDerivative = (error - prevError) / deltaT;
  errorIntegral = errorIntegral + error*deltaT;

  float u = kp*error + ki*errorIntegral + kd*errorDerivative; 

  float speed = fabs(u);
  if(speed > 255){
    speed = 255;
  }

  int dir = 1;
  if (u < 0) {
    dir = -1; // Move backward
  } else {
    dir = 1; // Move forward
  }

  setMotors(dir, speed);
  prevError = 0;
}

//==============================================================================================
// YOUR HOMEWORK ASSIGNMENT: Create a function to convert from encoder ticks to centimeters!
int tickConvertToCm(int encoderTicks){
  return encoderTicks/6;
}
//==============================================================================================

void loop(){

  // Starter Code

  int dipSwitch = analogRead(DIP_SWITCH);
  // Serial.println(dipSwitch);
  if(dipSwitch > 1000){
    switchOn = true;
  }

  if(switchOn){
    delay(500); // Wait half a second after pressing the button to actually start moving, safety first!
    int setPoint = 108;
    float kp = 0.665;
    float ki = 0.22;
    float kd = 0.1;
    motorPID(setPoint, kp, ki, kd);

    Serial.println(setPoint);
    Serial.print(",");
    Serial.println(rightEncoderPos);
  }

  // wallDetection();
}


// floodfill to call whenever stuck (need to code STUCK LOGIC)
void floodFill(int arr[MATRIX_SIZE][MATRIX_SIZE], bool hWalls[MATRIX_SIZE + 1][MATRIX_SIZE], bool vWalls[MATRIX_SIZE][MATRIX_SIZE + 1]) {
  // Initialize matrix to -1
  for (int i = 0; i < MATRIX_SIZE; i++) {
    for (int j = 0; j < MATRIX_SIZE; j++) {
      arr[i][j] = -1;
    }
  }

  Queue queue;

  // Enqueue end positions with distance 0
  for (int i = 0; i < END_POSITIONS_SIZE; i++) {
    queue.enqueue(END[i]);
    arr[END[i].x][END[i].y] = 0;
  }

  while (!queue.isEmpty()) {
    Cell p = queue.dequeue();

    // Right neighbor
    if (p.x >= 0 && p.x < MATRIX_SIZE && p.y >= 0 && p.y < MATRIX_SIZE - 1) {
      if (!vWalls[p.x][p.y + 1] && arr[p.x][p.y + 1] == -1) { // Check no vertical wall to the right
        arr[p.x][p.y + 1] = p.distance + 1;
        queue.enqueue({p.x, p.y + 1, p.distance + 1});
      }
    }

    // Down neighbor
    if (p.x >= 0 && p.x < MATRIX_SIZE - 1 && p.y >= 0 && p.y < MATRIX_SIZE) {
      if (!hWalls[p.x + 1][p.y] && arr[p.x + 1][p.y] == -1) { // Check no horizontal wall below
        arr[p.x + 1][p.y] = p.distance + 1;
        queue.enqueue({p.x + 1, p.y, p.distance + 1});
      }
    }

    // Left neighbor
    if (p.x >= 0 && p.x < MATRIX_SIZE && p.y >= 1 && p.y < MATRIX_SIZE) {
      if (!vWalls[p.x][p.y] && arr[p.x][p.y - 1] == -1) { // Check no vertical wall to the left
        arr[p.x][p.y - 1] = p.distance + 1;
        queue.enqueue({p.x, p.y - 1, p.distance + 1});
      }
    }

    // Up neighbor
    if (p.x >= 1 && p.x < MATRIX_SIZE && p.y >= 0 && p.y < MATRIX_SIZE) {
      if (!hWalls[p.x][p.y] && arr[p.x - 1][p.y] == -1) { // Check no horizontal wall above
        arr[p.x - 1][p.y] = p.distance + 1;
        queue.enqueue({p.x - 1, p.y, p.distance + 1});
      }
    }
  }
}

void moveToCell(Cell cell, Cell start){
  if(cell.x < start.x){
    changeDirection(NORTH);
    int point = (start.x - cell.x) * CELL_SIZE;
    // add this to old setPoint to get new setPoint
    // how do i get the start cell?
  }
  else if(cell.x > start.x){
    changeDirection(SOUTH);
    int point = (cell.x - start.x) * CELL_SIZE;
  }
  else if(cell.y < start.y){
    changeDirection(WEST);
    int point = (start.y - cell.y) * CELL_SIZE;
  }
  else{
    changeDirection(EAST);
    int point = (cell.y - start.y) * CELL_SIZE;
  }


}

void changeDirection(Direction direction){
  currentDirection = direction;
}

void wallDetection(){
  digitalWrite(EMITTERS, HIGH);
  int right = analogRead(RIGHT_SENSOR);
  int front = analogRead(FRONT_SENSOR);
  int left = analogRead(LEFT_SENSOR);

  int addX;
  int addY;

  if(left > WALL_THRESHOLD){
    addX = (currentDirection == WEST) ? 1 : 0;
    addY = (currentDirection == SOUTH) ? 1 : 0;
    vWalls[currentPosition.x+addX][currentPosition.y+addY] = true;
  }
  if(front > WALL_THRESHOLD){
    addX = (currentDirection == SOUTH) ? 1 : 0;
    addY = (currentDirection == EAST) ? 1 : 0;
    hWalls[currentPosition.x+addX][currentPosition.y+addY] = true;
  }
  if(right > WALL_THRESHOLD){
    addX = (currentDirection == EAST) ? 1 : 0;
    addY = (currentDirection == NORTH) ? 1 : 0;
    vWalls[currentPosition.x+addX][currentPosition.y+addY] = true;
  }

  // Serial.println(left);
}

// Debugging prints
void printVWalls(){
  for (int i = 0; i < MATRIX_SIZE; i++) {
      for (int j = 0; j <= MATRIX_SIZE; j++) {
        Serial.print(vWalls[i][j] ? "T " : "F ");
      }
      Serial.println();
    }
    Serial.println();
}

void printHWalls(){
  for (int i = 0; i <= MATRIX_SIZE; i++) {
      for (int j = 0; j < MATRIX_SIZE; j++) {
        Serial.print(hWalls[i][j] ? "T " : "F ");
      }
      Serial.println();
    }
    Serial.println();
}

void printMatrix(){
  for (int i = 0; i < MATRIX_SIZE; i++) {
      for (int j = 0; j < MATRIX_SIZE; j++) {
        Serial.print(matrix[i][j]);
        Serial.print(" ");
      }
      Serial.println();
    }
    Serial.println();
}
