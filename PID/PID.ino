// Simple PID Controller
// By: Ara Kourchians
// ----------------------------------------------------------------------------

// LED Variables
const byte ledPin = 13;

// Encoder Variables
#define TICK_NULL 0
#define TICK_A 1
#define TICK_B 2
#define FORWARD false
#define BACKWARD true
const byte encoder_A_pin = 2;
const byte encoder_B_pin = 3;
volatile unsigned char last_tick = TICK_NULL;
long position_ticks = 0;
unsigned long current_time = 0;
unsigned long last_time = 0;

// State Variables
float position = 0.0; // degrees
float velocity = 0.0; // rpm

// PID Variables
float target = 0.5; 
float P_gain = 1;
float I_gain = 0.01;
float D_gain = 0;
float error = 0;
float last_error = 0;
float sum_of_error = 0;
float pid_output = 0;

// Motor Variables
byte AI0_pin = 6;
byte AI1_pin = 7; 
byte motor_pin = 5;
byte motor_output = 0;
volatile boolean direction = FORWARD;

// Runs first and once
void setup() 
{
  pinMode(ledPin, OUTPUT);
  
  // Setup encoder interrupt routines
  pinMode(encoder_A_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_A_pin), encoderTickA, CHANGE);

  pinMode(encoder_B_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_B_pin), encoderTickB, CHANGE);

  // Set Motor Direction
  setDirection(FORWARD);

  // Set Motor Speed to Zero
  analogWrite(motor_pin, motor_output);
  
  // Initialize Serial Bus
  Serial.begin(115200);
}

// Main Loop
void loop() 
{
  digitalWrite(ledPin, direction);

  // Update Position and Velocity
  position = ((float)position_ticks)*(360/1800.0);
  velocity = (0.000555)/(0.000001*((float)(current_time - last_time + 1)));

  // PID Calculations
  last_error = error;           // Store the last error for derivative
  error = target - velocity;    // Calculate new velocity error
  sum_of_error = sum_of_error + error;    // Accumulate error
  if(sum_of_error > 1023)                 // Saturate to prevent large I term
  {
    sum_of_error = 1023;
  }
  
  // PID Control
  pid_output = P_gain*error + I_gain*sum_of_error;  // PID

  if(pid_output > 1.0)                              // Hard limit PID output
  {
    pid_output = 1.0;
  }
  else if(pid_output < -1.0)
  {
    pid_output = -1.0;
  }

  if(pid_output >= 0)                               // Set direction
  {
    setDirection(FORWARD);
  }
  else if(pid_output < 0)
  {
    setDirection(BACKWARD);
  }

  motor_output = (unsigned char)(255.0*abs(pid_output)); // Calculate

  analogWrite(motor_pin, motor_output);             // Apply output to motor 
  
  // Print out data
  Serial.print(target);
  Serial.print(" ");
  Serial.println(velocity);

  delay(50);
}

void setDirection(boolean input_direction)
{
  // Set Motor Direction
  digitalWrite(AI0_pin, input_direction);
  digitalWrite(AI1_pin, !input_direction);
}

void encoderTickA() 
{
  last_time = current_time;
  current_time = micros();
  
  if(last_tick == TICK_A) 
  {
    direction = !direction;
  }
  last_tick = TICK_A;

  if(direction == FORWARD) 
  {
    position_ticks++;    
  }
  else if(direction == BACKWARD) 
  {
    position_ticks--;
  }
}

void encoderTickB() 
{
  last_time = current_time;
  current_time = micros();

  if(last_tick == TICK_B)
  {
    direction = !direction;
  }
  last_tick = TICK_B;

  if(direction == FORWARD) 
  {
    position_ticks++;    
  }
  else if(direction == BACKWARD) 
  {
    position_ticks--;
  }
}
