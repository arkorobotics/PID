// Simple PID Controller
// By: Ara Kourchians
// ----------------------------------------------------------------------------

// LED Variables
const byte ledPin = 13;

// Encoder Variables
#define TICK_NULL 0
#define TICK_A 1
#define TICK_B 2
#define FORWARD 0
#define BACKWARD 1
const byte encoder_A_pin = 3;
const byte encoder_B_pin = 2;
volatile static unsigned char last_tick_a = TICK_NULL;
volatile static unsigned char last_tick_b = TICK_NULL;
volatile long position_ticks = 0;
volatile unsigned long current_time = 0;
volatile unsigned long last_time = 0;

// State Variables
volatile static unsigned char cmd_direction = FORWARD;
volatile float position = 0.0; // degrees
volatile float velocity = 0.0; // rpm

// PID Variables
volatile const float target = 90.0; 
volatile float P_gain = 0.01;
volatile float I_gain = 0.001;
volatile float D_gain = 0.00;
volatile float error = 0;
volatile float last_error = 0;
volatile float sum_of_error = 0;
volatile float pid_output = 0;

// Motor Variables
const byte AI0_pin = 6;
const byte AI1_pin = 7; 
const byte motor_pin = 9;
volatile static unsigned char motor_output = 0;
volatile static unsigned char read_direction = FORWARD;

// Runs first and once
void setup() 
{
  pinMode(ledPin, OUTPUT);
  
  // Setup encoder interrupt routines
  last_tick_a = digitalRead(encoder_A_pin) ? 1 : 0;
  last_tick_b = digitalRead(encoder_B_pin) ? 1 : 0;

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
  digitalWrite(ledPin, read_direction);
  
  // Update Position and Velocity
  position = ((float)position_ticks)*(360/1800.0);
  velocity = (0.000555)/(0.000001*((float)(current_time - last_time + 1)));

  // PID Calculations
  last_error = error;           // Store the last error for derivative term
  error = target - position;    // Calculate new error
  sum_of_error = sum_of_error + error;    // Accumulate error for integral term
  if(sum_of_error > 1000)                 // Saturate to prevent large integral term
  {
    sum_of_error = 1000;
  }
  
  // PID Controller
  pid_output = P_gain*error + I_gain*sum_of_error + D_gain*(error - last_error);

  // Hard limit PID output
  if(pid_output > 1.0)                                   
  {
    pid_output = 1.0;
  }
  if(pid_output < -1.0)
  {
    pid_output = -1.0;
  }

  if(pid_output >= 0.0 && cmd_direction == BACKWARD)     // Set direction
  {
    setDirection(FORWARD);
  }
  if(pid_output < 0.0 && cmd_direction == FORWARD)
  {
    setDirection(BACKWARD);
  }
  
  motor_output = (unsigned char)(254.0*abs(pid_output)); // Calculate
  analogWrite(motor_pin, motor_output);                  // Apply output to motor 
  
  // Print out data for plotting
  Serial.print(target);
  Serial.print(" ");
  Serial.println(error);
  
  delay(50);  // 20Hz Update Rate
              // Do not change this time delay. Changing this time delay
              // will effect the controller and the response of the system, given
              // a set of PID gain values. 
              //
              // TODO: This loop should actually be in a timer interrupt
              // with a fixed/non-maskable update rate. 
}

void setDirection(unsigned char input_direction)
{
  // Set Motor Direction
  cmd_direction = input_direction ? 1 : 0;
  digitalWrite(AI0_pin, (~cmd_direction & 0x01));
  digitalWrite(AI1_pin, cmd_direction);
}

void encoderTickA() 
{
  unsigned char tick_a = digitalRead(encoder_A_pin) ? 1 : 0;
  unsigned char tick_b = digitalRead(encoder_B_pin) ? 1 : 0;

  // XOR ticks to find direction
  unsigned char plus = tick_a ^ last_tick_b;
  unsigned char minus = tick_b ^ last_tick_a;

  if(plus) 
  {
    position_ticks++;
    read_direction = FORWARD;    
  }
  if(minus) 
  {
    position_ticks--;
    read_direction = BACKWARD;
  }

  last_tick_a = tick_a;
  last_tick_b = tick_b;

  last_time = current_time;
  current_time = micros();
}

void encoderTickB() 
{
  unsigned char tick_a = digitalRead(encoder_A_pin) ? 1 : 0;
  unsigned char tick_b = digitalRead(encoder_B_pin) ? 1 : 0;

  // XOR ticks to find direction
  unsigned char plus = tick_a ^ last_tick_b;
  unsigned char minus = tick_b ^ last_tick_a;

  if(plus) 
  {
    position_ticks++;
    read_direction = FORWARD;    
  }
  if(minus) 
  {
    position_ticks--;
    read_direction = BACKWARD;
  }

  last_tick_a = tick_a;
  last_tick_b = tick_b;

  last_time = current_time;
  current_time = micros();
}
