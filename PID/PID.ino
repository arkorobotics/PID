// LED Variables
const byte ledPin = 13;
volatile byte state = LOW;

// Encoder Variables
const byte encoder_input = 2;
unsigned long encoder_count = 10000;
unsigned long current_time = 0;
unsigned long last_time = 0;

// Motor Variables
byte AI0_pin = 6;
byte AI1_pin = 7; 
byte motor_pin = 5;
byte dir = 0;
byte motor_output = 0;
float velocity = 0;

// PID Variables
float target = 0.5; 
float P_gain = 100;
float I_gain = 500;
float D_gain = 0;
float error = target - encoder_count;
float last_error = 0;
float sum_of_error = 0;
float pid_output = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(encoder_input, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_input), encoderTick, CHANGE);

  // Set Motor Direction
  digitalWrite(AI0_pin, dir);
  digitalWrite(AI1_pin, !dir);
  // Set Motor Speed to Zero
  analogWrite(motor_pin, motor_output);
  
  Serial.begin(115200);
}

void loop() {
  digitalWrite(ledPin, state);
  
  Serial.print(target*100);
  Serial.print(" ");
  Serial.println(velocity*100);

  // Update
  velocity = 420.0/encoder_count;
  
  last_error = error;
  error = target - velocity;
  sum_of_error = sum_of_error + error;
  if(sum_of_error > 1023)
  {
    sum_of_error = 1023;
  }
  
  // PID Control
  pid_output = P_gain*error + I_gain*sum_of_error;
  if(pid_output/4 > 254)
  {
    motor_output = 255;
  }
  else
  {
    motor_output = pid_output/4;
  }
  analogWrite(motor_pin, motor_output);
  
  // Reset Encoder Count (to achieve velocity)
  //encoder_count = 0;

  delay(50);
}

void encoderTick() {
  state = !state;
  current_time = micros();
  encoder_count = current_time - last_time;
  last_time = current_time;
}
