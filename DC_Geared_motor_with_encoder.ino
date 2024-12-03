#include <Wire.h>
#include <MPU6050.h>
#include <LiquidCrystal.h>

// Motor Control Pins
#define ENA 11   // PWM for motor speed
#define IN1 9    // Motor direction pin 1
#define IN2 10   // Motor direction pin 2

// Encoder Pins
#define C1 2     // Encoder channel A (interrupt pin)
#define C2 3     // Encoder channel B (interrupt pin)

// Joystick Pins
#define JOY_Y A1 // Joystick Y-axis for direction (VRY)
#define SW 8     // Joystick switch button (optional)

// LCD Pins
#define RS 4     // Register Select pin
#define E 5      // Enable pin
#define D4 6     // Data Pin 4
#define D5 7     // Data Pin 5
#define D6 12    // Data Pin 6
#define D7 13    // Data Pin 7

// Encoder Variables
volatile long encoderCount = 0;  // Tracks the number of pulses
int lastEncoded = 0;              // Previous encoded state
int lastCount = 0;                // Last encoder count for direction tracking

// Motor State
bool motorRunning = false;        // Track motor running state

// MPU6050 Object (optional; can be removed if not used)
MPU6050 mpu;

// LCD Object
LiquidCrystal lcd(RS, E, D4, D5, D6, D7);

// Function Prototypes
void setupMotorPins();
void setupEncoderPins();
void setupJoystickPins();
void setMotorDirection(bool forward);
void encoderISR();

void setup() {
  Serial.begin(9600);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  } else {
    Serial.println("MPU6050 connected");
  }

  // Initialize motor, encoder, joystick, and LCD pins
  setupMotorPins();
  setupEncoderPins();
  setupJoystickPins();
  
  // Initialize the LCD
  lcd.begin(16, 2); // Set up the LCD's number of columns and rows
  lcd.print("Motor Control"); // Initial message

  Serial.println("System initialized.");
}

void loop() {
  // Check if the joystick button (SW) is pressed
  if (digitalRead(SW) == LOW) {
    // Toggle motor running state
    motorRunning = !motorRunning;
    // Debounce delay
    delay(300);  // Add a delay to debounce the button press
  }

  // Read joystick Y-axis value for speed control
  int joyY = analogRead(JOY_Y);  // Read Y-axis for speed
  int motorSpeed = map(joyY, 0, 1023, -255, 255); // Map to speed range (-255 to 255)

  if (motorRunning) {
    if (abs(motorSpeed) < 20) { // Threshold for stopping the motor
      analogWrite(ENA, 0);  // Set speed to 0 to stop the motor
    } else {
      // Set motor speed and direction
      if (motorSpeed > 0) {
        setMotorDirection(true);  // Forward
        analogWrite(ENA, motorSpeed);  // Set motor speed
      } else {
        setMotorDirection(false); // Reverse
        analogWrite(ENA, -motorSpeed); // Set motor speed
      }
    }

    // Print encoder position and direction
    Serial.print("Encoder Position: ");
    Serial.print(encoderCount / 810);
    Serial.print("    ");
    
    // Update LCD display
    lcd.clear();
    lcd.setCursor(0, 0); // Set cursor to first row
    lcd.print("Pos: ");   // Print position label
    lcd.print(encoderCount / 810); // Print encoder position
    lcd.setCursor(0, 1); // Set cursor to second row
    if (encoderCount > lastCount) {
      lcd.print("Dir: CCW   "); // Print counterclockwise
      Serial.println("Counterclockwise");
    } else if (encoderCount < lastCount) {
      lcd.print("Dir: CW    "); // Print clockwise
      Serial.println("Clockwise");
    } else {
      lcd.print("Dir: Stable "); // Print stable
      Serial.println("Stable");
    }
    lastCount = encoderCount;  // Update lastCount for next loop iteration

  } else {
    // Stop the motor if it's not running
    analogWrite(ENA, 0);  // Set speed to 0 to stop the motor
    Serial.println("Motor Stopped");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motor Stopped"); // Print motor stopped message
  }
}

// Function to initialize motor pins
void setupMotorPins() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

// Function to initialize encoder pins and interrupts
void setupEncoderPins() {
  pinMode(C1, INPUT_PULLUP);
  pinMode(C2, INPUT_PULLUP);

  // Attach interrupts for the encoder
  attachInterrupt(digitalPinToInterrupt(C1), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(C2), encoderISR, CHANGE);
}

// Function to initialize joystick pins
void setupJoystickPins() {
  pinMode(JOY_Y, INPUT);
  pinMode(SW, INPUT_PULLUP); // Use internal pull-up resistor for switch
}

// Function to set motor direction
void setMotorDirection(bool forward) {
  if (forward) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

// Interrupt Service Routine for encoder
void encoderISR() {
  int MSB = digitalRead(C1);  // Read encoder channel A
  int LSB = digitalRead(C2);  // Read encoder channel B

  int encoded = (MSB << 1) | LSB;  // Combine both signals
  int sum = (lastEncoded << 2) | encoded;  // Compute state transition

  // Determine direction and adjust encoder count
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderCount--;
  }

  lastEncoded = encoded;  // Update the last encoded state
}