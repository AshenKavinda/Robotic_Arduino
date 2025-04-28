#include <HX711_ADC.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Load Cell Configuration
const int HX711_DOUT = A1;
const int HX711_SCK = A2;
HX711_ADC loadCell(HX711_DOUT, HX711_SCK);
const float CALIBRATION_FACTOR = 21.71;

// Motor Control
const int MOTOR_IN1 = 9;
const int MOTOR_IN2 = 10;
const int MOTOR_SPEED_IN = 11;
const int MOTOR_SPEED = 255;
bool motorRunning = false;
bool precisionPhase = false;
// Keypad Configuration
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {2, 3, 4, 5};
byte colPins[COLS] = {6, 7, 8};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 16, 2);

// System State
enum State { INPUT_STATE, TARE_STATE, DISPENSE_STATE, COMPLETE_STATE };
State currentState = INPUT_STATE;

// Weight Variables
float targetWeight = 0.0;
float currentWeight = 0.0;
String inputString = "";

// Timing Variables
unsigned long lastUpdate = 0;
unsigned long motorPulseStart = 0;
unsigned long completeTime = 0;
const int DISPLAY_INTERVAL = 500;
const int LOADCELL_INTERVAL = 200;

void debugPrint(String message) {
  Serial.print("[DEBUG] ");
  Serial.println(message);
}

void debugPrintState() {
  Serial.print("State: ");
  switch(currentState) {
    case INPUT_STATE: Serial.println("INPUT"); break;
    case TARE_STATE: Serial.println("TARE"); break;
    case DISPENSE_STATE: Serial.println("DISPENSE"); break;
    case COMPLETE_STATE: Serial.println("COMPLETE"); break;
  }
}

void setup() {
  Serial.begin(9600);
  debugPrint("System initialization started");
  
  // Initialize Load Cell
  loadCell.begin();
  loadCell.start(2000, true);
  if(loadCell.getTareTimeoutFlag() || loadCell.getSignalTimeoutFlag()) {
    debugPrint("ERROR: Load cell initialization failed");
    lcdPrint("Sensor Error!");
    while(1);
  }
  loadCell.setCalFactor(CALIBRATION_FACTOR);
  debugPrint("Load cell initialized");

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcdPrint("Enter Weight (g):");
  debugPrint("LCD initialized");

  // Initialize Motor Control
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  analogWrite(MOTOR_SPEED_IN, MOTOR_SPEED);
  stopMotor();
  debugPrint("Motor control initialized");

  debugPrint("Setup complete");
  debugPrintState();
}

void loop() {
  switch(currentState) {
    case INPUT_STATE:
      handleInput();
      break;
    case TARE_STATE:
      handleTare();
      break;
    case DISPENSE_STATE:
      handleDispense();
      break;
    case COMPLETE_STATE:
      handleComplete();
      break;
  }
}

void handleInput() {
  char key = keypad.getKey();
  
  if (key) {
    debugPrint("Key pressed: " + String(key));
    
    if (key == '#') {
      if (inputString.length() > 0) {
        targetWeight = inputString.toFloat();
        debugPrint("Target weight set to: " + String(targetWeight) + " g");
        if(targetWeight > 0) {
          currentState = TARE_STATE;
          lcdPrint("Taring...");
          debugPrint("Moving to TARE state");
          debugPrintState();
        }
      }
    }
    else if (key == '*') {
      inputString = "";
      lcdPrint("Enter Weight (g):");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      debugPrint("Input reset");
    }
    else if ((isdigit(key) || key == '.') && inputString.length() < 6) {
      inputString += key;
      updateDisplayInput();
      debugPrint("Current input: " + inputString);
    }
  }
}

void handleTare() {
  static bool taring = false;
  static unsigned long tareStart = millis();
  
  if (!taring) {
    debugPrint("Starting tare...");
    loadCell.tare(); // This doesn't return a value
    taring = true;
    tareStart = millis();
    return; // Exit after initiating tare
  }

  // Add timeout check
  if(millis() - tareStart > 10000) { // 10 second timeout
    debugPrint("ERROR: Tare timeout!");
    if(loadCell.getTareTimeoutFlag()) debugPrint("Tare timeout flag set");
    if(loadCell.getSignalTimeoutFlag()) debugPrint("Signal timeout flag set");
    resetSystem();
    return;
  }
  if(loadCell.update()) {
    if (loadCell.getTareStatus()) {
      debugPrint("Tare completed successfully");
      taring = false;
      currentState = DISPENSE_STATE;
      startMotor();
    } else {
      debugPrint("Tare in progress...");
    }
  } else {
    debugPrint("Tare not updated.....");
  }
}

void handleDispense() {
  static unsigned long lastWeightUpdate = 0;
  static int pulseInterval = 100;

  // Update weight measurement
  if (millis() - lastWeightUpdate >= LOADCELL_INTERVAL) {
    if (loadCell.update()) {
      currentWeight = abs(loadCell.getData());
      Serial.print("Current weight: ");
      Serial.print(currentWeight);
      Serial.print(" / Target: ");
      Serial.println(targetWeight);
    }
    lastWeightUpdate = millis();
  }

  // Update display
  if (millis() - lastUpdate >= DISPLAY_INTERVAL) {
    lcd.setCursor(0, 1);
    lcd.print(currentWeight, 2);
    lcd.print("g/");
    lcd.print(targetWeight, 2);
    lcd.print("g   ");
    lastUpdate = millis();
  }

  if(targetWeight<=600) {
    precisionPhase = true;
  }

  if(currentWeight >= targetWeight - 200 && precisionPhase) {
    handlePrecisionDispensing(20);
  } else if(precisionPhase)  {
    handlePrecisionDispensing(80);
  }
  
  // Motor control logic
  if (currentWeight >= targetWeight) {
    debugPrint("Target weight reached");
    stopMotor();
    currentState = COMPLETE_STATE;
    lcdPrint("Thank You!");
    completeTime = millis();
    debugPrint("Moving to COMPLETE state");
    debugPrintState();
  }
  else if (currentWeight > targetWeight - 500 && !precisionPhase) {
    precisionPhase = true;
    debugPrint("Entering precision phase");
  }
}

void handlePrecisionDispensing(long duration) {
  static unsigned long lastPulseTime = 0;
  static bool motorState = false;  // Tracks whether motor is currently ON
  long onDuration = duration;  // Motor ON time (ms)
  const unsigned long offDuration = 200; // Motor OFF time (ms)

  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - lastPulseTime;

  if (motorState) {
    // Motor is currently ON - check if time to turn OFF
    if (elapsed >= onDuration) {
      digitalWrite(MOTOR_IN1, LOW);
      motorState = false;
      lastPulseTime = currentTime;
      debugPrint("Motor pulse OFF (precision phase)");
    }
  } else {
    // Motor is currently OFF - check if time to turn ON
    if (elapsed >= offDuration) {
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
      motorState = true;
      lastPulseTime = currentTime;
      debugPrint("Motor pulse ON (precision phase)");
    }
  }
}

void handleComplete() {
  if (millis() - completeTime >= 5000) {
    debugPrint("System resetting");
    resetSystem();
  }
}

void startMotor() {
  if (!precisionPhase) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    debugPrint("Motor started (full speed)");
  }
  motorRunning = true;
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  motorRunning = false;
  debugPrint("Motor stopped");
}

void resetSystem() {
  targetWeight = 0.0;
  currentWeight = 0.0;
  inputString = "";
  precisionPhase = false;
  stopMotor();
  currentState = INPUT_STATE;
  lcdPrint("Enter Weight (g):");
  debugPrint("System reset complete");
  debugPrintState();
}

void lcdPrint(const char* msg) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(msg);
  debugPrint("LCD display: " + String(msg));
}

void updateDisplayInput() {
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(inputString);
}