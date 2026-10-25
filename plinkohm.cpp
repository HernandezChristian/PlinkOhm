#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// Initialize 1602 LCD on Parallel Pins
LiquidCrystal lcd(36, 38, 48, 45, 43, 41);


// Initialize OLED on I2C (Pins 20 SDA, 21 SCL)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


Servo motor1;   // top motor (tips resistor left/right)
Servo motor2;   // bottom left gate
Servo motor3;   // bottom right gate


int leftBin = 0;
int midLeftBin = 0;
int midRightBin = 0;
int rightBin = 0;


const int motor1Pin = 3;
const int motor2Pin = 4;
const int motor3Pin = 5;
const int pinOut = 13;


// ---- LED Pins ----
const int led10  = 22;
const int led100 = 24;
const int led220 = 26;
const int led1k  = 28;


// ---- Conveyor (28BYJ-48 + ULN2003), runs independently on Timer2 ----
const int CONV_IN1 = 8;
const int CONV_IN2 = 9;
const int CONV_IN3 = 10;
const int CONV_IN4 = 11;

// Milliseconds between half-steps. Bigger = slower.
// 4096 half-steps per rev: 50 -> ~205 s/rev, 200 -> ~820 s/rev. Keep at 5 or higher.
const unsigned int CONV_STEP_INTERVAL_MS = 15;
const bool CONV_REVERSE = true;   // set true to run the other way

const byte CONV_SEQ[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

volatile int convStepIndex = 0;
volatile unsigned int convTicks = 0;

// Runs every 1 ms in the background, no matter what loop() is doing
ISR(TIMER2_COMPA_vect) {
  convTicks++;
  if (convTicks < CONV_STEP_INTERVAL_MS) return;
  convTicks = 0;

  digitalWrite(CONV_IN1, CONV_SEQ[convStepIndex][0]);
  digitalWrite(CONV_IN2, CONV_SEQ[convStepIndex][1]);
  digitalWrite(CONV_IN3, CONV_SEQ[convStepIndex][2]);
  digitalWrite(CONV_IN4, CONV_SEQ[convStepIndex][3]);

  convStepIndex += CONV_REVERSE ? -1 : 1;
  if (convStepIndex > 7) convStepIndex = 0;
  if (convStepIndex < 0) convStepIndex = 7;
}

void startConveyor() {
  pinMode(CONV_IN1, OUTPUT);
  pinMode(CONV_IN2, OUTPUT);
  pinMode(CONV_IN3, OUTPUT);
  pinMode(CONV_IN4, OUTPUT);

  noInterrupts();
  TCCR2A = (1 << WGM21);              // CTC mode
  TCCR2B = (1 << CS22);               // prescaler 64
  OCR2A = 249;                        // 16 MHz / 64 / 250 = 1000 Hz (1 ms)
  TIMSK2 = (1 << OCIE2A);             // enable compare-match interrupt
  interrupts();
}
// ---------------------------------------------------------------------


// ---- Motion Settings ----
const int NEUTRAL = 90;
const int STEP_DEG = 2;        // degrees per step (bigger = faster)
const int STEP_DELAY = 10;     // ms between steps (smaller = faster)
const int DROP_TIME = 300;     // ms to let the resistor fall at the tipped position
// -------------------------


int sensorValue = 0;


// Helper function to calculate measured resistance based on 220-ohm reference
float calculateOhms(float voltage) {
  if (voltage <= 0.1) return 0.0;
  return 220.0 * ((5.0 - voltage) / voltage);
}


// Keep 1602 LCD dedicated to overall bin tallying
void updateLCDDisplay() {
  lcd.setCursor(0, 0);
  lcd.print("10:");
  lcd.print(rightBin);
  lcd.print("   ");


  lcd.setCursor(8, 0);
  lcd.print("100:");
  lcd.print(midRightBin);
  lcd.print("   ");


  lcd.setCursor(0, 1);
  lcd.print("220:");
  lcd.print(midLeftBin);
  lcd.print("   ");


  lcd.setCursor(8, 1);
  lcd.print("1k:");
  lcd.print(leftBin);
  lcd.print("   ");
}


// MODIFICATION: Dedicated OLED Telemetry Screen showing Live Voltage, Calculated Ohms, and Status
void updateOLEDTelemetry(float voltage, const char* status) {
  oled.clearDisplay();
 
  // Header
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(14, 0);
  oled.print("LIVE SENSOR TELEMETRY");
  oled.drawFastHLine(0, 10, 128, SSD1306_WHITE);


  // Status Indicator
  oled.setCursor(0, 15);
  oled.print("STATUS: ");
  oled.print(status);


  // Live Voltage Readout
  oled.setCursor(0, 28);
  oled.print("VOLTAGE: ");
  oled.setTextSize(2);
  oled.setCursor(55, 25);
  oled.print(voltage, 2);
  oled.setTextSize(1);
  oled.print("V");


  // Calculated Resistance Readout
  float ohms = calculateOhms(voltage);
  oled.setCursor(0, 48);
  oled.print("RESIST: ");
 
  if (ohms >= 900.0) {
    oled.print(ohms / 1000.0, 2);
    oled.print(" k");
    oled.write(0xF4); // Ohm symbol / text label
  } else {
    oled.print((int)ohms);
    oled.print(" ");
    oled.write(0xF4);
  }


  oled.display();
}


// Move one servo to a target angle in small steps
void moveServo(Servo &s, int target) {
  int pos = s.read();
  while (pos != target) {
    if (abs(target - pos) <= STEP_DEG) pos = target;
    else pos += (target > pos) ? STEP_DEG : -STEP_DEG;
    s.write(pos);
    delay(STEP_DELAY);
  }
}


void setup() {
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);


  pinMode(pinOut, OUTPUT);
  digitalWrite(pinOut, LOW);


  // Configure LED pins as outputs
  pinMode(led10, OUTPUT);
  pinMode(led100, OUTPUT);
  pinMode(led220, OUTPUT);
  pinMode(led1k, OUTPUT);


  // Turn off all LEDs initially
  digitalWrite(led10, LOW);
  digitalWrite(led100, LOW);
  digitalWrite(led220, LOW);
  digitalWrite(led1k, LOW);


  // Set initial servo positions
  motor1.write(100);
  motor2.write(NEUTRAL);
  motor3.write(NEUTRAL);


  Serial.begin(9600);


  // Start 1602 LCD
  lcd.begin(16, 2);


  // Start OLED Display (try 0x3C or 0x3D)
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED allocation failed"));
  }


  updateLCDDisplay();
  updateOLEDTelemetry(0.0, "READY");


  startConveyor();   // belt now runs on its own in the background
}


void loop() {
  updateOLEDTelemetry(0.0, "WAITING...");


  // 1. Measure voltage (waits until a resistor is placed across contacts)
  digitalWrite(pinOut, HIGH);
  delay(10); // Small pause for signal to stabilize


  while (sensorValue <= 60) {
    sensorValue = analogRead(A0);
  }


  float voltage = sensorValue * (5.0 / 1023.0);
  sensorValue = 0;
  digitalWrite(pinOut, LOW); // Turn off measuring voltage during motor move


  // MODIFICATION: Update OLED immediately with live voltage reading & target calculation
  updateOLEDTelemetry(voltage, "MEASURED!");
  delay(200);


  int topTarget = 100;
  int gate2Target = NEUTRAL;
  int gate3Target = NEUTRAL;


  // 2. Select route, bin counters, and indicator LED
  if (voltage > 0.3 && voltage < 1.3) {           // 1k Resistor - 0.9
    topTarget = 0;
    gate2Target = 135;
    digitalWrite(led1k, HIGH);
    leftBin++;
  }
  else if (voltage >= 1.3 && voltage < 2.5) {      // 220 Resistor - 2.0
    topTarget = 0;
    gate2Target = 45;
    digitalWrite(led220, HIGH);
    midLeftBin++;
  }
  else if (voltage >= 2.5 && voltage < 2.85) {      // 100 Resistor - 2.7
    topTarget = 180;
    gate3Target = 135;
    digitalWrite(led100, HIGH);
    midRightBin++;
  }
  else if (voltage >= 2.85) {                       // 10 Resistor - 2.9
    topTarget = 180;
    gate3Target = 45;
    digitalWrite(led10, HIGH);
    rightBin++;
  }


  updateLCDDisplay(); // Update 1602 LCD bin counts
  updateOLEDTelemetry(voltage, "SORTING...");


  // 3. Set bottom gates first to form Plinko channel
  moveServo(motor2, gate2Target);
  moveServo(motor3, gate3Target);
  delay(100);


  // 4. Tip top motor to drop the resistor
  moveServo(motor1, topTarget);
  delay(DROP_TIME);


  // 5. Reset top motor back to NEUTRAL
  moveServo(motor1, 100);


  // 6. Reset bottom gates back to NEUTRAL
  moveServo(motor2, NEUTRAL);
  moveServo(motor3, NEUTRAL);


  // 7. Keep LED lit briefly so user sees result, then reset LEDs
  delay(1000);
  digitalWrite(led10, LOW);
  digitalWrite(led100, LOW);
  digitalWrite(led220, LOW);
  digitalWrite(led1k, LOW);


  // 8. Wait until the contact pads return to zero (prevents phantom dumps)
    // 8. Wait until the contact pads return to zero (prevents phantom dumps)
  digitalWrite(pinOut, HIGH);
  delay(10);
  while (true) {
    int raw = analogRead(A0 > 60);
    updateOLEDTelemetry(raw * (5.0 / 1023.0), "CLEARING...");
    Serial.println(raw);
    if (raw <= 60) break;
    delay(50);
  }
  digitalWrite(pinOut, LOW);


  delay(500); // Short debounce before accepting next resistor
}

