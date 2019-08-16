#include <Wire.h>
#include <VL53L0X.h>

#define MOTOR_PIN 9
#define FLOWMETER_PIN 2
#define FLOWMETER_INTERRUPT 0 // where: 0 for digital pin 2
#define FLOWMETER_FACTOR 2.083333 // where: Flow [mL/s] = freq [Hz] * FLOWMETER_FACTOR

// Flowmeter
volatile byte fl_pulseCount = 0;
unsigned long fl_oldTime = 0;
float fl_frequency = 0;

// Control Algorithm
float ca_fillSpeed = 0; // [mm/s]
uint16_t ca_dH = 0; // [mm]
float ca_dT = 0; // ca_dH/ca_fillSpeed [s]
bool ca_currentPartFilled = false;
uint16_t ca_partsNum = 1; // Total number of the vessel parts
uint16_t ca_k = 0; // Current part number
float ca_kVolume = 0; // The volume of the k part.
float ca_kNextFlux = 0; // ca_kVolume/ca_dT [ml/s]
//float ca_initialFlux = 0; // (S_min + S_max)*ca_dH / 2*ca_dT [mm3/s ~convert to~ ml/s :: divide by 1000]
unsigned long ca_startTime = 0;
unsigned long ca_endTime = 0;

// System
float sys_kp = 2.77648;
float sys_ki = 0.0925492;
float sys_integral = 0;
uint32_t sys_currentTime = 0;
uint32_t sys_previousTime = 0;
float sys_elapsedTime = 0;
float sys_currentFlux = 0; // [ml/s]
float sys_error = 0; // ca_kNextFlux - sys_currentFlux [ml/s]
long int sys_PI = 0;

// Sensor and Calibration
VL53L0X vl_sensor;
const float vl_rateLimit = 0.25; // 0.25 default
const uint32_t vl_timingBudget = 20000; // 33000 default
const uint16_t vl_correction = 10; // mm, 10 default
uint16_t vl_mainHeight = 0;
uint16_t vl_value = 0;

// Cup
uint16_t cup_height = 0;
uint16_t cup_minHeight = 20;
bool cup_inRange = false;
bool cup_placed = false;


void setup()
{
  Serial.begin(115200);
  while (!Serial) delay(1);
  Wire.begin();

  // Flowmeter
  pinMode(FLOWMETER_PIN, INPUT);
  digitalWrite(FLOWMETER_PIN, HIGH);
  attachInterrupt(FLOWMETER_INTERRUPT, fl_pulseCounter, FALLING);
  // Motor
  pinMode(MOTOR_PIN, OUTPUT);
  analogWrite(MOTOR_PIN, LOW); // PUMP :: 55-255
  // Sensor
  vl_sensor.init();
  vl_sensor.setTimeout(500);
  if (vl_sensor.setSignalRateLimit(vl_rateLimit)) Serial.println("Good Rate");
  if (vl_sensor.setMeasurementTimingBudget(vl_timingBudget)) Serial.println("Good Timing Budget");
  vl_sensor.startContinuous(); // sensor.startContinuous(100)
  Serial.println();

  // Detect Main Height  
  Serial.println("Detecting Main Height. Please, wait...");
  uint32_t mh_sum = 0;
  const int mh_measurements = 50;
  for (int i=0; i<mh_measurements; i++) {
    vl_value = vl_sensor.readRangeContinuousMillimeters() - vl_correction;
    mh_sum += (uint32_t)vl_value;
  }
  vl_mainHeight = (uint16_t)(mh_sum / mh_measurements);
  Serial.print("Done! Main Height: ");
  Serial.print(vl_mainHeight);
  Serial.println("mm");
  Serial.println();
}

void loop()
{
  // FLOW SENSOR
  if (!fl_oldTime) fl_oldTime = millis();
  if((millis() - fl_oldTime) > 500) {
    detachInterrupt(FLOWMETER_INTERRUPT);
    Serial.print("Pulse count: ");
    Serial.print(fl_pulseCount);
    Serial.print(" Elapsed time: ");
    Serial.print(millis() - fl_oldTime);
    fl_frequency = (float)fl_pulseCount / ((float)(millis() - fl_oldTime)/1000.0);
    fl_pulseCount = 0;
    Serial.print(" Freq: ");
    Serial.print(fl_frequency);
    Serial.print(" Flow (mL/s): ");
    Serial.print(fl_frequency * FLOWMETER_FACTOR); // mL/s
    Serial.print(" (L/min): ");
    Serial.println((fl_frequency * FLOWMETER_FACTOR)/16.667); // L/min*/
    fl_oldTime = millis();
    attachInterrupt(FLOWMETER_INTERRUPT, fl_pulseCounter, FALLING);
  }
  
  // DISTANCE SENSOR
  vl_value = vl_sensor.readRangeContinuousMillimeters() - vl_correction;
  vl_value = (vl_value < vl_mainHeight) ? (vl_mainHeight - vl_value) : 0;
  
  // Detect Cup Height and Placement
  if (!(cup_inRange && cup_placed) && vl_value > cup_height) cup_height = vl_value;
  if (!cup_placed && vl_value > cup_minHeight) cup_inRange = true;
  if (cup_inRange && vl_value < cup_height-10) cup_placed = true;
  if (cup_placed && vl_value > cup_height-5) cup_inRange = false;
  if (!cup_inRange && vl_value < cup_minHeight) {cup_placed = false;cup_height=0;}
  Serial.print("curr_value: ");
  Serial.print(vl_value);
  Serial.print(" | cup_height: ");
  Serial.print(cup_height);
  Serial.print(" | cup_inRange: ");
  Serial.print((cup_inRange) ? "Y" : "N");
  Serial.print(" | cup_placed: ");
  Serial.println((cup_placed) ? "Y" : "N");

  if (vl_sensor.timeoutOccurred()) {
    Serial.println("=== TIMEOUT ===");
    return wipe();
  }

  if (!(cup_inRange && cup_placed)) return wipe();

  Serial.println("--- FILLING ---");

  // CONTROL ALGORITHM
  if (!ca_kNextFlux) {
    ca_fillSpeed = 100; //cup_height/10.0;
    ca_dH = 30;
    ca_dT = ca_dH/ca_fillSpeed;
    ca_kNextFlux = ((12000*ca_dH)/(2*ca_dT))/1000;
    ca_k = 1;
    ca_currentPartFilled = false;
    ca_startTime = millis();
  } else if (ca_currentPartFilled) {
    ca_kVolume = ca_kNextFlux * ((ca_endTime - ca_startTime)/1000);
    ca_kNextFlux = ca_kVolume/ca_dT;
    ca_currentPartFilled = false;
    ca_startTime = millis();
    ca_k++;
  }
  
  Serial.print(" | ca_fillSpeed: ");
  Serial.print(ca_fillSpeed);
  Serial.print(" | ca_dT: ");
  Serial.print(ca_dT);
  Serial.print(" | ca_k: ");
  Serial.print(ca_k);
  Serial.print(" | ca_kNextFlux: ");
  Serial.print(ca_kNextFlux);
  Serial.print(" | ca_kVolume: ");
  Serial.print(ca_kVolume);
  Serial.print(" | ca_currentPartFilled: ");
  Serial.println(ca_currentPartFilled);

  if (!ca_currentPartFilled && vl_value > ca_k*ca_dH) {
    ca_currentPartFilled = true;
    ca_endTime = millis();
  }

  if (vl_value >= (cup_height * 0.5)) return wipe();

  sys_currentFlux = fl_frequency * FLOWMETER_FACTOR;
  sys_currentTime = millis();
  sys_elapsedTime = (sys_currentTime - sys_previousTime)/1000;

  sys_error = ca_kNextFlux - sys_currentFlux;Serial.print(" Error: ");Serial.print(sys_error);
  sys_integral += sys_error * sys_elapsedTime;
  sys_previousTime = sys_currentTime;

  if (sys_integral > 500000) sys_integral = 500000;
  else if (sys_integral < -5000000) sys_integral = -500000;Serial.print(" | Integral: ");Serial.print(sys_integral);

  sys_PI = sys_kp*sys_error + sys_ki*sys_integral;

  if (sys_PI > 255) sys_PI = 255;
  else if (sys_PI < 55) sys_PI = 55;Serial.print(" | PI: ");Serial.println(sys_PI);Serial.println();
  
  analogWrite(MOTOR_PIN, sys_PI);
}

void wipe()
{
  analogWrite(MOTOR_PIN, LOW);
  ca_kNextFlux = 0;
  Serial.println("--- WAITING ---");
}

void fl_pulseCounter()
{
  fl_pulseCount++;
}
