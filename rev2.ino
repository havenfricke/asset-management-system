#include <HUSKYLENS.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>

// V53L0X TOF SETTINGS
Adafruit_VL53L0X tof;
constexpr uint16_t DISTANCE_THRESHOLD_MM = 800;
constexpr unsigned long REQUIRED_TIME_MS = 10000UL;
constexpr unsigned long LOOP_DELAY_MS = 50UL; // Master loop delay

// HUSKYLENS SETTINGS
HUSKYLENS huskylens;
const int PERSON_ID = 1;
// For Huskylens dropouts
constexpr unsigned long PERSON_LOST_DEBOUNCE_MS = 500UL; // 500ms allowed if person detection drops out for camera
unsigned long lastPersonSeenMillis = 0;



// Timer/state
unsigned long proximityStartTime = 0;
bool timingActive = false;

// State machine
enum class SystemState : uint8_t { IDLE, PRESENCE_PENDING, CLASSIFIED };
SystemState systemState = SystemState::IDLE;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600);
  Wire.begin();

  delay(2000);
  InitTof();
  delay(2000);
  InitHuskylens();
}

void loop()
{
  // Single pass: read sensors, then update state machine
  VL53L0X_RangingMeasurementData_t measurement;
  bool tofOk = readTof(measurement);

  // Read all Huskylens results this pass and determine if person is seen
  bool personSeenRaw = readHuskylensForPerson();
  // Treat person as present if seen recently within debounce window
  bool personSeen = (personSeenRaw || (millis() - lastPersonSeenMillis) <= PERSON_LOST_DEBOUNCE_MS);

  // Update state machine using single source of truth
  updateStateMachine(personSeen, tofOk ? measurement.RangeMilliMeter : 0);

  // Small non-blocking pacing to avoid hogging I2C
  delay(LOOP_DELAY_MS);
}

// -------------------- Initialization --------------------
void InitTof()
{
  Serial.println("Initializing VL53L0X");
  while (!tof.begin())
  {
    Serial.println("VL53L0X not detected");
    delay(500);
  }
  Serial.println("VL53L0X connected");
}

void InitHuskylens()
{
  Serial.println("Initializing HUSKYLENS");
  while (!huskylens.begin(Serial1))
  {
    Serial.println("HUSKYLENS not detected");
    delay(5000);
  }
  Serial.println("HUSKYLENS connected");
  huskylens.writeAlgorithm(ALGORITHM_OBJECT_RECOGNITION);
  Serial.println("Mode: Object Recognition");
}

// -------------------- Sensor helpers --------------------
bool readTof(VL53L0X_RangingMeasurementData_t &measurement)
{
  tof.rangingTest(&measurement, false);
  uint16_t range = measurement.RangeMilliMeter;

  // VL53L0X sentinel/error values
  const bool isValid = (range != 8190 && range != 8191);

  static bool lastInRange = false;
  bool nowInRange = (isValid && range <= DISTANCE_THRESHOLD_MM);

  if (!isValid)
  {
    return false;
  }
  
  else if (nowInRange != lastInRange)
  {
    // print only when entering or leaving the proximity threshold
    Serial.print("TOF mm: ");
    Serial.println(range);
    lastInRange = nowInRange;
  }

  return true;
}


bool readHuskylensForPerson()
{
  if (!huskylens.request())
  {
    Serial.println("HUSKYLENS request failed");
    return false;
  }

  bool personSeen = false;

  // If no results available, return false (no person seen)
  if (!huskylens.available()) return false;

  // Read all results this pass
  while (huskylens.available())
  {
    HUSKYLENSResult r = huskylens.read();
    Serial.print("HUSKYLENS ID: ");
    Serial.print(r.ID);
    Serial.print(" X:");
    Serial.print(r.xCenter);
    Serial.print(" Y:");
    Serial.println(r.yCenter);

    if (r.ID == PERSON_ID)
    {
      personSeen = true;
      // record the time we actually saw the person for camera debounce
      lastPersonSeenMillis = millis(); 
    }
  }

  return personSeen;
}

// -------------------- State machine --------------------
void updateStateMachine(bool personSeen, uint16_t distanceMm)
{
  const bool inRange = (distanceMm > 0 && distanceMm <= DISTANCE_THRESHOLD_MM);

  switch (systemState)
  {
    case SystemState::IDLE:
      if (personSeen && inRange)
      {
        // start timer
        timingActive = true;
        proximityStartTime = millis();
        systemState = SystemState::PRESENCE_PENDING;
        Serial.println("State -> PRESENCE_PENDING (timer started)");
      }
      // otherwise remain idle
      break;

    case SystemState::PRESENCE_PENDING:
      // Reset if person lost or out of range
      if (!personSeen || !inRange)
      {
        resetProximityTimer("PRESENCE_PENDING -> IDLE");
        systemState = SystemState::IDLE;
        Serial.println("State -> IDLE (person lost or out of range)");
        break;
      }

      // Check timeout only when timer active
      if (timingActive && (millis() - proximityStartTime >= REQUIRED_TIME_MS))
      {
        // validated
        timingActive = false;
        proximityStartTime = 0;
        systemState = SystemState::CLASSIFIED;
        Serial.println("State -> CLASSIFIED (person validated)");
        // switch algorithm to classification
        huskylens.writeAlgorithm(ALGORITHM_OBJECT_CLASSIFICATION);
        Serial.println("HUSKYLENS: switched to OBJECT_CLASSIFICATION");
      }
      break;

    case SystemState::CLASSIFIED:
      // In classified state, keep monitoring presence and range
      if (!personSeen || !inRange)
      {
        // revert to recognition mode
        huskylens.writeAlgorithm(ALGORITHM_OBJECT_RECOGNITION);
        systemState = SystemState::IDLE;
        resetProximityTimer("CLASSIFIED -> IDLE");
        Serial.println("State -> IDLE (classification ended)");
      }
      else
      {
        // Person still present and in range: perform classification actions here
        // example: read classification results, trigger actions, etc.
      }
      break;
  }
}

// -------------------- Utility --------------------
void resetProximityTimer(const char *mode)
{
  // Only print if timer was active or start time non-zero
  if (!timingActive && proximityStartTime == 0) return;
  Serial.print("Proximity timer reset: ");
  Serial.println(mode);
  timingActive = false;
  proximityStartTime = 0;
}