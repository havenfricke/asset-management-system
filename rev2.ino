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
int CURRENT_PERSON_ID = -1;
// For Huskylens dropouts
constexpr unsigned long PERSON_LOST_DEBOUNCE_MS = 500UL; // 500ms allowed if person detection drops out for camera
unsigned long lastPersonSeenMillis = 0;

// Timer/state
// ToF Proximity Settings
unsigned long proximityStartTime = 0;
bool timingActive = false;
// Classification timer
unsigned long classifyStartTime = 0;
bool classifyTimingActive = false;

// State machine
enum class SystemState : uint8_t { 
  IDLE, 
  PRESENCE_PENDING, 
  PENDING_FACE_RECOGNITION, 
  KNOWN_FACE_RECOGNITION
  };
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
        Serial.println("State -> PRESENCE_PENDING (timer started)");
        timingActive = true;
        proximityStartTime = millis();
        systemState = SystemState::PRESENCE_PENDING;
      }
      break;

    case SystemState::PRESENCE_PENDING:
      if (!personSeen || !inRange)
      {
        resetProximityTimer("PRESENCE_PENDING -> IDLE");
        Serial.println("State -> IDLE (person lost or out of range)");
        systemState = SystemState::IDLE;
        break;
      }

      if (timingActive && (millis() - proximityStartTime >= REQUIRED_TIME_MS))
      {
        Serial.println("State -> PENDING_FACE_RECOGNITION (person validated)");
        Serial.println("HUSKYLENS: switched to PENDING_FACE_RECOGNITION");
        timingActive = false;
        proximityStartTime = 0;
        huskylens.writeAlgorithm(ALGORITHM_FACE_RECOGNITION);
        systemState = SystemState::PENDING_FACE_RECOGNITION;
      }
      break;

    case SystemState::PENDING_FACE_RECOGNITION:
      if (!inRange)
      {
        huskylens.writeAlgorithm(ALGORITHM_OBJECT_RECOGNITION);
        resetProximityTimer("PENDING_CLASSIFIED -> IDLE");
        Serial.println("State -> IDLE (face recognition ended)");
        systemState = SystemState::IDLE;
        break;
      }

      huskylens.request();

      if (!huskylens.available())
        break;

      if (!huskylens.isLearned())
      {
        if (!classifyTimingActive)
        {
          Serial.println("Unrecognized person: Begin recognition countdown");
          classifyTimingActive = true;
          classifyStartTime = millis();
        }

        if (classifyTimingActive && (millis() - classifyStartTime >= REQUIRED_TIME_MS))
        {
          int nextID = huskylens.countLearnedIDs() + 1;
          Serial.print("Learning new face ID: ");
          Serial.println(String(nextID));

          if (huskylens.writeLearn(nextID))
          {
            Serial.println("writeLearn succeeded");
            Serial.println("State -> KNOWN_FACE_RECOGNITION");
            classifyTimingActive = false;
            classifyStartTime = 0;
            CURRENT_PERSON_ID = nextID;
            systemState = SystemState::KNOWN_FACE_RECOGNITION;
          }
          else
          {
            Serial.println("writeLearn failed");
            classifyTimingActive = false;
            classifyStartTime = 0;
          }
        }
      }
      else
      {
        HUSKYLENSResult r = huskylens.read();

        if (!classifyTimingActive)
        {
          Serial.println("Person recognized");
          CURRENT_PERSON_ID = r.ID;
          classifyTimingActive = true;
          classifyStartTime = millis();
        }

        if (classifyTimingActive && (millis() - classifyStartTime >= REQUIRED_TIME_MS))
        {
          Serial.println("State -> KNOWN_FACE_RECOGNITION (person recognized)");
          classifyTimingActive = false;
          classifyStartTime = 0;
          systemState = SystemState::KNOWN_FACE_RECOGNITION;
        }
      }
      break;

    case SystemState::KNOWN_FACE_RECOGNITION:
      if (!inRange)
      {
        resetProximityTimer("KNOWN_FACE_RECOGNITION -> IDLE");
        systemState = SystemState::IDLE;
        huskylens.writeAlgorithm(ALGORITHM_OBJECT_RECOGNITION);
        break;
      }

      huskylens.request();

      if (!huskylens.available())
      {
        if ((millis() - lastPersonSeenMillis) > PERSON_LOST_DEBOUNCE_MS)
        {
          resetProximityTimer("KNOWN_FACE_RECOGNITION -> IDLE (lost face)");
          Serial.println("State -> IDLE (known face lost)");
          systemState = SystemState::IDLE;
          huskylens.writeAlgorithm(ALGORITHM_OBJECT_RECOGNITION);
        }
        break;
      }

      {
        bool stillHere = false;

        while (huskylens.available())
        {
          HUSKYLENSResult r = huskylens.read();

          if (r.ID == CURRENT_PERSON_ID)
          {
            stillHere = true;
            Serial.println("Hello");

            // TODO: ADD LOGIC FOR RECOGNIZED PERSON HERE 
            // EXAMPLE: POST REQUEST AFTER X AMOUNT OF TIME
            // 
          }
        }

        if (!stillHere && (millis() - lastPersonSeenMillis) > PERSON_LOST_DEBOUNCE_MS)
        {
;
          resetProximityTimer("KNOWN_FACE_RECOGNITION -> IDLE (lost face)");
          Serial.println("State -> IDLE (known face lost)");
          systemState = SystemState::IDLE;
          huskylens.writeAlgorithm(ALGORITHM_OBJECT_RECOGNITION);
        }
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