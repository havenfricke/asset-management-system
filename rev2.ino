#include <HUSKYLENS.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>

// V53L0X TOF SETTINGS
Adafruit_VL53L0X tof;

#define DISTANCE_THRESHOLD_MM 800
#define REQUIRED_TIME_MS 10000

unsigned long proximityStartTime = 0;
bool timingActive = false;

// HUSKYLENS SETTINGS
HUSKYLENS huskylens;

// ID trained as "person" using default recognition model
const int PERSON_ID = 1;     
bool switchedToClassification = false;

void setup()
{
  // SERIAL INIT
  Serial.begin(115200);
  Serial1.begin(9600);

  //I2C
  Wire.begin();

  // CALL INIT METHOD
  delay(2000);
  InitTof();

  // CALL INIT METHOD
  delay(2000);
  InitHuskylens();
}

void loop()
{
  HuskylensTofRecognitionLoop();
  HuskylensTofClassificationLoop();
}

// ------------------------------------------------------------
// PROGRAM LOGIC
// ------------------------------------------------------------

// SETUP CALLS

// TOF INITIALIZE SETUP CALL
void InitTof()
{
  Serial.println("Initializing VL53L0X");
  while (!tof.begin())
  {
    Serial.println("VL53L0X not dectected");
  }

  Serial.println("VL53L0X connected");
}

// HUSKYLENS INITIALIZE SETUP CALL
void InitHuskylens()
{
  Serial.println("Initializing HUSKYLENS");

  while (!huskylens.begin(Serial1))
  {
    Serial.println("HUSKYLENS not detected");
    delay(5000);
  }

  Serial.println("HUSKYLENS connected");

  // Start in Object Recognition
  huskylens.writeAlgorithm(ALGORITHM_OBJECT_RECOGNITION);
  Serial.println("Mode: Object Recognition");
}

// LOOP CALLS

// RECOGNITION - CALLED IN LOOP
void HuskylensTofRecognitionLoop()
{
  if (!huskylens.request()){ Serial.println("Request failed"); return;}

  if (!huskylens.available()){ delay(200); return; }

  while (!switchedToClassification && huskylens.available())
  {
    HUSKYLENSResult result = huskylens.read();

    Serial.print("ID: ");
    Serial.print(result.ID);
    Serial.print(" | X: ");
    Serial.print(result.xCenter);
    Serial.print(" | Y: ");
    Serial.print(result.yCenter);
    Serial.println("------------");

    // PERSON DETECTED
    if (!switchedToClassification && result.ID == PERSON_ID)
    {
      VL53L0X_RangingMeasurementData_t measurement;

      // pass memory address of measurement
      tof.rangingTest(&measurement, false);

      Serial.println(measurement.RangeMilliMeter);

      if (measurement.RangeMilliMeter <= DISTANCE_THRESHOLD_MM)
      {
        if (!timingActive)
        {
          timingActive = true;
          proximityStartTime = millis();
          Serial.println("Proximity timer started: Recognition");
          break;
        }
      }

      // Distance > Distance Threshold -> Switch to Object recognition
      if (measurement.RangeMilliMeter > DISTANCE_THRESHOLD_MM)
      {
        huskylens.writeAlgorithm(ALGORITHM_OBJECT_RECOGNITION);
        Serial.println("Mode: Object Recognition");
      }

      // Time - Start Time >= Required Time -> Switch to Classification and assign ID
      if (millis() - proximityStartTime >= REQUIRED_TIME_MS)
      {
        Serial.println("Person validated: Switching to object classification");

        huskylens.writeAlgorithm(ALGORITHM_OBJECT_CLASSIFICATION);

        switchedToClassification = true;
        timingActive = false;

        proximityStartTime = 0;

        delay(200);
        return;
      }

      if (measurement.RangeMilliMeter > DISTANCE_THRESHOLD_MM)
      {
        resetProximityTimer();
      }

      return;
    }

    Serial.println();
  }

  delay(200);
}

// CLASSIFICATION - CALLED IN LOOP
void HuskylensTofClassificationLoop()
{
  if (!huskylens.request()){ Serial.println("Request failed"); return;}

  if (!huskylens.available()){ delay(200); return; }

  while (switchedToClassification && huskylens.available())
  {
    HUSKYLENSResult result = huskylens.read();

    Serial.print("Classified ID: ");
    Serial.println(result.ID);

    Serial.print("X: ");
    Serial.print(result.xCenter);
    Serial.print(" Y: ");
    Serial.println(result.yCenter);

    VL53L0X_RangingMeasurementData_t measurement;

    // pass memory address of measurement
    tof.rangingTest(&measurement, false);

    Serial.println(measurement.RangeMilliMeter);

    if (measurement.RangeMilliMeter <= DISTANCE_THRESHOLD_MM)
    {
      if (!result.ID)
      {
        Serial.print("Unrecognized person");
        // TODO: wait 10s then classify person
        return;
      }

      if (!timingActive)
      {
        timingActive = true;
        proximityStartTime = millis();
        Serial.println("Proximity timer started: Classification");
        break;
      }

      // TODO: if result.ID = true, do something that is useful for a classified person

      if (measurement.RangeMilliMeter > DISTANCE_THRESHOLD_MM)
      {
        resetProximityTimer();
      }
    }
  }
  delay(200);
}

// ------------------------------------------------------------
// HELPERS / UTILITY
// ------------------------------------------------------------

void resetProximityTimer()
{
  timingActive = false;
}

