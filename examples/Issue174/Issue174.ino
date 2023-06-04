#include "FastAccelStepper.h"

long chirpTimeInitial = 0;

// Stepper Wiring
//#define dirPinStepper 8   // This can be any output capable port pin.
//#define stepPinStepper 9  // step pin must be pin 9, 10 or 11

// Stepper Wiring
#define dirPinStepper 19
#define stepPinStepper 14

// no clue what this does
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

float steps_per_rev = 800;
float rpm = 4000;
float maxStepperSpeed =
    (rpm / 60 *
     steps_per_rev);  // needs to be in us per step || 1 sec = 1000000 us
// float maxStepperAccel = 1e2;

// this works with esp32 and mcpwm
// this fails with esp32 and rmt...even stops working
float maxStepperAccel = 1e8;

// this works with esp32 and rmt
// float maxStepperAccel = 1e5;

void setup() {
  // Serial.begin(250000);
  Serial.begin(115200);

  delay(1000);

  // FastAccelStepper setup
  engine.init();
  //  engine.setDebugLed(1); // will disable communication

#if defined(SUPPORT_SELECT_DRIVER_TYPE)
  stepper = engine.stepperConnectToPin(stepPinStepper, DRIVER_RMT);
  // stepper = engine.stepperConnectToPin(stepPinStepper);
#else
  stepper = engine.stepperConnectToPin(stepPinStepper);
#endif

  if (stepper) {
    Serial.println("Setup stepper!");

    // Stepper Parameters
    stepper->setDirectionPin(dirPinStepper, false);
    stepper->setAutoEnable(true);

    stepper->setSpeedInHz(maxStepperSpeed);     // steps/s
    stepper->setAcceleration(maxStepperAccel);  // steps/s²
  }

  stepper->forceStopAndNewPosition(0);
  stepper->moveTo(0);
  // pulse counter 0 is occupied, if using mcpwm/pcnt => so use 1
#if defined(SUPPORT_ESP32_PULSE_COUNTER)
  stepper->attachToPulseCounter(1, 0, 0);
#endif

  chirpTimeInitial = micros();
}

long currentTime = 0;
long elapsedTime = 0;
long previousTime = 0;

long Position_Next = 0;

long targetCycleTime = 130;

float f_0 = 0.5;     // Hz, initial freuqncy of oscillation
float f_ramp = 0.1;  // Hz / s  frequency ramp of oscillation
float f = 0;
float positionOscillation;
float oscilattionAmplitude = 500;

uint16_t loopCnt = 0;

void loop() {
  loopCnt++;
#if defined(SUPPORT_ESP32_PULSE_COUNTER)
  if ((loopCnt % 1000) == 0) {
    Serial.print("Go to ");
    Serial.print(loopCnt);
    Serial.print(" from ");
    Serial.print(stepper->getCurrentPosition());
    Serial.print(" ramp state=");
    Serial.print(stepper->rampState());
    Serial.print(' ');
    int res = stepper->moveTo(loopCnt);
    if (!stepper->isRunning()) {
      Serial.println();
      Serial.println("stepper is not running !!!");
    }
    if (res != MOVE_OK) {
      Serial.println();
      Serial.print("Error return from move: ");
      Serial.println(res);
    }
    while (stepper->isRunning()) {
    }
    int32_t pos = stepper->getCurrentPosition();
    delay(10);
    int32_t pos2 = stepper->getCurrentPosition();
    if (pos != pos2) {
      Serial.println();
      Serial.print("stepper position changed after isRunning() loop: ");
      Serial.print(pos);
      Serial.print(' ');
      Serial.println(pos2);
    }
    // delay(10);
    Serial.print(stepper->rampState());
    Serial.print(':');
    Serial.print(stepper->getPositionAfterCommandsCompleted());
    Serial.print(' ');
    int32_t pcnt = stepper->readPulseCounter();
    if (pcnt < 0) {
      pcnt += 65536;
    }
    if (pcnt == loopCnt) {
      Serial.print("=> OK");
    } else {
      Serial.print("=> FAIL with pcnt=");
      Serial.print(pcnt);
      Serial.print("!=");
      Serial.print(loopCnt);
      digitalWrite(dirPinStepper,
                   digitalRead(dirPinStepper) == HIGH ? LOW : HIGH);
      delay(10);
      stepper->clearPulseCounter();
      stepper->setCurrentPosition(0);
      digitalWrite(dirPinStepper,
                   digitalRead(dirPinStepper) == HIGH ? LOW : HIGH);
      delay(10);
    }
    Serial.print(" ramp state (must be 0) = ");
    Serial.println(stepper->rampState());
  }
#endif

  float chirpTime = (micros() - chirpTimeInitial) * 1e-6;

  // generate chirp signal
  f = f_0 + f_ramp * chirpTime;
  positionOscillation = cos(2 * PI * f * chirpTime);
  positionOscillation *= oscilattionAmplitude;

  // if frequency increases the maximum frequency, reset and repeat
  if (f > 50) {
    chirpTimeInitial = micros();
  }

  // obtain time
  currentTime = micros();
  elapsedTime = currentTime - previousTime;
  if (elapsedTime < 1) {
    elapsedTime = 1;
  }

  // the average cycle time on my device is approx. 130 us --> mimic that
  long waitTime = targetCycleTime - elapsedTime;
  if (waitTime > 0) {
    delayMicroseconds(waitTime);
  }

  previousTime = currentTime;

  // add target position
  stepper->moveTo(positionOscillation, false);
}
