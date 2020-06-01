#include "FastAccelStepper.h"

#define dirPinStepper1    5
#define enablePinStepper1 6
#define stepPinStepper1   9  /* OC1A */

#define dirPinStepper2    7
#define enablePinStepper2 8
#define stepPinStepper2   10 /* OC1B */

#define LED 13

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper1 = engine.stepperA();
FastAccelStepper *stepper2 = engine.stepperB();

void setup() {
   Serial.begin(115200);

   engine.init();
   engine.setDebugLed(LED);

   stepper1->setDirectionPin(dirPinStepper1);
   stepper2->setDirectionPin(dirPinStepper2);

   stepper1->setEnablePin(enablePinStepper1);
   stepper2->setEnablePin(enablePinStepper2);

   stepper1->set_auto_enable(true);
   stepper2->set_auto_enable(true);
}

uint8_t in_ptr = 0;
char in_buffer[256];
uint8_t in_val_ptr = 0;
long in_vals[8];
bool stopped = false;

void loop() {
   bool cmd_ok = false;

   if (Serial.available()) {
      char ch = Serial.read();
      if ((ch == '\n') || (ch == ' ')) {
         in_buffer[in_ptr] = 0;
         in_ptr = 0;
         if (in_val_ptr < 8) {
            in_vals[in_val_ptr++] = atol(in_buffer);
         }
      }
      else {
         in_buffer[in_ptr++] = ch;
      } 
      if (ch == '\n') {
         if (in_val_ptr == 4) {
            cmd_ok = true;
         }
         in_val_ptr = 0;
         in_ptr = 0;
      }
   }

   if (cmd_ok) {
     long motor = in_vals[0];
     long move = in_vals[1];
     long speed = in_vals[2];
     long accel = in_vals[3];
     if  (((motor == 1) || (motor == 2)) && move) {
        Serial.print("speed=");
        Serial.print(speed);
        Serial.print("  accel=");
        Serial.print(accel);
        Serial.print("  move=");
        Serial.print(move);
        if (motor == 1) {
           stopped = false;
           stepper1->set_dynamics(speed*1.0, accel*1.0);
           stepper1->move(move);
           Serial.print("  Start stepper 1: ");
           Serial.println(stepper1->getCurrentPosition());
        }
        if (motor == 2) {
           stopped = false;
           stepper2->set_dynamics(speed*1.0, accel*1.0);
           stepper2->move(move);
           Serial.print("  Start stepper 2: ");
           Serial.println(stepper2->getCurrentPosition());
        }
      }
   }
   delay(100);

   if (!stopped) {
      Serial.print("Stepper 1: ");
      Serial.print(stepper1->getCurrentPosition());
      if (stepper1->isRunning()) {
         Serial.print("  RUNNING");
      }
      else {
         Serial.print("  PAUSED ");
      }
      Serial.print("  Stepper 2: ");
      Serial.print(stepper2->getCurrentPosition());
      if (stepper2->isRunning()) {
         Serial.print("  RUNNING");
      }
      else {
         Serial.print("  PAUSED ");
      }
      Serial.print("  TCCR1A=");
      Serial.print(TCCR1A);
      Serial.print("  TCCR1B=");
      Serial.print(TCCR1B);
      Serial.print("  TIMSK1=");
      Serial.print(TIMSK1);
      Serial.println("");
      stopped = !(stepper1->isRunning() || stepper2->isRunning());
      if (stopped) {
         Serial.println("Please enter one line with <motor> <steps> <speed> <acceleration> e.g.");
         Serial.println("1 10000 1000 100");
      }
   }
}
