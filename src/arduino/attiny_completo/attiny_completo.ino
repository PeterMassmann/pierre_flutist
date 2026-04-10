#include <AccelStepper.h>
#include <TinyWire.h> //Se inicia la librería TinyWire
#define myDir 18
#define stepPin 4
#define dirPin 3

AccelStepper stepper(1, stepPin, dirPin);

void setup() {
  //stepper.setMaxSpeed(8000); // Set maximum speed value for the stepper
  stepper.setAcceleration(50000); // Set acceleration value for the stepper
  stepper.setCurrentPosition(0); // Set the current position to 0 steps

  TinyWire.begin(myDir); //Se entra al bus I2C en la direccion 10
  TinyWire.onReceive(receiveEvent);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (stepper.currentPosition() != stepper.targetPosition()) {
    stepper.run();
  }
  
}

void receiveEvent(int howMany) {
  while (TinyWire.available()) {
    byte c = TinyWire.read();
    if(c){
      stepper.moveTo(0);
    }
    else {
      stepper.moveTo(100);
    }
  }
}
