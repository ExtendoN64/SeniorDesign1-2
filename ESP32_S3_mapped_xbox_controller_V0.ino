//still need to add more motors and need to set up for mecanum wheels.
//Creator: GENE
//Project: ARIES
//Date:2/5/2026
//Board: ESP32-S3
//About: A basic code to map xbox controller buttons for functions.
//Edit: updating pins-2/5/2026
//    Currently working on CeeCee's frame. Auto Connects to Controller

//Ripped from https://www.youtube.com/watch?v=Laa93Wj7f-I
//Orginal Code: https://racheldebarros.com/connect-your-game-controller-to-an-esp32/

#include <Bluepad32.h>
#include <ESP32Servo.h>

//PIN CONNECTIONS

int ledPin1 = 20; //green LED A-button
int ledPin2 = 19; //red LED B-button

int IN1_FL = 5; // motor 1 dir1 
int IN2_FL = 4; // motor 1 dir2

int IN1_RL = 6; // motor 2 dir1
int IN2_RL = 7; // motor 2 dir2

int IN1_FR = 16; // motor 3 dir1 
int IN2_FR = 15; // motor 3 dir2

int IN1_RR = 17; // motor 4 dir1
int IN2_RR = 18; // motor 4 dir2

int xServoPin = 12;
int yServoPin = 13;

Servo xServo;
Servo yServo;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
      }
    }

    if (!foundEmptySlot) {
      Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

    if (!foundController) {
      Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// ========= SEE CONTROLLER VALUES IN SERIAL MONITOR ========= //

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
  "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
  "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY(),       // (-511 - 512) right Y axis
  ctl->brake(),        // (0 - 1023): brake button
  ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
  ctl->miscButtons(),  // bitmask of pressed "misc" buttons
  ctl->gyroX(),        // Gyro X
  ctl->gyroY(),        // Gyro Y
  ctl->gyroZ(),        // Gyro Z
  ctl->accelX(),       // Accelerometer X
  ctl->accelY(),       // Accelerometer Y
  ctl->accelZ()        // Accelerometer Z
  );
}

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...
 

  //== XBOX no buttons = 0x000 ==//
  if (ctl->buttons() == 0x0000) {
    // code for when A button is pushed
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
  }

  //== XBOX A button = 0x0001 ==//
  if (ctl->buttons() == 0x0001) {
    // code for when A button is pushed
    digitalWrite(ledPin1, HIGH);
  }
  if (ctl->buttons() != 0x0001) {
    // code for when A button is released
  }

  //== XBOX B button = 0x0002 ==//
  if (ctl->buttons() == 0x0002) {
    // code for when B button is pushed
     digitalWrite(ledPin2, HIGH);
  }
  if (ctl->buttons() != 0x0002) {
    // code for when B button is released
  }

  //== XBOX A&B button = 0x0003 ==//
  if (ctl->buttons() == 0x0003) {
    // code for when A button is pushed
    digitalWrite(ledPin1, HIGH);
    digitalWrite(ledPin2, HIGH);
  }
  if (ctl->buttons() != 0x0003) {
    // code for when A button is released
  }

  //== XBOX X button = 0x0004 ==//
  if (ctl->buttons() == 0x0004) {
    // code for when X button is pushed
  }
  if (ctl->buttons() != 0x0004) {
  // code for when X button is released
  }

  //== XBOX Y button = 0x0008 ==//
  if (ctl->buttons() == 0x0008) {
    // code for when Y button is pushed
  }
  if (ctl->buttons() != 0x0008) {
    // code for when Y button is released
  }

  //== XBOX Dpad UP button = 0x01 ==//
  if (ctl->buttons() == 0x01) {
    // code for when dpad up button is pushed
  }
  if (ctl->buttons() != 0x01) {
    // code for when dpad up button is released
  }

  //==XBOX Dpad DOWN button = 0x02==//
  if (ctl->buttons() == 0x02) {
    // code for when dpad down button is pushed
  }
  if (ctl->buttons() != 0x02) {
    // code for when dpad down button is released
  }

  //== XBOX Dpad LEFT button = 0x08 ==//
  if (ctl->buttons() == 0x08) {
    // code for when dpad left button is pushed
  }
  if (ctl->buttons() != 0x08) {
    // code for when dpad left button is released
  }

  //== XBOX Dpad RIGHT button = 0x04 ==//
  if (ctl->buttons() == 0x04) {
    // code for when dpad right button is pushed
  }
  if (ctl->buttons() != 0x04) {
    // code for when dpad right button is released
  }

  //== XBOX R1 trigger button = 0x0020 ==//
  if (ctl->buttons() == 0x0020) {
    // code for when R1 button is pushed
    digitalWrite(IN1_FL, LOW);  digitalWrite(IN2_FL, HIGH);
    digitalWrite(IN1_RL, HIGH);  digitalWrite(IN2_RL, LOW);
    digitalWrite(IN1_FR, LOW);  digitalWrite(IN2_FR, HIGH);   
    digitalWrite(IN1_RR, HIGH);  digitalWrite(IN2_RR, LOW);
  }
  if (ctl->buttons() != 0x0020) {
    // code for when R1 button is released
  }

  //== XBOX R2 trigger button = 0x0080 ==//
  if (ctl->buttons() == 0x0080) {
    // code for when R2 button is pushed
  }
  if (ctl->buttons() != 0x0080) {
    // code for when R2 button is released
  }

  //== XBOX L1 trigger button = 0x0010 ==//
  if (ctl->buttons() == 0x0010) {
    // code for when L1 button is pushed
    digitalWrite(IN1_FL, HIGH);  digitalWrite(IN2_FL, LOW);
    digitalWrite(IN1_RL, LOW);  digitalWrite(IN2_RL, HIGH);
    digitalWrite(IN1_FR, HIGH);  digitalWrite(IN2_FR, LOW);   
    digitalWrite(IN1_RR, LOW);  digitalWrite(IN2_RR, HIGH);
  }
  if (ctl->buttons() != 0x0010) {
    // code for when L1 button is released
  }

  //== XBOX L2 trigger button = 0x0040 ==//
  if (ctl->buttons() == 0x0040) {
    // code for when L2 button is pushed
  }
  if (ctl->buttons() != 0x0040) {
    // code for when L2 button is released
  }

    //== THROTTLE L2 ==//  Rotate ClounterClockWise!
  if (ctl->throttle() >= 25) {
    // code for when L2 is Pressed
    int motorSpeed = map(ctl->throttle(), 25, 1023, 0, 255);
    digitalWrite(IN1_FL, motorSpeed);  digitalWrite(IN2_FL, LOW);
    digitalWrite(IN1_RL, motorSpeed);  digitalWrite(IN2_RL, LOW);
    digitalWrite(IN1_FR, LOW);  digitalWrite(IN2_FR, motorSpeed);   
    digitalWrite(IN1_RR, LOW);  digitalWrite(IN2_RR, motorSpeed);
    }

        //== THROTTLE R2 ==//  Rotate ClockWise!
  if (ctl->brake() >= 25) {
    // code for when R2 is Pressed
    int motorSpeed = map(ctl->brake(), 25, 1023, 0, 255);    
    digitalWrite(IN1_FL, LOW);  digitalWrite(IN2_FL, motorSpeed);
    digitalWrite(IN1_RL, LOW);  digitalWrite(IN2_RL, motorSpeed);
    digitalWrite(IN1_FR, motorSpeed);  digitalWrite(IN2_FR, LOW);   
    digitalWrite(IN1_RR, motorSpeed);  digitalWrite(IN2_RR, LOW);
    }

  //== LEFT JOYSTICK - UP ==//  Foward movement!
  if (ctl->axisY() <= -25) {
    // code for when left joystick is pushed up
    int motorSpeed = map(ctl->axisY(), -25, -512, 0, 255);
    digitalWrite(IN1_FL, motorSpeed);  digitalWrite(IN2_FL, LOW);
    digitalWrite(IN1_RL, motorSpeed);  digitalWrite(IN2_RL, LOW);
    digitalWrite(IN1_FR, motorSpeed);  digitalWrite(IN2_FR, LOW);   
    digitalWrite(IN1_RR, motorSpeed);  digitalWrite(IN2_RR, LOW);
    }

  //== LEFT JOYSTICK - DOWN ==// Reverse!!
  if (ctl->axisY() >= 25) {
    // code for when left joystick is pushed down
    int motorSpeed = map(ctl->axisY(), 25, 511, 0, 255);
    digitalWrite(IN1_FL, LOW);  digitalWrite(IN2_FL, motorSpeed);
    digitalWrite(IN1_RL, LOW);  digitalWrite(IN2_RL, motorSpeed);
    digitalWrite(IN1_FR, LOW);  digitalWrite(IN2_FR, motorSpeed);   
    digitalWrite(IN1_RR, LOW);  digitalWrite(IN2_RR, motorSpeed);
   }

  //== LEFT JOYSTICK - LEFT ==//  Strafe Left
  if (ctl->axisX() <= -25) {
    // code for when left joystick is pushed left
  }

  //== LEFT JOYSTICK - RIGHT ==//  Strafe Right
  if (ctl->axisX() >= 25) {
    // code for when left joystick is pushed right
  }

  //== LEFT JOYSTICK DEADZONE ==// Stop All Motors
  if (ctl->axisY() > -25 && ctl->axisY() < 25 && ctl->axisX() > -25 && ctl->axisX() < 25 && ctl->throttle() < 25 && ctl->brake() < 25 && ctl->buttons() == 0x0000) {
    // code for when left joystick is at idle
    digitalWrite(IN1_FL, LOW);  digitalWrite(IN2_FL, LOW);
    digitalWrite(IN1_RL, LOW);  digitalWrite(IN2_RL, LOW);
    digitalWrite(IN1_FR, LOW);  digitalWrite(IN2_FR, LOW);   
    digitalWrite(IN1_RR, LOW);  digitalWrite(IN2_RR, LOW);   
  }

  //== RIGHT JOYSTICK - X AXIS ==//  Servo Arm!
  if (ctl->axisRX()) {
    // code for when right joystick moves along x-axis
    int servoPos = map(ctl->axisRX(), -512, 511, 0, 180);
    xServo.write(servoPos);
  }

  //== RIGHT JOYSTICK - Y AXIS ==//
  if (ctl->axisRY()) {
  // code for when right joystick moves along y-axis
    int servoPos = map(ctl->axisRY(), -512, 511, 0, 180);
    xServo.write(servoPos);
  }
  dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
         processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(IN1_FL, OUTPUT);
  pinMode(IN2_FL, OUTPUT);
  pinMode(IN1_RL, OUTPUT);
  pinMode(IN2_RL, OUTPUT);
  pinMode(IN1_FR, OUTPUT);
  pinMode(IN2_FR, OUTPUT);
  pinMode(IN1_RR, OUTPUT);
  pinMode(IN2_RR, OUTPUT);

  xServo.attach(xServoPin);
  yServo.attach(yServoPin);

  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    // vTaskDelay(1);
  delay(150);
}
