#include <SoftwareSerial.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// initialize software serial between Arduino Uno and ESP8266
SoftwareSerial esp(4, 5);

//PID initialization
double Setpoint, Input, Output;
//15,130,1
// 30,30,1.5
PID myPID(&Input, &Output, &Setpoint, 15, 120, 1.1, DIRECT);

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define DEBUG false
bool left = false;
bool right = false;
uint8_t loopCount = 0;
#define LED_PIN 13
#define MOTOR_SPEED 255
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//volatile bool balance = false;
void dmpDataReady() {
  mpuInterrupt = true;
  loopCount++;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // PID initial varialbles declarations
  Setpoint = 1.9; //2.8 backwards
  // 1.9 balance
  Input = 0;  //Gyro not configured yet so set default input

  // turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-1 * MOTOR_SPEED, MOTOR_SPEED);
  //10
  // 5ms - 9v on the battery
  myPID.SetSampleTime(5);   // 1mS


  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  esp.begin(115200);

  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  while (Serial.available() && Serial.read()); // empty buffer

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(-50);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  //Configure Motor Shield
  //Setup Channel A
  pinMode(12, OUTPUT); //Initiates Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates Brake Channel A pin

  //Setup Channel B
  pinMode(13, OUTPUT); //Initiates Motor Channel A pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel A pin
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (loopCount == 20) {
      loopCount = 0;
    }

    // Check for incoming commands
    if (esp.available()) {
      switch (int(esp.read())) {
        case 2:    //Foreward
          Serial.println("Foreward");
          Setpoint = .7;
          myPID.SetTunings(40, 120, 2);
          break;
        case 3:    //Backward
          Serial.println("Backwards");
          Setpoint = 2.3;
          myPID.SetTunings(30, 120, 2);
          break;
        case 4:    //Left
          // Serial.println("left");
          left = true;
          Setpoint = 3.5;
          myPID.SetTunings(15, 150, 2);
          break;
        case 5:    //Right
          right = true;
          Setpoint = 0;
          myPID.SetTunings(15, 150, 2.1);
          break;
        default:   //Unknown command or 0 causes stop
          Serial.println("Stop");
          left = false;
          right = false;
          Setpoint = 1.9;
          //Setpoint = 2.5;
          myPID.SetTunings(15, 120, 1.1);
      }
    }

    // Use PID to find error component
    Input = (ypr[1] * 180 / M_PI);
    myPID.Compute();

#if DEBUG
    Serial.print("Input: ");
    Serial.println(Input);
#endif

    // translate error component to a motorspeed
    int motorSpeed = int(Output);

#if DEBUG
    Serial.print("Motor Speed: ");
    Serial.println(motorSpeed);
#endif
    if (right && loopCount > 15) {
      if (motorSpeed < 0) {
        digitalWrite(9, HIGH);  //Engage the Brake for Channel A
        digitalWrite(9, HIGH);  //Engage the Brake for Channel B

        digitalWrite(12, LOW);  //Establishes backward direction of Channel A
        digitalWrite(9, LOW);   //Disengage the Brake for Channel A
        analogWrite(3, abs((-1 * motorSpeed) - 180)); //Spins the motor on Channel A

        digitalWrite(13, LOW);  //Establishes backward direction of Channel B
        digitalWrite(8, LOW);   //Disengage the Brake for Channel B
        analogWrite(11, (-1 * motorSpeed));  //Spins the motor on Channel B
      }

      //Motors backward
      else if (motorSpeed > 0) {
        digitalWrite(9, HIGH);  //Engage the Brake for Channel A
        digitalWrite(9, HIGH);  //Engage the Brake for Channel B

        digitalWrite(12, HIGH); //Establishes forward direction of Channel A
        digitalWrite(9, LOW);   //Disengage the Brake for Channel A
        analogWrite(3, motorSpeed);   //Spins the motor on Channel A

        digitalWrite(13, HIGH); //Establishes forward direction of Channel B
        digitalWrite(8, LOW);   //Disengage the Brake for Channel B
        analogWrite(11, abs(motorSpeed - 180)); //Spins the motor on Channel B
      }

      else {
        digitalWrite(9, HIGH);  //Engage the Brake for Channel A
        digitalWrite(9, HIGH);  //Engage the Brake for Channel B
      }
    }

    else  if (left && loopCount > 15) {
      if (motorSpeed < 0) {
        digitalWrite(9, HIGH);  //Engage the Brake for Channel A
        digitalWrite(9, HIGH);  //Engage the Brake for Channel B

        digitalWrite(12, LOW);  //Establishes backward direction of Channel A
        digitalWrite(9, LOW);   //Disengage the Brake for Channel A
        analogWrite(3, (-1 * motorSpeed));  //Spins the motor on Channel A

        digitalWrite(13, LOW);  //Establishes backward direction of Channel B
        digitalWrite(8, LOW);   //Disengage the Brake for Channel B
        analogWrite(11, abs((-1 * motorSpeed) - 180)); //Spins the motor on Channel B
      }

      //Motors backward
      else if (motorSpeed > 0) {
        digitalWrite(9, HIGH);  //Engage the Brake for Channel A
        digitalWrite(9, HIGH);  //Engage the Brake for Channel B

        digitalWrite(12, HIGH); //Establishes forward direction of Channel A
        digitalWrite(9, LOW);   //Disengage the Brake for Channel A
        analogWrite(3, abs(motorSpeed - 180)); //Spins the motor on Channel A

        digitalWrite(13, HIGH); //Establishes forward direction of Channel B
        digitalWrite(8, LOW);   //Disengage the Brake for Channel B
        analogWrite(11, motorSpeed);   //Spins the motor on Channel B
      }

      else {
        digitalWrite(9, HIGH);  //Engage the Brake for Channel A
        digitalWrite(9, HIGH);  //Engage the Brake for Channel B
      }
    }

    else {
      if (motorSpeed < 0) {
        digitalWrite(9, HIGH);  //Engage the Brake for Channel A
        digitalWrite(9, HIGH);  //Engage the Brake for Channel B

        digitalWrite(12, LOW);  //Establishes backward direction of Channel A
        digitalWrite(9, LOW);   //Disengage the Brake for Channel A
        analogWrite(3, (-1 * motorSpeed));  //Spins the motor on Channel A

        digitalWrite(13, LOW);  //Establishes backward direction of Channel B
        digitalWrite(8, LOW);   //Disengage the Brake for Channel B
        analogWrite(11, (-1 * motorSpeed));  //Spins the motor on Channel B
      }

      //Motors backward
      else if (motorSpeed > 0) {
        digitalWrite(9, HIGH);  //Engage the Brake for Channel A
        digitalWrite(9, HIGH);  //Engage the Brake for Channel B

        digitalWrite(12, HIGH); //Establishes forward direction of Channel A
        digitalWrite(9, LOW);   //Disengage the Brake for Channel A
        analogWrite(3, motorSpeed);   //Spins the motor on Channel A

        digitalWrite(13, HIGH); //Establishes forward direction of Channel B
        digitalWrite(8, LOW);   //Disengage the Brake for Channel B
        analogWrite(11, motorSpeed);   //Spins the motor on Channel B
      }

      else {
        digitalWrite(9, HIGH);  //Engage the Brake for Channel A
        digitalWrite(9, HIGH);  //Engage the Brake for Channel B
      }
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

#if DEBUG
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
