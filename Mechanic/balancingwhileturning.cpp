// // I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include <AccelStepper.h>
 
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define dirPin 14
#define stepPin 4
#define stepsPerRevolution 200

#define dirPin2 15
#define stepPin2 23


MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high




AccelStepper m1(AccelStepper::DRIVER, stepPin, dirPin); //motor left
AccelStepper m2(AccelStepper::DRIVER, stepPin2, dirPin2);   //motor right
TaskHandle_t Task1;


#define OUTPUT_READABLE_YAWPITCHROLL

//#define OUTPUT_READABLE_ACCELGYRO

#define INTERRUPT_PIN  2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
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
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

int stepcount=0;
float position[3] = {0.0, 0.0, 0.0};
float wheelc;   //CALCULATE WHEEL CIRCUMFERENCE AND ENTER IT HERE
float displacement;
float yaw;
float roll;
volatile float pitch, tpitch=0;
float direction=M_PI/4;
float inertia=1.4;
float s1=0;
float s2=0;
float speed=0;
unsigned long interval = 1000;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
// TUNE THESE BY TRIAL AND ERROR
float Kp=0, Ki=0, Kd=0;
float Kps=0, Kis=0, Kds=0;
// variables for PID controller
float pepitch=0, epitch=0, ipitch=0, dpitch=0, pid=0;
float tspeed=0, pespeed=0, espeed=0, ispeed=0, dspeed=0, pids=0;
hw_timer_t * timer1=NULL;
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void motion(){
        m1.runSpeed();  // Acceleration in steps per second per second
        m2.runSpeed();  // Acceleration in steps per second per second
}

void initTimers(){
  timer1=timerBegin(0,40,true);
  timerAttachInterrupt(timer1, &motion, true);
  timerAlarmWrite(timer1, 50, true);
  timerAlarmEnable(timer1);
}
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
   
 
    // Declare pins as output:
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(stepPin2, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    yaw=0.0;
    roll=0.0;
    pitch=0.0;
    displacement=0.0;
    wheelc=2*PI*0.0325; //radius of the wheels are 0.0325
    direction=0;
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);  
    while (!Serial); // wait for Leonardo enumeration, others continue immediately


    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    m1.setMaxSpeed(8000);
    m2.setMaxSpeed(8000);
    m1.setAcceleration(1000);
    m2.setAcceleration(1000);
    initTimers();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
       
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
     
    }
    xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      1,  /* Priority of the task */
      &Task1,  /* Task handle. */
      0); /* Core where the task should run */

}

void Task1code( void* parameter) {
  Serial.print("inside other core");
  while(1) {
        currentMillis = millis();

        //SPEED CONTROLLER
        tspeed=0;
        Kps=-0.0001;
        Kis=0;
        Kds=-0.001;
        speed=s1;
        espeed = tspeed - speed;
        ispeed += espeed;
        dspeed=espeed-pespeed;
        pids = Kps * espeed + Kis * ispeed + Kds * dspeed;
        tpitch+=pids;
        tpitch=constrain(tpitch, -4, 4);
        
        pespeed=espeed;


        //PITCH CONTROLLER
        Kp=5;
        Ki=0;
        Kd=400;
        epitch = tpitch - pitch;
        ipitch += epitch;
        dpitch=epitch-pepitch;
        pid = Kp * epitch + Ki * ipitch + Kd * dpitch;
        pepitch=epitch;
        s1+=pid;
        s1=constrain(s1, -8000, 8000);

        m1.setSpeed(-s1);
        m2.setSpeed(s1+2000);

        delay(5);

 
    }
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    // if programming failed, don't try to do anything
   
if (!dmpReady) return;
    // read a packet from FIFO
    currentMillis = millis();

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
       
     
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");

            Serial.print(tpitch);
            Serial.print("\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            yaw=ypr[0];
            pitch=ypr[1]* 180/M_PI;
            roll=ypr[2];
        #endif
           }
     
    }
