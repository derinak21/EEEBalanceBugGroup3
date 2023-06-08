// // I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include <AccelStepper.h>

#include <BasicLinearAlgebra.h>

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

using namespace BLA;

AccelStepper m1(AccelStepper::DRIVER, stepPin, dirPin); //motor left
AccelStepper m2(AccelStepper::DRIVER, stepPin2, dirPin2);   //motor right


#define OUTPUT_READABLE_YAWPITCHROLL

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
bool turn=0;
bool back=0;
float position[3] = {0.0, 0.0, 0.0};
float wheelc;   //CALCULATE WHEEL CIRCUMFERENCE AND ENTER IT HERE
float displacement;
float x;
float yaw;
float roll;
float pitch;
float direction=M_PI/4;
float inertia=1.4;
float a1=0;
float a2=0;
float s1=0;
float s2=0;
float velocity=0;
unsigned long interval = 50;;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
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
    BLA::Matrix<6, 6> A_d={1, 0.0922600151152885, 0, 0, -0.00916245729359372, 0.000463725836507565, 0, 0.849653078000291, 0, 0, -0.177725984313866, 0.00587223490637719, 0, 0, 1, 0.0925496377079686, 0, 0, 0, 0, 0.854788506660701, 0, 0, 0, 0, 0, 0.0161490660142083, 0, 0, 0, 1.03089655650108, 0.0994242787165783, 0, 0.314345697068390, 0, 0, 0, 0.607645253205616, 0.999461986794241};
    BLA::Matrix<6, 2> B_d={0.00386999244235575, 0.00386999244235575, 0.0751734609998545, 0.0751734609998545, 0.0232823821625982, -0.0232823821625982, 0.453785916685308, -0.453785916685308, -0.00807453300710413, -0.00807453300710413, -0.157172848534195, -0.157172848534195};
    BLA::Matrix<2, 6> C_d={1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
    BLA::Matrix<2, 6> K={-5.89561854850652, -13.2835976137713, 0.393874612977434, 0.33854205283429, -14.4569280300536, -8.67211173540422, -8.21393795538072, -17.1177514455303, -0.427339321361424, -0.371990906329615, -17.5029813691579, -10.7338112176612};
    BLA::Matrix<1, 6> x_i={0 ,0, 0, 0, 0, 0};
    BLA::Matrix<6, 6> I={1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1};
    BLA::Matrix<1, 6> y_d={0.5, direction};
    BLA::Matrix<2, 1> u={0,0};

    // Declare pins as output:
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(stepPin2, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    yaw=0.0;
    roll=0.0;
    pitch=0.0;
    x=0.0;
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
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

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
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
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
            
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            yaw=ypr[0];
            pitch=ypr[1];
            roll=ypr[2];
        #endif
        //yaw, roll. pitch

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif

        currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        u=-K*xi+Invert(C_d*Invert(I-(A_d-B_d*K))*B_d)*yd; 
        a1=u[0]/inertia;
        a2=u[1]/inertia;
        m1.setAcceleration(a1);  // Acceleration in steps per second per second
        m2.setAcceleration(a2);  // Acceleration in steps per second per second
        s1=m1.speed();
        s2=m2.speed();
        //torque=inertia*acceleration
        previousMillis = currentMillis;
        if(s1>0 && s2>0){
            displacement=(s1+s2)*interval/2;
            velocity=(s1+s2)/2;
            position[0]+=displacement*cos(roll);
            position[1]+=displacement*sin(roll); 
        }
        xi={displacement, velocity, roll, mpu.getRotationX(), pitch, mpu.getRotationY()}
        }

        m1.runSpeed();
        m2.runSpeed();
        // long m1p = m1.currentPosition();
        // long m2p = m2.currentPosition();
        Serial.println("displacement: ");
        Serial.println(displacement);
        Serial.println("x: ");
        Serial.println(position[0]);
        Serial.println("y: ");
        Serial.println(position[1]);
        Serial.println("yaw: ");
        Serial.println(yaw*180/M_PI);
        }
    }

