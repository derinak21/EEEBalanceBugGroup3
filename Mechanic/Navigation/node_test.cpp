#include "I2Cdev.h"
#include <unordered_map>
#include <map>
#include <vector>
#include <NewPing.h>
#include <cstdlib>


#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define dirPin 14
#define stepPin 4
#define stepsPerRevolution 200

#define dirPin2 15
#define stepPin2 23

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN  2 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// ================================================================
// ===                   MPU INITIALISATION                     ===
// ================================================================

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
float position[2] = {0.0, 0.0};
float position_rl[2] = {0.0, 0.0};
float position_ll[2] = {0.0, 0.0};
float position_b1[2] = {0.0, 0.0};
float position_b2[2] = {0.0, 0.0};
float position_p; 
float wheelc;   //CALCULATE WHEEL CIRCUMFERENCE AND ENTER IT HERE
float displacement;
float yaw;
float roll;
float pitch;
bool node;
bool nodeoptions;
float gradient1;
float gradient2;
float min_angle = 360;
float max_angle = -360;

float d0 = 0.58; //DEFINE THIS
//float d1; //DEFINE THIS

float theta_1 = 0; 
float theta_2 = 0; 

float de_init_yaw;
float initial_yaw;
float target_yaw;
char command; 
float x, y;
bool turn;
int camera_command;
int direction;
char color;
std::vector<char> initbeacons;
std::map<char, std::pair<float, float>> beaconMap;
std::unordered_map<size_t, std::unordered_map<int, bool>> nodes;
std::unordered_map<float, bool> angles;
angles[-90] = false;
angles[-179] = false;

//ultrasonic sensors
int dis_left, dis_right;
unsigned int us1, us2;

// ================================================================
// ===                          FLAGS                           ===
// ================================================================
bool beacon_flag_1 = false; 
bool beacon_flag_2 = false; 
bool check_node = false;
bool nodeflag = false; 
bool first_time_node = false; 
bool initialisation=true;
bool initialbeacon=true;
bool beaconposition=true;
bool beacon_flag_fpga = false; 
bool de_flag=false;
bool done_checking=false;
bool is_path = false;
bool forward_path = false;

bool nodeflag_left = false;
bool nodeflag_right = false;
bool nodeflag_front = false;
bool nodeflag = false;
bool turning = false;
bool one_round = false;
bool left_turn = false;
bool right_turn = false;
bool finding_target_yaw = false;
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===              SETUP TO READ FROM FPGA                     ===
// ================================================================

#define RXD2 16
#define TXD2 17
byte reading;
unsigned int fpga_r;
inline size_t key(int i,int j) {return (size_t) i << 32 | (unsigned int) j;}


// ================================================================
// ===        SETUP TO READ FROM ULTRASONIC SENSOR              ===
// ================================================================

//TO-DO: CHANGE THE PINS
//!!!!!!!!!!!!!!!!!!!!
#define TRIGGER_PIN  12  
#define ECHO_PIN     11  
#define TRIGGER_PIN_2  12  
#define ECHO_PIN_2     11  
#define MAX_DISTANCE 200 

NewPing sonar1(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);


// ================================================================
// ===                    SETUP FOR SERVER                     ===
// ================================================================

// #define USE_WIFI_NINA         false
// #define USE_WIFI101           false

// #include <WiFiWebServer.h>
// #include <WiFiHttpClient.h>
// #include <ArduinoJson.h>

// const char ssid[] = "ALINA";
// const char pass[] = "02025509";
// char serverAddress[] = "172.20.10.2";  // server address
// int port = 3001;
// WiFiClient           client;
// WiFiWebSocketClient  wsClient(client, serverAddress, port);


// ================================================================
// ===                      MOTOR SETUP                     ===
// ================================================================



void setup() {
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
    displacement=0.0;
    wheelc=2*PI*0.0325;

    Serial.begin(115200);   //MIGHT NEED TO CHANGE TO 9600
    // WiFi.begin(ssid, pass);
    // while (WiFi.status() != WL_CONNECTED) {
    //   delay(1000);
    //   Serial.println("Connecting to WiFi...");
    // }
    // Serial.println("Connected to WiFi");
    delay(1000);
    
    //wsClient.begin();
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // initialize device
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

    //FGPA GETTING DATA
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    //Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);
  
}


std::vector<int> is_node(int x, int y, std::unordered_map<size_t, std::unordered_map<int, bool>> nodes);

void print_node(std::unordered_map<size_t, std::unordered_map<int, bool>> myMap){
  for (const auto& outerPair : myMap) {
        size_t outerKey = outerPair.first;
        const auto& innerMap = outerPair.second;

        Serial.println("Outer Key: ");
        Serial.println(outerKey);

        // Iterate over the inner map
        for (const auto& innerPair : innerMap) {
            int innerKey = innerPair.first;
            bool innerValue = innerPair.second;
            Serial.println("  Inner Key: ");
            Serial.println(innerKey);
            Serial.println("  Value: ");
            Serial.println(innerValue);
            
        }
  }

}

void motion(char command){
    if(command=='f'){   //TURN FORWARD BY 1 STEP
            digitalWrite(dirPin, HIGH);
            digitalWrite(dirPin2, LOW);
            digitalWrite(stepPin, HIGH);
            digitalWrite(stepPin2, HIGH);                   
            delayMicroseconds(2000);                     
            digitalWrite(stepPin, LOW);                 
            digitalWrite(stepPin2, LOW);                   
            delayMicroseconds(2000);
            displacement=wheelc/200;
            position[0]+=displacement*cos(yaw);
            position[1]+=displacement*sin(yaw);   
             
      }
      else if(command=='b'){  //TURN BACK BY 1 STEP
          digitalWrite(dirPin, LOW);
          digitalWrite(dirPin2, HIGH);
          digitalWrite(stepPin, HIGH);
          digitalWrite(stepPin2, HIGH);           
          delayMicroseconds(2000);                     
          digitalWrite(stepPin, LOW);                 
          digitalWrite(stepPin2, LOW);                   
          delayMicroseconds(2000); 
          displacement=wheelc/200;
          position[0]-=displacement*cos(yaw);
          position[1]-=displacement*sin(yaw);          
        
      }
       else if(command=='l'){   //TURN LEFT BY 1 STEP
            digitalWrite(dirPin, LOW);
            digitalWrite(dirPin2, LOW);
            digitalWrite(stepPin, HIGH);
            digitalWrite(stepPin2, HIGH);                  
            delayMicroseconds(2000);                     
            digitalWrite(stepPin, LOW);                 
            digitalWrite(stepPin2, LOW);                   
            delayMicroseconds(2000);      
          
        }
        
        else if(command=='r'){   //TURN RIGHT BY 1 STEP
              digitalWrite(dirPin, HIGH);
              digitalWrite(dirPin2, HIGH);
              digitalWrite(stepPin, HIGH);
              digitalWrite(stepPin2, HIGH);                  
              delayMicroseconds(2000);                     
              digitalWrite(stepPin, LOW);                 
              digitalWrite(stepPin2, LOW);                   
              delayMicroseconds(2000);
              
        }

        else if(command=='s'){  //STOP
              digitalWrite(dirPin, HIGH);
              digitalWrite(dirPin2, HIGH);
              digitalWrite(stepPin, LOW);
              digitalWrite(stepPin2, LOW);                  
              delayMicroseconds(2000);                     
              digitalWrite(stepPin, LOW);                 
              digitalWrite(stepPin2, LOW);                   
              delayMicroseconds(2000);
        }
}

float find_target_yaw(float l_initial_yaw, std::unordered_map<float, bool> &l_angles){
    //assumming first time visiting a node
    //assumming only one path on the right and one path on the left
    if (one_round || right_turn){
        if (l_initial_yaw > 90 && l_initial_yaw < 180){
            float max_angle = l_angles.begin()->first;
            float min_angle = l_angles.begin()->first;
            bool negative_exist = false;
            for (auto it : l_angles){
                if (it->first > max_angle && !it->second){
                    max_angle = it->first;
                }
                if (it->first < min_angle && !it->second){
                    min_angle = it->first;
                }
                if (it->first < 0 && !it->second){
                    negative_exist = true;
                }
                else {
                    positive_exist = true;
                }
            }
            if (negative_exist){
                l_angles[min_angle] = true;
                return min_angle;
            }
            else {
                l_angles[max_angle] = true;
                return max_angle;
            }
        }
        else{
            float max_angle = l_angles.begin()->first;
            for (auto it : l_angles){
                if (it->first > max_angle && !it->second){
                    max_angle = it->first;
                }
            }
            l_angles[max_angle] = true;
            return max_angle;
        }
        
    }
    else{
        if (nodeflag_front){
            l_angles[initial_yaw] = true;
            return initial_yaw;
        }
        else{
            float max_angle = l_angles.begin()->first;
            for (auto it : l_angles){
                if (it->first > max_angle && !it->second){
                    max_angle = it->first;
                }
            }
            l_angles[max_angle] = true;
            return max_angle;
        }
    }

}



void loop() {
   // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Ladtest packet 

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
    }

    command = '*';

    //check ultrasonic sensor readings to determine whether it is a node
    us1 = sonar1.ping();
    us2 = sonar2.ping();

    dis_left = sonar1.convert_cm(us1);
    dis_right = sonar2.convert_cm(us2);

    if (finding_target_yaw){
        if ((target_yaw < -178 && target_yaw >=-180) || (target_yaw > 178 && target_yaw <=180)){
            if (yaw < -177 || yaw > 177){
                //when target_yaw is reached, go forward by 20 steps to avoid checking node again
                finding_target_yaw = false;
                for (int i = 0; i < 15; i++){
                    motion('f');
                }
            }
            else if (target_yaw > yaw){
                for (int i = 0; i < 10; i++){
                    motion('r');
                }
            }
            else{
                for (int i = 0; i < 10; i++){
                    motion('l');
                }
            }
        }
        else {
            if (yaw < (target_yaw+2) || yaw > (target_yaw-2)){
                //when target_yaw is reached, go forward by 20 steps to avoid checking node again
                finding_target_yaw = false;
                for (int i = 0; i < 15; i++){
                    motion('f');
                }
            }
            else if (target_yaw > yaw){
                for (int i = 0; i < 10; i++){
                    motion('r');
                }
            }
            else{
                for (int i = 0; i < 10; i++){
                    motion('l');
                }
            }
        }
        
    }

    else {
        if (dis_left < 27 && dis_left > 3){
        nodeflag_left = true;
        }
        else{
            nodeflag_left = false;
        }

        if (dis_right < 27 && dis_right > 3){
            nodeflag_right = true;
        }
        else {
            nodeflag_right = false;
        }

        nodeflag = nodeflag_left || nodeflag_right;

        if (nodeflag && !turning){
            initial_yaw = yaw;
            turning = true;
            if (nodeflag_left && nodeflag_right){
                //turn 360 degree
                one_round = true;
                command = 'r';
            }
            else if (nodeflag_left){
                //turn 180 degree to the left
                left_turn = true;
                command = 'l';
            }
            else if (nodeflag_right){
                //turn 180 degree to the right
                right_turn = true;
                command = 'r';
            }
        }
        
        //start turning
        if (turning && one_round){
            if (initial_yaw > -180 && initial_yaw <=-175){
                if ((yaw < initial_yaw) || (yaw > (355 + initial_yaw))){
                    //stop turning and find target angle
                    target_yaw = find_target_yaw(initial_yaw, angles);
                    finding_target_yaw = true;
                    turning = false;
                    one_round = false;
                }
                else{
                    command = 'r';
                }
            }
            else{
                if (yaw >= (initial_yaw - 5) && yaw <= initial_yaw){
                    target_yaw = find_target_yaw(initial_yaw, angles);
                    finding_target_yaw = true;
                    turning = false;
                    one_round = false;
                }
                else{
                    command = 'r';
                }
            }
        }

        if (turning && left_turn){
            if (initial_yaw <= -90 && initial_yaw > -180){
                if (yaw > (initial_yaw + 180) && yaw < (initial_yaw + 185)){
                    target_yaw = find_target_yaw(initial_yaw, angles);
                    finding_target_yaw = true;
                    turning = false;
                    left_turn = false;
                }
                else{
                    command = 'l';
                }
            }
            else{
                if (yaw > initial_yaw - 175 && yaw < (initial_yaw - 180)){
                    target_yaw = find_target_yaw(initial_yaw, angles);
                    finding_target_yaw = true;
                    turning = false;
                    left_turn = false;
                }
                else {
                    command = 'l';
                }
            }
        }

        if (turning && right_turn){
            if (initial_yaw >= 90 && initial_yaw < 180){
                if (yaw <= (initial_yaw - 180) && yaw >= (initial_yaw - 185)){
                    target_yaw = find_target_yaw(initial_yaw, angles);
                    finding_target_yaw = true;
                    turning = false;
                    right_turn = false;
                }
                else{
                    command = 'r';
                }
            }
            else{
                if (yaw > initial_yaw + 175 && yaw < (initial_yaw + 180)){
                    target_yaw = find_target_yaw(initial_yaw, angles);
                    finding_target_yaw = true;
                    turning = false;
                    right_turn = false;
                }
                else {
                    command = 'r';
                }
            }
        }
    }
    
    motion(command);
      
      //PIN IS LEFT AND PIN2 IS RIGHT MOTOR
    print_node(nodes);

    delay(100);

}