#include "I2Cdev.h"
#include <unordered_map>
#include <map>
#include <vector>
#include <NewPing.h>
#include <cstdlib>
// #include <AccelStepper.h>


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
std::vector<float> position(2, 0.0);
std::vector<float> position_rl(2, 4.0);
std::vector<float> position_ll(2, 4.0);
std::vector<float> position_rl2(2, 4.0);
std::vector<float> position_ll2(2, 4.0);
std::vector<float> position_b1(2, 0.0);
std::vector<float> position_b2(2, 0.0);
float position_p1; 
float position_p2;
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
float min_node_angle = 360;
float max_node_angle = -360;
int dt=0, t=0, prevtime=0;
float d0 = 0.58; //DEFINE THIS
float d1 = 0.075; //DEFINE THIS

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
std::unordered_map<size_t, std::unordered_map<float, bool>> nodes;
std::unordered_map<float, bool> angles;
int node_count = 0;

int wscount = 0;

//ultrasonic sensors
int dis_left;
int dis_right;
unsigned int us1;
unsigned int us2;
int turning_count = 0;
int correct_left_count = 0;
int correct_right_count = 0;
int path_right_count = 0;

int node_count_left = 0;
int node_count_right = 0;
int current_node; // keep track of next node the rover is going to when finding_node is true
std::unordered_map<size_t, std::vector<std::pair<size_t, float>>> adjacent_nodes;//float stores the direction to value node if rover is at key node
std::unordered_map<size_t, bool> explored_nodes;
std::vector<size_t> tmp_coord;
// ================================================================
// ===                          FLAGS                           ===
// ================================================================
bool beacon_flag_1 = false; 
bool beacon_flag_2 = false; 
bool check_node = false;
bool nodeflag = false; 
//bool nodeflag_left = false;
//bool nodeflag_right = false;
bool first_time_node = false; 
bool initialisation=true;
bool initialbeacon=true;
bool beaconposition=true;
bool beacon_flag_fpga = false; 
bool de_flag=false;
bool done_checking=false;
bool is_path = false;
bool forward_path = false;
bool path_ahead = false;

bool nodeflag_left = false;
bool nodeflag_right = false;
bool nodeflag_front = false;
//bool nodeflag = false;
bool turning = false;
bool one_round = false;
bool left_turn = false;
bool right_turn = false;
bool finding_target_yaw = false;
bool to_left = false;
bool to_right = false;
bool positive_exist = false;
bool path_right = false;
bool sensor_left = false;
bool sensor_right = false;
bool not_forward_path = false;

bool finding_node = false;//true when the rover is on the way to a know node
bool angle_offset = false;
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
#define TXD2 26
byte reading;
unsigned int fpga_r;
inline size_t key(int i,int j) {return (size_t) i << 32 | (unsigned int) j;}


// ================================================================
// ===        SETUP TO READ FROM ULTRASONIC SENSOR              ===
// ================================================================

//TO-DO: CHANGE THE PINS
//!!!!!!!!!!!!!!!!!!!!
#define TRIGGER_PIN  18  
#define ECHO_PIN     19  
#define TRIGGER_PIN_2  17  
#define ECHO_PIN_2     5  
#define MAX_DISTANCE 200 

NewPing sonar1(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);


// ================================================================
// ===                      MOTOR SETUP                     ===
// ================================================================



// ================================================================
// ===                    SETUP FOR SERVER                     ===
// ================================================================

// #define USE_WIFI_NINA         false
// #define USE_WIFI101           false

// //#include <WiFiWebServer.h>
// #include <ArduinoWebsockets.h>
// #include <WiFiHttpClient.h>
// #include <ArduinoJson.h>

// const char ssid[] = "ALINA";
// const char pass[] = "02025509";
// // char serverAddress[] = "172.20.10.2";  // server address
// // int port = 3001;
// const char* websockets_server_host = "172.20.10.2"; //Enter server adress
// const uint16_t websockets_server_port = 3001; // Enter server port
// using namespace websockets;
// WebsocketsClient client;
// bool connect;
// unsigned long lastConnectionTime = 0;
// const unsigned long connectionInterval = 21000;
// unsigned long start;




// ================================================================
// ===                      MOTOR SETUP                     ===
// ================================================================

// AccelStepper m1(AccelStepper::DRIVER, stepPin, dirPin); //motor left
// AccelStepper m2(AccelStepper::DRIVER, stepPin2, dirPin2);   //motor right


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

    // m1.setMaxSpeed(2000);
    // m2.setMaxSpeed(2000);
    // m1.setAcceleration(10);
    // m2.setAcceleration(10);

    position_rl[1] = 195;
    position_ll[1] = 195;

    Serial.begin(115200);   //MIGHT NEED TO CHANGE TO 9600
    // WiFi.begin(ssid, pass);
    // while (WiFi.status() != WL_CONNECTED) {
    //   delay(1000);
    //   Serial.println("Connecting to WiFi...");
    // }
    // Serial.println("Connected to WiFi");
    // delay(1000);
    
    // connect = client.connect(websockets_server_host, websockets_server_port, "/");
    // if (connect){
    //   Serial.println("connected");
    // }
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // initialize device
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


// void websocket_send(){
//   if(client.available()){
//         DynamicJsonDocument jsonDocument(2048);
        
//         //ROVER POSITION
//         jsonDocument["x_r"] = position[0]*400/3.6 +10;
//         jsonDocument["y_r"] = (3.6-position[1])*400/3.6;

//         //LEFT WHITE LED POSITION
//         jsonDocument["x_ll"] = position_ll[0]*400/3.6 + 10;
//         jsonDocument["y_ll"] = (3.6-position_ll[1])*400/3.6;

        
//         //RIGHT WHITE LED POSITION         
//         jsonDocument["x_rl"] = position_rl[0]*400/3.6+10;
//         jsonDocument["y_rl"] = (3.6-position_rl[1])*400/3.6;

//         jsonDocument["x_sll"] = position_ll2[0]*400/3.6+10;
//         jsonDocument["y_sll"] = (3.6-position_ll2[1])*400/3.6;

//         jsonDocument["x_srl"] = position_rl2[0]*400/3.6+10;
//         jsonDocument["y_srl"] = (3.6-position_rl2[1])*400/3.6;
        
//         String output;  // Serialize the JSON document to a string
//         serializeJson(jsonDocument, output);
                
//         Serial.println("Sending data ");
//         Serial.println("Sending data ");
//         Serial.println("Sending data ");
//         Serial.println("Sending data ");
//         Serial.println("Sending data ");


//         // unsigned long start = millis();
//         client.send(output);
//         // unsigned long end = millis();
//         // Serial.println(end-start);
//         jsonDocument.clear();


//     } 
//     else{
//       Serial.println("disconnected");

//     }
    
// }
std::vector<int> is_node(int x, int y, std::unordered_map<size_t, std::unordered_map<float, bool>> nodes){
    std::unordered_map<size_t, std::unordered_map<float, bool>>::iterator it;
    std::vector<int> coord(2,555);
    for (it = nodes.begin(); it!=nodes.end(); it++){
        int tmp_x = it->first>>32;
        int tmp_y = ((it->first)<<32)>>32;
        int x_diff = tmp_x-position[0];
        int y_diff = tmp_y-position[1];
        if ((x_diff>-0.2) && (x_diff<0.2) && (y_diff>-0.2) && (x_diff<0.2)){
            coord[0] = tmp_x;
            coord[1] = tmp_y;
            return coord;
        }
    }
    return coord;
}

void motion(char command){
    if(command=='f'){   //TURN FORWARD BY 1 STEP
            // t=millis();
            // m1.setSpeed(50);
            // m2.setSpeed(-50);
            // m1.runSpeed();  // Acceleration in steps per second per second
            // m2.runSpeed();
            digitalWrite(dirPin, HIGH);
            digitalWrite(dirPin2, LOW);
            digitalWrite(stepPin, HIGH);
            digitalWrite(stepPin2, HIGH);                   
            delayMicroseconds(5000);                     
            digitalWrite(stepPin, LOW);                 
            digitalWrite(stepPin2, LOW);                   
            delayMicroseconds(5000);
            displacement=wheelc/200;
            Serial.print(" f ");
            // dt=2;
            // displacement=m1.speed()*dt;
            position[0]+=displacement*cos(yaw);
            position[1]+=displacement*sin(yaw);   
             
      }
      else if(command=='b'){  //TURN BACK BY 1 STEP
            // m1.setSpeed(-500);
            // m2.setSpeed(500);
            // m1.runSpeed();  // Acceleration in steps per second per second
            // m2.runSpeed();
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
          // prevtime=t;         
        
      }
       else if(command=='l'){   //TURN LEFT BY 1 STEP
              // m1.setSpeed(-20);
              // m2.setSpeed(-20);
              // m1.runSpeed();  // Acceleration in steps per second per second
              // m2.runSpeed();              
            digitalWrite(dirPin, LOW);
            digitalWrite(dirPin2, LOW);
            digitalWrite(stepPin, HIGH);
            digitalWrite(stepPin2, HIGH);                  
            delayMicroseconds(6000);                     
            digitalWrite(stepPin, LOW);                 
            digitalWrite(stepPin2, LOW);                   
            delayMicroseconds(6000); 
            Serial.print(" l ");
     
          
        }
        
        else if(command=='r'){   //TURN RIGHT BY 1 STEP
              // m1.setSpeed(20);
              // m2.setSpeed(20);
              // m1.runSpeed();  // Acceleration in steps per second per second
              // m2.runSpeed();
              digitalWrite(dirPin, HIGH);
              digitalWrite(dirPin2, HIGH);
              digitalWrite(stepPin, HIGH);
              digitalWrite(stepPin2, HIGH);                  
              delayMicroseconds(6000);                     
              digitalWrite(stepPin, LOW);                 
              digitalWrite(stepPin2, LOW);                   
              delayMicroseconds(6000);
              Serial.print(" r ");

        }

        else if(command=='s'){  //STOP
              // m1.setSpeed(0);
              // m2.setSpeed(0);
              // m1.runSpeed();  // Acceleration in steps per second per second
              // m2.runSpeed();
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
    std::unordered_map<float, bool>::iterator it;
    Serial.print("number of paths: ");
    Serial.print(l_angles.size());
    for (it = l_angles.begin(); it != l_angles.end(); it++){
      Serial.print(it->first);
      Serial.print(" ");
      Serial.print(it->second);
      Serial.print(" ");
    }
    if (l_angles.size()==2){
        for (auto it = l_angles.begin(); it != l_angles.end(); it++){
            if (!(abs(it->first-yaw)>=(PI-0.33) && abs(it->first-yaw)<=(PI+0.33))){
                return it->first;
                Serial.print("yaw: ");
                Serial.print(it->first);
            }
        }
    }   
    
    else{
        if (one_round || right_turn){
            if (l_initial_yaw > 1.57 && l_initial_yaw < PI){
                float max_angle = l_angles.begin()->first;
                float min_angle = l_angles.begin()->first;
                bool negative_exist = false;
                for (it = l_angles.begin(); it != l_angles.end(); it++){
                    if (l_angles[max_angle]==true){
                        max_angle = it->first;
                    }
                    if (l_angles[min_angle]==true){
                        min_angle = it->first;
                    }
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
                if (negative_exist && l_angles[min_angle] == false){
                    l_angles[min_angle] = true;
                    return min_angle;
                }
                else if (l_angles[max_angle] == false){
                    l_angles[max_angle] = true;
                    return max_angle;
                }
            }
            else{
                float max_angle = l_angles.begin()->first;
                for (it = l_angles.begin(); it != l_angles.end(); it++){
                    if (l_angles[max_angle]==true){
                        max_angle = it->first;
                    }
                    if (it->first > max_angle && !it->second){
                        max_angle = it->first;
                    }
                }
                if (l_angles[max_angle]==false){
                    l_angles[max_angle] = true;
                    return max_angle;
                }
            }
            
        }
        else{    
            float max_angle = l_angles.begin()->first;
            for (it = l_angles.begin(); it != l_angles.end(); it++){
                if (it->first > (initial_yaw-0.07) && it->first < (initial_yaw+0.07)){
                      it->second = true;
                      return it->first;
                  }
                if (l_angles[max_angle]==true){
                    max_angle = it->first;
                }
                if (it->first > max_angle && !it->second){
                    max_angle = it->first;
                }
            }
            if (l_angles[max_angle]==false){
                l_angles[max_angle] = true;
                return max_angle;
            }  
        }
    }

    //will reach this point when all paths at this node has been visited
    
    // std::vector<int> node_coord = is_node(x,y,nodes);
    // size_t tmp = key(node_coord[0], node_coord[1]);
    // std::unordered_map<size_t, std::vector<size_t>>::const_iterator it2;
    // it2 = adjacent_nodes.find(tmp);
    // std::vector<size_t> tmp_nodes = it2->second;
    // std::unordered_map<size_t, bool>::iterator it3;
    // for (int i = 0; i<tmp_nodes.size(); i++){
    //     it3 = explored_nodes.find(tmp_nodes[i].first);
    //     if (!it3->second){
    //         return tmp_nodes[i].second;//to find direction to next node not fully explored
    //     }
    // }

    // //when all adjacent node is fully explored too
    // for (it3 = explored_nodes.begin(); it3!=explored_nodes.end(); it3++){
    //     if (!it3->second){
    //         tmp_coord = shortest_path(tmp, it3->first);//need to return a list of nodes to visit including the target node
    //         //going to tmp_coord from tmp
    //         for (int i = 0; i<tmp_nodes.size(); i++){
    //             if (tmp_nodes[i].first == tmp_coord[0]){
    //                 finding_node = true;
    //                 current_node = 0;//keep track of the node that the rover is on the way to
    //                 return tmp_nodes[i].second;
    //             }
    //         }            //need to figure out a way to store all nodes to be visited
    //     }
    // }
    
}

void calculate_path_angle(float &path_angle){
  if (path_angle > PI/2){
            path_angle = path_angle - 1.5*PI;
        }
        else {
            path_angle = path_angle + 0.48*PI;
        }
        Serial.print("path angle");
        Serial.println(path_angle);

        if (!forward_path){
          path_angle = path_angle - 0.16*PI;
        }
        

        if (path_angle < -PI){
          path_angle = -PI+0.05;
        }

        if (abs(path_angle-initial_yaw)>PI-0.33 && abs(path_angle-initial_yaw)<PI+0.33){
          angles[path_angle] = true;
        }
        else{
          angles[path_angle] = false;
        }
        

        min_node_angle = 360;
        max_node_angle = -360;
}

void add_node(){
    //forward_path is set true when initial_yaw is one of the paths
    //path_ahead value depends on sensor reading
    //Serial.println("add node");
    Serial.print("forward_path ");
    Serial.print(forward_path);
    Serial.print("not_forward_path ");
    Serial.print(not_forward_path);

    if (forward_path && not_forward_path){
            forward_path = false;
    }
    if (!is_path && path_right && !forward_path){
      Serial.print("min yaw");
        is_path = true;
        min_node_angle = yaw;
    }
    else if (is_path && !path_right && min_node_angle != 360){
      Serial.print("calculate node");
        is_path = false;
        max_node_angle = yaw;

        float path_angle = (min_node_angle+max_node_angle)/2;

        if (max_node_angle <0 && min_node_angle>0){
            if (path_angle <0){
                path_angle = PI + path_angle;
            }
            else if (path_angle > 0){
                path_angle = -PI + path_angle;
            }
            else {
                path_angle = PI;
            }
        }

        //change direction of rover to direction of sensor on the right of rover
        calculate_path_angle(path_angle);
        
    }
}


void loop() {
  //Serial.println("void loop");
      // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            // Serial.print("ypr\t");
            
            // Serial.print(ypr[0] * 180/M_PI);
            // Serial.print("\t");
            // Serial.print(ypr[1] * 180/M_PI);
            // Serial.print("\t");
            // Serial.println(ypr[2] * 180/M_PI);
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
            // Serial.print("aworld\t");
            // Serial.print(aaWorld.x);
            // Serial.print("\t");
            // Serial.print(aaWorld.y);
            // Serial.print("\t");
            // Serial.println(aaWorld.z);
        #endif
    }
// ================================================================
// ===                      READ FROM CAMERA/FPGA                ===
// ================================================================
    position_p1 = 1000;
    position_p2 = 1000;
    int count_reading = 0;
    int count_coordinates = 0;
//& (count_reading != 14)
    while(Serial2.available() && (count_reading != 14)) {
        //Serial.print("fpga");
        reading = Serial2.read();  
        //Serial.println(reading,HEX);
        if (reading > 127) {
            if (count_coordinates == 0){
                position_p1 = reading - 128;
                count_coordinates += 1;
            }
            else if (count_coordinates == 1){
                position_p2 = reading - 128;
                break;
            }
        }
        count_reading += 1;
    }  

// ================================================================
// ===                      READ FROM sensor                ===
// ================================================================
  us1 = sonar1.ping();
  dis_left = sonar1.convert_cm(us1);
  us2 = sonar2.ping();
  dis_right = sonar2.convert_cm(us2);

  if (dis_left > 1 && dis_left < 5){
    sensor_right = true;
  }
  else{
    sensor_right = false;
  }

  if (dis_right > 1 && dis_right < 5){
    sensor_left = true;
  }
  else{
    sensor_left = false;
  }

  if (!turning && !finding_target_yaw && dis_left < 35 && dis_left > 3 && dis_right < 35 && dis_right > 3){
      if (dis_left>dis_right+2){
        motion('l');
        motion('l');
      }
      else if (dis_left < dis_right-2){
        motion('r');
        motion('r');
      }
  }
        

  //  Serial.print("dis_left ");
  //  Serial.println(dis_left);

  // Serial.print("dis_right ");
  // Serial.println(dis_right);
// ================================================================
// ===               GET WHITE LED POSITION                      === 
// ================================================================

    //TO-DO: CALCULATE d0 AND d1 !!!
    if (position_p1 != 1000){
      position_ll[0]=position[0]+d0*cos(yaw)+(position_p1-40)*sin(yaw)*0.545/80;
      // Serial.print("pos llx");
      // Serial.println(position_ll[0]);
    }
    else{
      position_ll[0] = 4;
    }
    position_ll[1]=position[1]+d0*sin(yaw)+(position_p1-40)*cos(yaw)*0.545/80;
    if (position_p2 != 1000){
        position_rl[0]=position[0]+d0*cos(yaw)+(position_p2-40)*sin(yaw)*0.545/80;
        // Serial.print("pos rlx");
        // Serial.println(position_rl[0]);
    } 
    else{
      position_rl[0] = 4;
    }
    position_rl[1]=position[1]+d0*sin(yaw)+(position_p2-40)*cos(yaw)*0.545/80; 
    // Serial.println("disleft");
    // Serial.println(dis_left);
    // Serial.println("disright");
    // Serial.println(dis_right);
    // if (dis_left >4 && dis_left <26 ){
    //   position_ll2[0]=position[0]+d1*cos(yaw)+dis_left*sin(yaw)/100;
    //   position_ll2[1]=position[1]+d1*sin(yaw)+dis_left*cos(yaw)/100;
    // }

    // if (dis_right >4 && dis_right <26 ){
    //   position_rl2[0]=position[0]+d1*cos(yaw)-dis_right*sin(yaw)/100;
    //   position_rl2[1]=position[1]+d1*sin(yaw)-dis_right*cos(yaw)/100;
    // }

  //angles[1.54] = false;
    //angles[3.14] = false;

    command = '*';

    // count_reading = 0;
    // while(Serial2.available() && (count_reading != 14)) {
    //     //Serial.print("fpga");
    //     reading = Serial2.read();  
    //     //Serial.println(reading,HEX);
    //     if (reading == 2) {
    //       to_left = true;
    //       break;
    //     }
    //     else {
    //       to_left = false;
    //     }
    //     if (reading == 3){
    //       to_right = true;
    //       break;
    //     }
    //     else{
    //       to_right = false;
    //     }
    //     count_reading += 1;
    // }  

  if (finding_target_yaw){
    correct_left_count = 0;
    correct_right_count = 0;
    Serial.print("finding_target_yaw");
    //angles[-3.1] = false;
        if ((target_yaw < -3.1 && target_yaw >=-3.14) || (target_yaw > 3.1 && target_yaw <= 3.14)){
            if (yaw < -3.09 || yaw > 3.09){
                //when target_yaw is reached, go forward by 20 steps to avoid checking node again
                finding_target_yaw = false;
                one_round = false;
                left_turn = false;
                right_turn = false;
                // client.connect(websockets_server_host, websockets_server_port, "/");
                // lastConnectionTime = millis();
                for (int i = 0; i < 270; i++){
                    motion('f');
                }
            }
            else if (target_yaw > yaw){
              motion('r');
            }
            else{
                
              motion('l');
                
            }
        }
        else {
            if (yaw < (target_yaw+0.025) && yaw > (target_yaw-0.025)){
                //when target_yaw is reached, go forward by 20 steps to avoid checking node again
                finding_target_yaw = false;
                one_round = false;
                left_turn = false;
                right_turn = false;
                for (int i = 0; i < 270; i++){
                    motion('f');
                }
                //angles[-3.1] = false;
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

    else if ((sensor_left) && !turning && correct_left_count <8){
      //Serial.println("to_left");
      motion('l');
      //to_left = false;
      correct_left_count += 1;
      delay(20);
      if (correct_left_count == 8){
        for (int i = 0; i<20; i++){
          motion('f');
        }
      }
    }

    else if ((sensor_right) && !turning && correct_right_count <8){
      //Serial.println("to_right");
      motion('r');
      //to_right = false;
      correct_right_count += 1;
      delay(20);
      if (correct_right_count == 8){
        for (int i = 0; i<20; i++){
          motion('f');
        }
      }
    }

    else {
      correct_left_count = 0;
      correct_right_count = 0;
      //check ultrasonic sensor readings to determine whether it is a node     
      us1 = sonar1.ping();
      dis_left = sonar1.convert_cm(us1);
      us2 = sonar2.ping();
      dis_right = sonar2.convert_cm(us2);

      Serial.print("dis_right: ");
      Serial.print(dis_right);

      if (dis_left >4 && dis_left <26 ){
        position_ll2[0]=position[0]+d1*cos(yaw)+dis_left*sin(yaw)/100;
        position_ll2[1]=position[1]+d1*sin(yaw)+dis_left*cos(yaw)/100;
      }

      if (dis_right >4 && dis_right <26 ){
        position_rl2[0]=position[0]+d1*cos(yaw)-dis_right*sin(yaw)/100;
        position_rl2[1]=position[1]+d1*sin(yaw)-dis_right*cos(yaw)/100;
      }

      if (dis_left < 35 && dis_left > 3){
        node_count_left = 0;
        }
        else{
            node_count_left += 1;
        }

    if (node_count_left >= 20){
        nodeflag_left = true;
        node_count_left = 0;
    }

    //to determine whether there is a node on the right
    if (dis_right < 35 && dis_right > 1){
        node_count_right = 0;
        path_right_count += 1;
        
        }
        else{
            node_count_right += 1;
            path_right_count = 0;
            not_forward_path = false;
        }

    if (node_count_right == 20){
        nodeflag_right = true;
        node_count_right = 0;
        path_right = true;
    }

    if (path_right_count==20){
      path_right = false;
      not_forward_path = true;
    }

    //to determine whether current position is a node
    nodeflag = nodeflag_left || nodeflag_right;
      Serial.print("nodeflag ");
      // Serial.print(" ");
      Serial.print(nodeflag);

      Serial.print("dis_left");
      Serial.print(dis_left);

      // Serial.print("yaw");
      // Serial.println(yaw);

      // Serial.print("turning 1:");
      // Serial.println(turning);

      if (path_right){
        path_ahead = true;
      }
      //Serial.println(dis_right);
      std::vector<int> node_coord = is_node(x,y,nodes);
      Serial.print ("is_node return: ");
      Serial.print(node_coord[1]);
      
      if (nodeflag && node_coord[1]==555){
        //Serial.println("new_node");
        //Serial.println(path_right);
          if (nodeflag && !turning){
            turning = true;
            initial_yaw = yaw;
            if (nodeflag_right){
              forward_path = true;
              calculate_path_angle(yaw);
            }
            if (nodeflag_left && nodeflag_right){
                //turn 360 degree
                one_round = true;
            }
            else if (nodeflag_left){
                //turn 180 degree to the left
                left_turn = true;
            }
            else if (nodeflag_right){
                //turn 180 degree to the right
                right_turn = true;
                angle_offset = true;
            }
            for (int i = 0; i<100; i++){//was 110, 130
              motion('f');
            }
          }

          else if (turning_count > 100 && turning && yaw >= (initial_yaw - 0.03) && yaw < (initial_yaw)){
            turning = false;
            finding_target_yaw = true;
            Serial.print("before find target yaw");
            target_yaw = find_target_yaw(initial_yaw, angles);
            Serial.print("after find target yaw");
            const size_t current_xy = key(x,y);
            nodes[current_xy] = angles;
            angles.clear();
          }
          else if (turning) {
            motion('r');
            turning_count += 1;
            //delay(10);
            add_node();
          }
      }
      else{
        motion('f');
        motion('f');
        motion('f');
      }
    }
    if (is_path){
      Serial.print("is_path true ");
    }
    else{
      Serial.print("is_path false ");
    }
    Serial.print("x: ");
    Serial.print(position[0]);
    Serial.print("y: ");
    Serial.print(position[1]);
    Serial.print("yaw: ");
    Serial.println(yaw);
    
    
    //delay(30);
}