#include <bits/stdc++.h>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
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
char command; 
float x, y;
bool turn;
int camera_command;
int direction;
char color;
std::vector<char> initbeacons;
std::map<char, std::pair<float, float>> beaconMap;
std::unordered_map<size_t, std::unordered_map<int, bool>> nodes;

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

#define USE_WIFI_NINA         false
#define USE_WIFI101           false

#include <WiFiWebServer.h>
#include <WiFiHttpClient.h>
#include <ArduinoJson.h>

const char ssid[] = "ALINA";
const char pass[] = "02025509";
char serverAddress[] = "172.20.10.2";  // server address
int port = 3001;
WiFiClient           client;
WiFiWebSocketClient  wsClient(client, serverAddress, port);



int fake_main() {
  AdjacencyList adjList;
  coords a = make_pair(100, 0);
  coords b = make_pair(0, 0);
  coords c = make_pair(0, 500);
  coords d = make_pair(200, 500);
  coords e = make_pair(300, 500);
  coords f = make_pair(200, 0);
  coords g = make_pair(260, 0);
  coords h = make_pair(600, 300);
  coords i = make_pair(270, 350);
  coords j = make_pair(400, 200);
  coords k = make_pair(600, 500);
  coords l = make_pair(850, 500);
  coords m = make_pair(850, 300);
  coords n = make_pair(650, 0);
  coords o = make_pair(850, 0);
  // Add edges to the adjacency list
  addEdge(adjList, a, b);
  addEdge(adjList, b, c);
  addEdge(adjList, c, d);
  addEdge(adjList, d, e);
  addEdge(adjList, e, i);
  addEdge(adjList, i, j);
  addEdge(adjList, d, f);
  addEdge(adjList, f, g);
  addEdge(adjList, g, h);
  addEdge(adjList, h, n);
  addEdge(adjList, n, o);
  addEdge(adjList, h, m);
  addEdge(adjList, m, l);
  addEdge(adjList, l, k);
  addEdge(adjList, k, h);
  addEdge(adjList, k, e);

  // Print the adjacency list
  printAdjacencyList(adjList);
  shortestPath(c, o, adjList);

  return 0;
}



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
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
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

//typedef std::unordered_map<size_t, std::unordered_map<int, bool>> node_map;

coords if_is_node(int x, int y, node_map nodes){
    node_map::iterator it;
    coords coord = make_pair(555,555);
    for (it = nodes.begin(); it!=nodes.end(); it++){
        int tmp_x = it->first>>32;
        int tmp_y = ((it->first)<<32)>>32;
        int x_diff = tmp_x-x;
        int y_diff = tmp_y-y;
        if ((x_diff>-20) & (x_diff<20) & (y_diff>-20) & (x_diff<20)){
            coord = make_pair(tmp_x,tmp_y);
            return coord;
        }
    }
    return coord;
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
// ================================================================
// ===                    SETUP FOR SHORTEST_PATH                     ===
// ================================================================

using namespace std;

typedef pair<int, int> coords;
typedef pair<int, coords> iPair;
typedef pair<coords, double> weighted_node;

// Hash function using the provided key function
inline size_t key(int i, int j) { return (size_t)i << 32 | (unsigned int)j; }

// Custom hash function for Coordinate
struct WHash {
  size_t operator()(const weighted_node &node) const {
    return key(node.first.first, node.first.second);
  }
};
struct CoordinateHash {
  size_t operator()(const coords &coordinate) const {
    return key(coordinate.first, coordinate.second);
  }
};

// Define a data structure to represent the adjacency list
typedef unordered_map<coords, unordered_set<weighted_node, WHash>, CoordinateHash> AdjacencyList;

// Function to add an edge to the adjacency list
void addEdge(AdjacencyList &adjList, coords &src, coords &dest) {  
  double weight = sqrt(pow((dest.first - src.first), 2) + pow((dest.second - src.second), 2));
  adjList[src].insert(make_pair(dest, weight));
  adjList[dest].insert(make_pair(src, weight));
}



void shortestPath(coords src, coords dest, AdjacencyList adjList) {
  priority_queue<iPair, vector<iPair>, greater<iPair>> pq;
  unordered_map<coords, int, CoordinateHash> dist;
  unordered_map<coords, coords, CoordinateHash> prev;
  for (const auto& pair : adjList) {
        dist[pair.first] = 100000;
    }
  pq.push(make_pair(0, src));
  dist[src] = 0;
  
  while (!pq.empty()) {
    coords u = pq.top().second;
    pq.pop();

    for (const auto& neighbor : adjList[u]) {
      // Get vertex label and weight of the current
      // neighbor of u.
      coords v = neighbor.first;
      int weight = neighbor.second;

      // If there is a shorter path to v through u.
      if (dist[v] > dist[u] + weight) {
        // Updating the distance of v
        dist[v] = dist[u] + weight;
        prev[v] = u;
        pq.push(make_pair(dist[v], v));
      }
    }
  }
  printf("Shortest distance from Source to Dest: %d\n", dist[dest]);

  // Construct the path from source to destination
  vector<coords> path;
  coords current = dest;
  while (current != src) {
    path.push_back(current);
    current = prev[current];
  }
  path.push_back(src);

  // Print the path
  cout << "Path: ";
  for (int i = path.size() - 1; i >= 0; --i) {
    cout << "(" << path[i].first << " " << path[i].second << ")";
    if (i != 0) {
      cout << " -> ";
    }
  }
  cout << endl;
}



// Function to print the adjacency list
void printAdjacencyList(const AdjacencyList &adjList) {
  for (const auto &pair : adjList) {
    cout << "Node (" << pair.first.first << ", " << pair.first.second << "): ";
    for (const auto &neighbor : pair.second) {
      cout << "( (" << neighbor.first.first << ", " << neighbor.first.second
           << "), " << neighbor.second << ")";
    }
    cout << endl;
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

    //start turning
    if (!first_time_node){
        initial_yaw = yaw;
        first_time_node = true;

        //set coordinates of this node
        x = position[0];
        y = position[1];

        int count_reading = 0;
        while(Serial2.available() && (count_reading != 14)) {
            //Serial.print("fpga");
            reading = Serial2.read();  
            Serial.println(reading,HEX);
            if (reading == 1){
              Serial.println("forward");
                //forward: is a path ahead
                nodes[key(x,y)][initial_yaw] = false; //add the forward path to the node
                forward_path = true;
                break;
            }
            else{
              Serial.println("stop");
              break;
            }
            count_reading += 1;
        } 
        command = 'r'; 
    }
    else{
        int count_reading = 0;
        bool path_ahead = false;
        while(Serial2.available() && (count_reading != 14)) {
            Serial.print("fpga");
            reading = Serial2.read();  
            Serial.println(reading,HEX);
            if (reading == 1){
                //forward: is a path ahead
                Serial.println("forward");
                path_ahead = true;
                break;
            }
            else{
              Serial.println("stop");
              break;
            }
            count_reading += 1;
        } 
        if (forward_path && !path_ahead){
            forward_path = false;
        }
        else if ((is_path == false) && (!forward_path) && path_ahead){
            is_path = true;
            min_angle = yaw;
        }
        else if (is_path && (!path_ahead)){
            is_path = false;
            max_angle = yaw;

            float path_angle = (min_angle+max_angle)/2;

            if ((max_angle <0) && (min_angle>0)){
                if (path_angle <0){
                    path_angle = -180 - path_angle;
                }
                else if (path_angle > 0){
                    path_angle = 180 - path_angle;
                }
                else {
                    path_angle = 180;
                }
            }
            nodes[key(x,y)][path_angle] = false;

            min_angle = 360;
            max_angle = -360;
        }
        command = 'r';
    }

// ================================================================
// ===                      MOTION CODE                       ===
// ================================================================

      
      //PIN IS LEFT AND PIN2 IS RIGHT MOTOR

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
        print_node(nodes);

        delay(100);

        //

}
