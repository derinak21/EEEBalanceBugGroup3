

//SERVER SIDE CODE




//READ FROM FPGA
#define RXD2 16
#define TXD2 17
byte reading[4];
unsigned int fpga_r;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  //Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);
}

void loop() {
  while (Serial2.available()) {
    //Serial.println(Serial2.read(), HEX);
    // Serial.println(Serial2.read(), HEX);
    // Serial.println(Serial2.read(), HEX);
    // Serial.println(Serial2.read(), HEX);
    Serial2.readBytes(reading, 4);
    fpga_r = reading[3]<<24 | reading[2]<<16 | reading[1]<<8 |reading[0];
    //fpga_r = reading[0]<<24 | reading[1]<<16 | reading[2]<<8 |reading[3];
    Serial.println(fpga_r,HEX);
  }  
}

//DECODE FPGA

//CONVERT FPGA AND SERVER COMMANDS TO DELAY IN MICROSECONDS CODE (COMBINED NO PID)
//ROTATE BOTH MOTORS TO TURN LEFT OR RIGHT SO POSITION STAYS THE SAME
//GET POSITION FROM DEAD-RECKONING BETWEEN NODES
//GET POSITION FROM BEACONS AT A NODE
//SEND POS TO SERVER 

