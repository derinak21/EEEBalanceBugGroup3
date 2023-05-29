/*Example sketch to control a stepper motor with A4988 stepper motor driver and Arduino without a library. More info: https://www.makerguides.com */

// Define stepper motor connections and steps per revolution:
#define dirPin 12
#define stepPin 14
#define stepsPerRevolution        //LOOK INTO THIS

#define dirPin2 27
#define stepPin2 26

int revolutioncount=0;

void setup() {
  Serial.begin(9600);
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
}

void loop() {


  // Set the spinning direction counterclockwise:
  digitalWrite(dirPin, LOW);
  digitalWrite(dirPin2, HIGH);

  //Spin the stepper motor 5 revolutions fast:
  while (1){
    // These four lines result in 1 step:
    // digitalWrite(stepPin, HIGH);
    // digitalWrite(stepPin2, HIGH);
    // delayMicroseconds(2000);

    // digitalWrite(stepPin, LOW);
    // digitalWrite(stepPin2, LOW);
    // delayMicroseconds(2000);

    if (Serial.available()) {
      char command = Serial.read();
      switch (command) {

        case 'w':
          Serial.println("forward");
          digitalWrite(dirPin, LOW);                      
          digitalWrite(dirPin2, HIGH);                     

          for (int i = 0; i < stepsPerRevolution; i++) {
            digitalWrite(stepPin, HIGH);                  
            digitalWrite(stepPin2, HIGH);                   
            delayMicroseconds(2000);                     
            digitalWrite(stepPin, LOW);                 
            digitalWrite(stepPin2, LOW);                   
            delayMicroseconds(2000);
          }
          revolutioncount++;                   
          break;

        case 's':
          Serial.println("backward");
          digitalWrite(dirPin, HIGH);                      
          digitalWrite(dirPin2, LOW);                     


          // delete for loops and use the steps to calculate the displacement
          // change the delaymicroseconds accordingly 
          // test it to see if this would work 
          for (int i = 0; i < stepsPerRevolution; i++) {
            digitalWrite(stepPin, HIGH);                  
            digitalWrite(stepPin2, HIGH);                   
            delayMicroseconds(2000);                     
            digitalWrite(stepPin, LOW);                 
            digitalWrite(stepPin2, LOW);                   
            delayMicroseconds(2000); 
          }
          revolutioncount++; 
          break;

        //I AM NOT SURE ABOUT HOW TO COUNT REVOLUTIONS IN LEFT AND RIGHT
        //ASSUME THAT PIN IS LEFT AND PIN2 IS CONNECTED TO RIGHT MOTOR
        case 'a':
          Serial.println("left");
          digitalWrite(dirPin, HIGH);                      
          digitalWrite(dirPin2, LOW);                     

          for (int i = 0; i < stepsPerRevolution; i++) {  //PROBABLY NEED TO CHANGE THIS PART BECAUSE IT CANNOT TURN LEFT WITH ONE REVOLUTION
            digitalWrite(stepPin, LOW);                  
            digitalWrite(stepPin2, HIGH);                   
            delayMicroseconds(2000);                     
            digitalWrite(stepPin, LOW);                 
            digitalWrite(stepPin2, LOW);                   
            delayMicroseconds(2000); 
          }          
          break;

        case 'd':
          Serial.println("right");
          digitalWrite(dirPin, LOW);                      
          digitalWrite(dirPin2, HIGH);                     

          for (int i = 0; i < stepsPerRevolution; i++) {  //PROBABLY NEED TO CHANGE THIS PART BECAUSE IT CANNOT TURN LEFT WITH ONE REVOLUTION
            digitalWrite(stepPin, HIGH);                  
            digitalWrite(stepPin2, LOW);                   
            delayMicroseconds(2000);                     
            digitalWrite(stepPin, LOW);                 
            digitalWrite(stepPin2, LOW);                   
            delayMicroseconds(2000);
          }            break;

        default:
          Serial.println("Invalid command");
          break;
        }
    }

  }

  //delay(1000);
}


