#include <NewPing.h>

#define TRIGGER_PIN  18  //d6
#define ECHO_PIN     19  //d5
// #define TRIGGER_PIN_2  12  
// #define ECHO_PIN_2     11  
#define MAX_DISTANCE 200 

NewPing sonar1(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
// NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
bool nodeflag=false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int uS = sonar1.ping(); // Send ping, get ping time in microseconds (uS).
  Serial.print(sonar1.convert_cm(uS)); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.print("cm ");

  if(sonar1.convert_cm(uS)<27 & sonar1.convert_cm(uS)>3){   // TO-DO: MEASURE THE MAX DISTANCE FROM ROVER TO WHITE LEDS
            nodeflag=false;
        }
        else{
            nodeflag=true;
        }
  //
        // Serial.Print(sonar2.ping_cm());
  Serial.println(nodeflag);
  
}
