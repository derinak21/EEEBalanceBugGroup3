//#include <WiFi.h>

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


JsonObject CreateJson( String CameraFeed){
  DynamicJsonDocument jBuffer(1024);
//  DynamicJsonBuffer jBuffer;
  JsonObject root=jBuffer.createNestedObject();
  root["CameraFeed"]=CameraFeed;
  root["msgSender"]="Esp32";

  return root;
}
// void WSretrieve(){
//   if(wsClient.connected()){
//     int messageSize = wsClient.parseMessage();
//     if (messageSize > 0) {
//       //Serial.println("Received a message:");
//       DynamicJsonDocument parsed(300);
//       // JsonObject parsed;
//         String recieved=wsClient.readString();
//         DeserializationError error = deserializeJson(parsed, recieved);
//         if (error){
//           return;}
//         JsonObject root = parsed.as<JsonObject>();
//         UpdateValues(root);

//         LEDControl();
//         //MotorControl();
//     }
//   wsClient.flush();
//   }
//   else{
//     delay(1000);
//     Serial.println("Starting WebSocket client");
//     wsClient.begin();
//   }

// }

void setup(){

  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  delay(1000);
  wsClient.begin();
  
}

String generateRandomCoordinates() {
  // Create a JSON document
  StaticJsonDocument<64> jsonDocument;

  // Generate random X and Y coordinates
  int randomX = random(0, 901);
  int randomY = random(0, 601);

  // Add the coordinates to the JSON document
  jsonDocument["x"] = randomX;
  jsonDocument["y"] = randomY;

  // Serialize the JSON document to a string
  String coordinatesData;
  serializeJson(jsonDocument, coordinatesData);

  // Return the coordinates data string
  return coordinatesData;
}


void loop(){
  
  //JsonObject JsonMessage=CreateJson(data);
  while (wsClient.connected())
  {
    Serial.print("Sending data ");
    String output = generateRandomCoordinates();
    unsigned long start = millis();
    wsClient.beginMessage(TYPE_TEXT);
    wsClient.print(output);
    wsClient.endMessage();
    // check if a message is available to be received
    int messageSize = wsClient.parseMessage();

    unsigned long end = millis();
    unsigned long rtt = end - start;

    // Print the RTT
    Serial.print("RTT: ");
    Serial.print(rtt);
    Serial.println(" ms");
    
    if (messageSize > 0)
    {
      Serial.println("Received a message:");
      Serial.println(wsClient.readString());
    }

    // wait 5 seconds
    delay(3000);
  }

  Serial.println("disconnected");

}










