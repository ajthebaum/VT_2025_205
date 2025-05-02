// For internal pull down resistors, button state == High means that the button is not being pushed
// and button state == low means the button is being pushed.
#include <ArduinoJson.h>
#include <Preferences.h>
#include <float.h>

// Digital Pin naming, Please dont change these, they correspond to the wiring diagram and are used in multiple places in the code
#define D0 14   //LED Pin
#define D1 32   //button 1
#define D2 15   //Button 2
#define D3 33   //Button 3
#define D4 27   //Button 4
#define D5 12   //Button 5
#define D6 13   //Joystick Button 1
#define D7 37   //Joystick Button 2

unsigned long sleeptime = 10000;  // Microseconds -- What is this for

unsigned long currentJstkMillis; //Millis function specific for joystick
unsigned long lastJstkUpdate = 0; //Time at which the currentJstkMillis was >= 25ms.
const unsigned long JstkUpdateInterval = 40; //Interval time the Jstk should read values at a rate of 25ms.

// Declare our local coordinate variables on feather side
float curxpos, curypos, curzpos, curtpos;
float distance;


//Declare variables used for safety checks in joystick function
float varbuffz, deadzone;
const float caseMaxX = 20;
const float caseMinX = -310;
const float caseMaxY = 50;
const float caseMinY = -50;
const float caseMaxZ = 10;
const float caseMinZ = -190;
int safetybuff;

// Declare state variables to be used for the light
bool lightstate;

// Create the counters for all four buttons that require programming and recal capabilities
int debounceTimer = 50;
unsigned long minButtonHold = 3000; //Minimum hold time to start the "saving" funciton


int buttonStatePrevious1 = HIGH;  //Set initial button state to no click (OFF)
int button1 = HIGH;
unsigned long buttonLongPressMillis1; //Time in ms the button was pressed
bool buttonStateLongPress1 = false; 
unsigned long buttonPreviousMillis1;
unsigned long buttonPressDuration1;
unsigned long currentMillis1; //Variable to store the number of milliseconds since the Arduino has been turned on

int buttonStatePrevious2 = HIGH;  //Set initial button state to no click (OFF)
int button2 = HIGH;
unsigned long buttonLongPressMillis2; //Time in ms the button was pressed
bool buttonStateLongPress2 = false;  
unsigned long buttonPreviousMillis2;
unsigned long buttonPressDuration2;
unsigned long currentMillis2; //Variable to store the number of milliseconds since the Arduino has been turned on

int buttonStatePrevious3 = HIGH;  //Set initial button state to no click (OFF)
int button3 = HIGH;
unsigned long buttonLongPressMillis3; //Time in ms the button was pressed
bool buttonStateLongPress3 = false;  
unsigned long buttonPreviousMillis3;
unsigned long buttonPressDuration3;
unsigned long currentMillis3; //Variable to store the number of milliseconds since the Arduino has been turned on

int buttonStatePrevious4 = HIGH;  //Set initial button state to no click (OFF)
int button4 = HIGH;
unsigned long buttonLongPressMillis4; //Time in ms the button was pressed
bool buttonStateLongPress4 = false;  
unsigned long buttonPreviousMillis4;
unsigned long buttonPressDuration4;
unsigned long currentMillis4; //Variable to store the number of milliseconds since the Arduino has been turned on

// Variables for if button 5 is to be used as an extra saved position rather than the end effector LED
// Uncomment below if you intend to use the extra button function:

// int buttonStatePrevious5 = HIGH;  //Set initial button state to no click (OFF)
// int button5 = HIGH;
// unsigned long buttonLongPressMillis5; //Time in ms the button was pressed
// bool buttonStateLongPress5 = false;  
// unsigned long buttonPreviousMillis5;
// unsigned long buttonPressDuration5;
// unsigned long currentMillis5; //Variable to store the number of milliseconds since the Arduino has been turned on


void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);

  // Set Pinmodes
  // Analog Pins:
  pinMode(A0, INPUT);  // Joystick X Input
  pinMode(A1, INPUT);  // Joystick Y Input
  pinMode(A2, INPUT);  // Joystick Z Input
  pinMode(A3, INPUT);  // Switch Logic input for joystick
  pinMode(A4, INPUT);  // Switch Logic input for system enable
  // Digital Pins:
  pinMode(D0, OUTPUT);        // Status LED
  pinMode(D1, INPUT_PULLUP);  // Panel Button 1
  pinMode(D2, INPUT_PULLUP);  // Panel Button 2
  pinMode(D3, INPUT_PULLUP);  // Panel Button 3
  pinMode(D4, INPUT_PULLUP);  // Panel Button 4
  pinMode(D5, INPUT_PULLUP);  // Panel Button 5
  pinMode(D6, INPUT_PULLUP);  // Joystick Button 1
  pinMode(D7, INPUT_PULLUP);  // Joystick Button 2

  // Check for the status of the serial 1, ESP 32 has startup code it needs to run, we need to know when we get connection and can send commands
  Serial.println("Attempting to connect to serial1...");
  while(!Serial1.available()) {
    delay(100);
    Serial.print("Waiting for connection .");
    delay(300);
    Serial.print(" .");
    delay(300);
    Serial.print(" .");
    delay(300);
    Serial.println();
  }
  Serial.println("Serial1 Connection Established");
  delay(2000);

  // Move to our initial postion that we define
  curxpos = 150;
  curypos = 0;
  curzpos = 150;
  curtpos = 3.24;
  lightstate = true;
  StaticJsonDocument<200> curposdoc;
  curposdoc["T"] = 1041;
  curposdoc["x"] = curxpos;
  curposdoc["y"] = curypos;
  curposdoc["z"] = curzpos;
  curposdoc["t"] = curtpos;
  serializeJson(curposdoc, Serial1);
  Serial1.println();
  Serial.println("Moving to Initial position . . . ");
  delay(1500);

  // Grab the feedback from the robotic arm to sync up local coordinate with robot coordinate
  StaticJsonDocument<200> initdoc;
  initdoc["T"] = 105;
  serializeJson(initdoc, Serial1);
  Serial1.println();
  delay(100);
  String response = Serial1.readStringUntil('\n');
  StaticJsonDocument<200> responseDoc;
  DeserializationError error = deserializeJson(responseDoc, response);
  if (!error) {
    curxpos = responseDoc["x"];
    curypos = responseDoc["y"];
    curzpos = responseDoc["z"];
    curtpos = responseDoc["t"];
  }
  String syncinfo = String(curxpos) + " " + String(curypos) + " " + String(curzpos) + " " + String(curtpos);
  Serial.println(syncinfo);
  Serial.println("The feather and robot coordinates have been synced");
  delay(1500);
}

// Define key functions for use later in the main loop:

void ledblinker() { // Flashing LED when in programming mode
  digitalWrite(D0, HIGH);
  delay(10);
  digitalWrite(D0, LOW);
  delay(10);
}

void IsInsideCase() {
  safetybuff = 5;

  bool inside = (curxpos >= caseMinX && curxpos <= caseMaxX &&
                 curypos >= caseMinY && curypos <= caseMaxY &&
                 curzpos >= caseMinZ && curzpos <= caseMaxZ);

  if (inside) {
    // Calculate distances to each boundary
    float xdistToMin = abs(curxpos - caseMinX);
    float xdistToMax = abs(curxpos - caseMaxX);
    float ydistToMin = abs(curypos - caseMinY);
    float ydistToMax = abs(curypos - caseMaxY);
    float zdistToMin = abs(curzpos - caseMinZ);
    float zdistToMax = abs(curzpos - caseMaxZ);

    // Find minimum distance and direction
    float minDist = xdistToMin;
    char axis = 'x';
    int dir = -1;

    if (xdistToMax < minDist) { minDist = xdistToMax; axis = 'x'; dir = 1; }
    if (ydistToMin < minDist) { minDist = ydistToMin; axis = 'y'; dir = -1; }
    if (ydistToMax < minDist) { minDist = ydistToMax; axis = 'y'; dir = 1; }
    if (zdistToMin < minDist) { minDist = zdistToMin; axis = 'z'; dir = -1; }
    if (zdistToMax < minDist) { minDist = zdistToMax; axis = 'z'; dir = 1; }

    // Apply correction only to the axis with minimum distance
    switch (axis) {
      case 'x':
        curxpos += dir * (minDist + safetybuff);
        break;
      case 'y':
        curypos += dir * (minDist + safetybuff);
        break;
      case 'z':
        curzpos += dir * (minDist + safetybuff);
        break;
    }

    // Send updated position
    StaticJsonDocument<200> curposdoc;
    curposdoc["T"] = 1041;
    curposdoc["x"] = curxpos;
    curposdoc["y"] = curypos;
    curposdoc["z"] = curzpos;
    curposdoc["t"] = curtpos;
    serializeJson(curposdoc, Serial1);
    Serial1.println();
    Serial.println("Correcting case bounds (minimal axis)");
    delay(50);
  }
}

void IsOutsideBounds(){ // SAFETY CHECK: Endure the arm does not exceed its phsyical limits
  if ((sqrt(sq(curxpos) + sq(curypos)) < 100) && curzpos > 0){ //Check the lower limits, dont let the arm go over the origin
    ledblinker();
    if (curxpos > 0){
      curxpos +=5;
    }
    if (curxpos < 0){
      curxpos -=5;
    }
    if (curypos > 0){
      curypos +=5;
    }
    if (curypos < 0){
      curypos -=5;
    }
    StaticJsonDocument<200> curposdoc;
    curposdoc["T"] = 1041;
    curposdoc["x"] = curxpos;
    curposdoc["y"] = curypos;
    curposdoc["z"] = curzpos;
    curposdoc["t"] = curtpos;
    serializeJson(curposdoc, Serial1);
    Serial1.println();
    Serial.println("Above the origin");
    delay(50);
  }
  if (distance > 498.99) { // Check the upper limits, dont let the arm go past its physical max reach
    ledblinker();
    delay(500);
    if (curxpos > 0){
      curxpos -= 5;
    }
    if (curxpos < 0){
      curxpos += 5;
    }
    if (curypos > 0){
      curypos -= 5;
    }
    if (curypos < 0){
      curypos += 5;
    }
    if (curzpos > 0){
      curzpos -= 2;
    }
    if (curzpos < 0){
      curzpos += 2;
    }

    StaticJsonDocument<200> curposdoc;
    curposdoc["T"] = 1041;
    curposdoc["x"] = curxpos;
    curposdoc["y"] = curypos;
    curposdoc["z"] = curzpos;
    curposdoc["t"] = curtpos;
    serializeJson(curposdoc, Serial1);
    Serial1.println();
    Serial.println("Exceeding the max distance bounds");
    delay(50);
  }
  else{
    return;
  }
}
void IsAboveHand(){ // SAFETY CHECK: Ensure the arm does not move above the persons hand
  const float theta1 = -150;  // Lower bound of restricted arc (in degrees)
  const float theta2 = 150; // Upper bound of restricted arc (in degrees)
  float theta_cur = atan2(curypos, curxpos) * 180.0 / PI;
  // Ensure arm does not enter the restricted arc
  if (theta_cur <= theta1 || theta_cur >= theta2) {
    // Move the arm to a safe angle
    if (theta_cur < 0) {
      theta_cur = theta1 + 6;  // Move slightly below restricted arc
    } else {
      theta_cur = theta2 - 6;  // Move slightly above restricted arc
    }
    // Convert back to Cartesian coordinates while maintaining current radius
    float radius = sqrt(sq(curxpos) + sq(curypos));
    curxpos = (radius + 1) * cos(theta_cur * PI / 180.0);
    curypos = (radius + 1) * sin(theta_cur * PI / 180.0);
    Serial.println("Above the Hand");
    StaticJsonDocument<200> curposdoc;
    curposdoc["T"] = 1041;
    curposdoc["x"] = curxpos;
    curposdoc["y"] = curypos;
    curposdoc["z"] = curzpos;
    curposdoc["t"] = curtpos;
    serializeJson(curposdoc, Serial1);
    Serial1.println();
    serializeJson(curposdoc, Serial);
    Serial.println();
    delay(50);
  }
  else{
    return;
  }
}

void jstkctrl() { // Logic for changing the local coordinates to be sent to the robotic arm

  if (analogRead(A3) < 1000){
    return; // if the joystick enable switch is off, do nothing, disable all control and commands sent
  }

  if(analogRead(A3) > 1000){ // Joystick Enable Switch on
    currentJstkMillis = millis(); //Millis function specific for joystick
    distance = sqrt(sq(curxpos) + sq(curypos) + sq(curzpos));

    IsOutsideBounds(); // Safety Check: Make sure the arm is not above the origin or outside its physical reach
    readButtonState5();
    if (distance <= 500) {
      IsAboveHand(); // Safety Check: Make sure the end effector is not directly above the users hand
      IsInsideCase(); // Safety Check: Do not let the arm hit the case. Move it away if it gets too close 
      if(currentJstkMillis - lastJstkUpdate >= JstkUpdateInterval){ //If the incremental time between previous joystick value and current value is 25ms or over begin executing.
        lastJstkUpdate = currentJstkMillis;
        if (analogRead(A0) > 2250) {
          curxpos += (analogRead(A0) - 2250) / 200;
        }
        if (analogRead(A0) < 1750) {
          curxpos -= (1750 - analogRead(A0)) / 200;
        }
        if (analogRead(A1) > 2250) {
          curypos += (analogRead(A1) - 2250) / 200;
        }
        if (analogRead(A1) < 1750) {
          curypos -= (1750 - analogRead(A1)) / 200;
        }
        if (analogRead(A2) > 2250) {
          curzpos += (analogRead(A2) - 2250) / 200;
        }
        if (analogRead(A2) < 1750) {
          curzpos -= (1750 - analogRead(A2)) / 200;
        }
      }
      if (digitalRead(D6) == LOW && curtpos > 1.6){
        curtpos -= 0.1;
        Serial.println("Opening the end effector . . .");
      }
      if (digitalRead(D7) == LOW && curtpos < 3.24){
        curtpos += 0.1;
        Serial.println("Closing the end effector . . .");
      }
      // Create the JSON document that will be sent to the waveshare board with the info necessary for robot control
      StaticJsonDocument<200> curposdoc;
      curposdoc["T"] = 1041;
      curposdoc["x"] = curxpos;
      curposdoc["y"] = curypos;
      curposdoc["z"] = curzpos;
      curposdoc["t"] = curtpos;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      serializeJson(curposdoc, Serial);
      Serial.println();
      delay(50);
    }
  }
}

void jstkbtn() {
  if (digitalRead(D6) == LOW && curtpos > 1.6) {
    curtpos -= 0.1;
    Serial.println("Opening the end effector");
  }
  if (digitalRead(D7) == LOW && curtpos < 3.24) {
    curtpos += 0.1;
    Serial.println("Closing the end effector");
  }
  
  StaticJsonDocument<200> curposdoc;
  curposdoc["T"] = 1041;
  curposdoc["x"] = curxpos;
  curposdoc["y"] = curypos;
  curposdoc["z"] = curzpos;
  curposdoc["t"] = curtpos;
  serializeJson(curposdoc, Serial1);
  Serial1.println();
  delay(50);
}

void programreadyhome() { // Corresponds to D1
  // Start the flash memory read write
  Preferences SAVEDPOSITIONS;
  SAVEDPOSITIONS.begin("SAVEDPOSITIONS", false);

  // grab the current pos data from JSON feedback and deserialization
  StaticJsonDocument<200> initdoc;
  initdoc["T"] = 105;
  serializeJson(initdoc, Serial1);
  Serial1.println();
  String response = Serial1.readStringUntil('\n');
  delay(100);
  StaticJsonDocument<200> responseDoc;
  DeserializationError error = deserializeJson(responseDoc, response);
  if (!error) {
    curxpos = responseDoc["x"];
    curypos = responseDoc["y"];
    curzpos = responseDoc["z"];
    curtpos = responseDoc["t"];
  }

  // Update the state to Program Ready/home position
  bool running = true;
  while (running){
    if (digitalRead(D1) == LOW) { // If the button is pushed, save the position
      float homex = curxpos;
      float homey = curypos;
      float homez = curzpos;
      float homet = curtpos;

      digitalWrite(D0, HIGH);
      SAVEDPOSITIONS.putString("HomeXPosition", String(homex));
      SAVEDPOSITIONS.putString("HomeYPosition", String(homey));
      SAVEDPOSITIONS.putString("HomeZPosition", String(homez));
      SAVEDPOSITIONS.putString("HomeTPosition", String(homet));

      delay(500);
      digitalWrite(D0 ,LOW);

      SAVEDPOSITIONS.end();

      running = false;
    }
    else {
      jstkctrl();
      ledblinker();
    }
  }
}

void programdrinkfunction() { // Corresponds to D2
  // Start the flash memory read write
  Preferences SAVEDPOSITIONS;
  SAVEDPOSITIONS.begin("SAVEDPOSITIONS", false);

  // grab the current pos from the JSON feedback and deserializaiton
  StaticJsonDocument<200> initdoc;
  initdoc["T"] = 105;
  serializeJson(initdoc, Serial1);
  Serial1.println();
  delay(100);
  String response = Serial.readStringUntil('\n');
  StaticJsonDocument<200> responseDoc;
  DeserializationError error = deserializeJson(responseDoc, response);
  if (!error) {
    curxpos = responseDoc["x"];
    curypos = responseDoc["y"];
    curzpos = responseDoc["z"];
    curtpos = responseDoc["t"];
  }
  // Update the state to Programming the Drink Function
  bool running = true;
  int locationstored = 1;
  while (running) {
    if (digitalRead(D2) == LOW && locationstored == 1) { // If the button is pushed, save the drink location
      float locationdrinkx = curxpos;
      float locationdrinky = curypos;
      float locationdrinkz = curzpos;
      float locationdrinkt = curtpos;

      digitalWrite(D0, HIGH);
      SAVEDPOSITIONS.putString("DrinkXPosition", String(locationdrinkx));
      SAVEDPOSITIONS.putString("DrinkYPosition", String(locationdrinky));
      SAVEDPOSITIONS.putString("DrinkZPosition", String(locationdrinkz));
      SAVEDPOSITIONS.putString("DrinkTPosition", String(locationdrinkt));
      digitalWrite(D0 ,LOW);

      Serial.println("The drink position has been saved");
      locationstored = 2;
      delay(750);
    }
    if (digitalRead(D2) == LOW && locationstored == 2) { // If the button is pushed save the 1st safe location
      float safe1x = curxpos;
      float safe1y = curypos;
      float safe1z = curzpos;
      float safe1t = curtpos;

      digitalWrite(D0, HIGH);
      SAVEDPOSITIONS.putString("Safe1XPosition", String(safe1x));
      SAVEDPOSITIONS.putString("Safe1YPosition", String(safe1y));
      SAVEDPOSITIONS.putString("Safe1ZPosition", String(safe1z));
      SAVEDPOSITIONS.putString("Safe1TPosition", String(safe1t));
      digitalWrite(D0 ,LOW);

      Serial.println("The drink position has been saved");
      locationstored = 3;
      delay(750);
    }
    if (digitalRead(D2) == LOW && locationstored == 3) { // If the button is pushed, save the 2nd safe location
      float safe2x = curxpos;
      float safe2y = curypos;
      float safe2z = curzpos;
      float safe2t = curtpos;

      digitalWrite(D0, HIGH);
      SAVEDPOSITIONS.putString("Safe2XPosition", String(safe2x));
      SAVEDPOSITIONS.putString("Safe2YPosition", String(safe2y));
      SAVEDPOSITIONS.putString("Safe2ZPosition", String(safe2z));
      SAVEDPOSITIONS.putString("Safe2TPosition", String(safe2t));
      digitalWrite(D0 ,LOW);

      Serial.println("The drink position has been saved");
      locationstored = 4;
      delay(750);
    }
    if (digitalRead(D2) == LOW && locationstored == 4) { // If the button is pushed, save the user location
      float locationuserx = curxpos;
      float locationusery = curypos;
      float locationuserz = curzpos;
      float locationusert = curtpos;

      digitalWrite(D0, HIGH);
      SAVEDPOSITIONS.putString("UserXPosition", String(locationuserx));
      SAVEDPOSITIONS.putString("UserYPosition", String(locationusery));
      SAVEDPOSITIONS.putString("UserZPosition", String(locationuserz));
      SAVEDPOSITIONS.putString("UserTPosition", String(locationusert));
      digitalWrite(D0 ,LOW);

      Serial.println("The user position has been saved");
      SAVEDPOSITIONS.end();
      delay(750);
      running = false;
    }
    else { // Enables joystick control when the button is not being pushed
      jstkctrl();
      ledblinker();
    }
  }
}

void programstow1() { // Corresponds to D3
  // Start the flash memory read write
  bool running = true;
  Preferences SAVEDPOSITIONS;
  SAVEDPOSITIONS.begin("SAVEDPOSITIONS", false); // (FLASHFILENAME, ReadOnly = {true, false})
  // Grab the current pos from JSON feedback and deserialization
  StaticJsonDocument<200> initdoc;
  initdoc["T"] = 105;
  serializeJson(initdoc, Serial1);
  Serial1.println();
  delay(100);
  String response = Serial.readStringUntil('\n');
  StaticJsonDocument<200> responseDoc;
  DeserializationError error = deserializeJson(responseDoc, response);
  if (!error) {
    curxpos = responseDoc["x"];
    curypos = responseDoc["y"];
    curzpos = responseDoc["z"];
    curtpos = responseDoc["t"];
  }

  // While in ProgramStow, enable joystick control if no button is being pushed, and when pushed
  // record the position values and write to EEPROM to be called later
  while (running) {
    if (digitalRead(D3) == LOW){ // If the button is pushed, save the Stow1 position
      float stow1x = curxpos;
      float stow1y = curypos;
      float stow1z = curzpos;
      float stow1t = curtpos;

      digitalWrite(D0, HIGH);
      SAVEDPOSITIONS.putString("Stow1XPosition", String(stow1x));
      SAVEDPOSITIONS.putString("Stow1YPosition", String(stow1y));
      SAVEDPOSITIONS.putString("Stow1ZPosition", String(stow1z));
      SAVEDPOSITIONS.putString("Stow1TPosition", String(stow1t));

      delay(1000);
      digitalWrite(D0 ,LOW);

      SAVEDPOSITIONS.end();

      running = false;
    }
    else {
      jstkctrl();
      ledblinker();
    }
  }
}

void programstow2() { // Corresponds to D4
  // Start the flash memory read write
  Preferences SAVEDPOSITIONS;
  SAVEDPOSITIONS.begin("SAVEDPOSITIONS", false);

  // Grab the current pos from JSON feedback and deserialization
  StaticJsonDocument<200> initdoc;
  initdoc["T"] = 105;
  serializeJson(initdoc, Serial1);
  Serial1.println();
  delay(100);
  String response = Serial.readStringUntil('\n');
  StaticJsonDocument<200> responseDoc;
  DeserializationError error = deserializeJson(responseDoc, response);
  if (!error) {
    curxpos = responseDoc["x"];
    curypos = responseDoc["y"];
    curzpos = responseDoc["z"];
    curtpos = responseDoc["t"];
  }

  //Update the state to Program Stow
  bool running = true;

  // While in ProgramStow, enable joystick control if no button is being pushed, and when pushed
  // record the position values and write to EEPROM to be called later
  while (running) {
    if (digitalRead(D4) == LOW){ // If the button is pushed, save the Stow2 position
      float stow2x = curxpos;
      float stow2y = curypos;
      float stow2z = curzpos;

      digitalWrite(D0, HIGH);
      SAVEDPOSITIONS.putString("Stow2XPosition", String(stow2x));
      SAVEDPOSITIONS.putString("Stow2YPosition", String(stow2y));
      SAVEDPOSITIONS.putString("Stow2ZPosition", String(stow2z));

      delay(500);
      digitalWrite(D0 ,LOW);

      SAVEDPOSITIONS.end();

      running = false;
    }
    else {
      jstkctrl();
      ledblinker();
    }
  }
}

void readyhome() { // Corresponds to D1
  // Start the memory grab
  digitalWrite(D0, HIGH);

  Preferences SAVEDPOSITIONS;
  SAVEDPOSITIONS.begin("SAVEDPOSITIONS", true);

  String tempx = SAVEDPOSITIONS.getString("HomeXPosition");
  String tempy = SAVEDPOSITIONS.getString("HomeYPosition");
  String tempz = SAVEDPOSITIONS.getString("HomeZPosition");
  String tempt = SAVEDPOSITIONS.getString("HomeTPosition");

  float desxpos = tempx.toFloat();
  float desypos = tempy.toFloat();
  float deszpos = tempz.toFloat();
  float destpos = tempt.toFloat();

  // Local variables for orthogonal vector calculation
  float orthox, orthoy, orthoz, mpointx, mpointy, mpointz, p3x, p3y, p3z;
  
  // Find the midpoint between the current position and the destination position
  // We're going to make an isosceles triangle using the midpoint and orthogonal vector, so the robot doesnt try to pass through itself when going to the ready position
  mpointx = (curxpos+desxpos)/2;
  mpointy = (curypos+desypos)/2;
  mpointz = (curzpos+deszpos)/2;

  // Create the orthogonal vector
  orthox = -1* (desypos-curypos)/4;
  orthoy = (desxpos-curxpos)/4;
  orthoz = (deszpos-curzpos)/4;

  // Theres two options for this third point (technically infinitely many) but we know where z has to be, so theres only two points and we want the one furthest
  // from the origin so the arms path bulges out rather than in. Calculate those two points:
  float c1x = mpointx + orthox;
  float c1y = mpointy + orthoy;
  float c2x = mpointx - orthoy;
  float c2y = mpointy - orthoy;
  float dist1 = sqrt(sq(c1x)+sq(c1y));
  float dist2 = sqrt(sq(c2x)+sq(c2y));

  // Compare the two points and choose which one you want
  if (dist1 > dist2) {
    p3x = c1x;
    p3y = c1y;
    p3z = mpointz + orthoz;
  } else {
    p3z = c2x;
    p3y = c2y;
    p3z = mpointz + orthoz;
  }

  // Go to the third position we just calculated
  StaticJsonDocument<200> curposdoc;
  curposdoc["T"] = 104;
  curposdoc["x"] = p3x;
  curposdoc["y"] = p3y;
  curposdoc["z"] = p3z;
  curposdoc["t"] = curtpos;
  curposdoc["spd"] = 0.2;
  serializeJson(curposdoc, Serial1);
  Serial1.println();
  delay(1000);
  // Now go to the destination position
  StaticJsonDocument<200> curposdoc2;
  curposdoc2["T"] = 104;
  curposdoc2["x"] = desxpos;
  curposdoc2["y"] = desypos;
  curposdoc2["z"] = deszpos;
  curposdoc2["t"] = destpos;
  curposdoc2["spd"] = 0.2;
  serializeJson(curposdoc2, Serial1);
  Serial1.println();
  delay(100);
  
  //Update the coordinates
  curxpos = desxpos;
  curypos = desypos;
  curzpos = deszpos;

  SAVEDPOSITIONS.end();

  digitalWrite(D0, LOW);
}

void drinkfunction() { // Corresponds to D2
 
  digitalWrite(D0, HIGH);
  // Start the memory grab
  Preferences SAVEDPOSITIONS;
  SAVEDPOSITIONS.begin("SAVEDPOSITIONS", true);

  String tempdrinkx = SAVEDPOSITIONS.getString("DrinkXPosition");
  String tempdrinky = SAVEDPOSITIONS.getString("DrinkYPosition");
  String tempdrinkz = SAVEDPOSITIONS.getString("DrinkZPosition");
  String tempuserx = SAVEDPOSITIONS.getString("UserXPosition");
  String tempusery = SAVEDPOSITIONS.getString("UserYPosition");
  String tempuserz = SAVEDPOSITIONS.getString("UserZPosition");
  String tempsafe1x = SAVEDPOSITIONS.getString("Safe1XPosition");
  String tempsafe1y = SAVEDPOSITIONS.getString("Safe1YPosition");
  String tempsafe1z = SAVEDPOSITIONS.getString("Safe1ZPosition");
  String tempsafe2x = SAVEDPOSITIONS.getString("Safe2XPosition");
  String tempsafe2y = SAVEDPOSITIONS.getString("Safe2YPosition");
  String tempsafe2z = SAVEDPOSITIONS.getString("Safe2ZPosition");

  float drinkx = tempdrinkx.toFloat();
  float drinky = tempdrinky.toFloat();
  float drinkzorig = tempdrinkz.toFloat();
  float userx = tempuserx.toFloat();
  float usery = tempusery.toFloat();
  float userz = tempuserz.toFloat();
  float safe1x = tempsafe1x.toFloat();
  float safe1y = tempsafe1y.toFloat();
  float safe1z = tempsafe1z.toFloat();
  float safe2x = tempsafe2x.toFloat();
  float safe2y = tempsafe2y.toFloat();
  float safe2z = tempsafe2z.toFloat();
  float drinkz;

  delay(1000);
  int state = 1;
  bool running = true;
  while (running) {
    if (digitalRead(D2) == HIGH && state == 1) {
      
      // Move the arm somewhere so it doesnt try to travel through itself
      StaticJsonDocument<200> curposdoc;
      curposdoc["T"] = 104;
      curposdoc["x"] = safe1x;
      curposdoc["y"] = safe1y;
      curposdoc["z"] = safe1z;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.15;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(4000);
      
      // Open the end effector and move above the drink location
      curtpos = 3.14/2.5;
      curposdoc["T"] = 104;
      curposdoc["x"] = drinkx;
      curposdoc["y"] = drinky;
      curposdoc["z"] = drinkzorig + 200;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.15;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(5000);
      // serializeJson(curposdoc, Serial);
      // Serial.println();

      // Move down to the drink location
    
      curposdoc["T"] = 104;
      curposdoc["x"] = drinkx;
      curposdoc["y"] = drinky;
      curposdoc["z"] = drinkzorig;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.15;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(5000);
      // serializeJson(curposdoc, Serial);
      // Serial.println();
      
      //Close the end effector
      curtpos = 3.24;
      curposdoc["T"] = 104;
      curposdoc["x"] = drinkx;
      curposdoc["y"] = drinky;
      curposdoc["z"] = drinkzorig;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.5;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(4000);
      // serializeJson(curposdoc, Serial);
      // Serial.println();

      // Move up out of the cupholder
      drinkz = drinkzorig + 200;

      curposdoc["T"] = 104;
      curposdoc["x"] = drinkx;
      curposdoc["y"] = drinky;
      curposdoc["z"] = drinkz;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.15;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(3000);
      // serializeJson(curposdoc, Serial);
      // Serial.println();

      // Move to a safe spot

      curposdoc["T"] = 104;
      curposdoc["x"] = safe1x;
      curposdoc["y"] = safe1y;
      curposdoc["z"] = safe1z;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.15;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(3000);
      // serializeJson(curposdoc, Serial);
      // Serial.println();

      // Move to a safe spot 2

      curposdoc["T"] = 104;
      curposdoc["x"] = safe2x;
      curposdoc["y"] = safe2y;
      curposdoc["z"] = safe2z;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.15;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(3000);
      // serializeJson(curposdoc, Serial);
      // Serial.println();

      // Move in front of the user

      curposdoc["T"] = 104;
      curposdoc["x"] = userx;
      curposdoc["y"] = usery;
      curposdoc["z"] = userz;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.15;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(3000);
      
      curxpos = userx;
      curypos = usery;
      curzpos = userz;
      state = 2;
    }
    if (digitalRead(D2) == LOW && state == 2) {
      // Go to safe spot
      StaticJsonDocument<200> curposdoc;
      curposdoc["T"] = 104;
      curposdoc["x"] = safe2x;
      curposdoc["y"] = safe2y;
      curposdoc["z"] = safe2z;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.15;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(3000);
      // serializeJson(curposdoc, Serial);
      // Serial.println();

      // Move to a safe spot

      curposdoc["T"] = 104;
      curposdoc["x"] = safe1x;
      curposdoc["y"] = safe1y;
      curposdoc["z"] = safe1z;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.15;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(3000);
      // serializeJson(curposdoc, Serial);
      // Serial.println();

      // Go above cupholder

      curposdoc["T"] = 104;
      curposdoc["x"] = drinkx;
      curposdoc["y"] = drinky;
      curposdoc["z"] = drinkz;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.15;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(3000);
      
      curxpos = drinkx;
      curypos = drinky;
      curzpos = drinkz;
      state = 3;
    }
    if (digitalRead(D2) == LOW && state == 3) {
      // Go down into the cupholder
      StaticJsonDocument<200> curposdoc;
      curposdoc["T"] = 104;
      curposdoc["x"] = drinkx;
      curposdoc["y"] = drinky;
      curposdoc["z"] = drinkzorig;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.15;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(3000);
      // serializeJson(curposdoc, Serial);
      // Serial.println();

      // Open the end effector
      curtpos = 3.14/2.5;
      curposdoc["T"] = 104;
      curposdoc["x"] = drinkx;
      curposdoc["y"] = drinky;
      curposdoc["z"] = drinkzorig;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.5;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(3000);
      // serializeJson(curposdoc, Serial);
      // Serial.println();

      // Move up out of the cupholder
      drinkz = drinkzorig + 200;

      curposdoc["T"] = 104;
      curposdoc["x"] = drinkx;
      curposdoc["y"] = drinky;
      curposdoc["z"] = drinkz;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.15;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(3000);
      // serializeJson(curposdoc, Serial);
      // Serial.println();

      // Move to ready position
      curtpos = 3.24;
      curposdoc["T"] = 104;
      curposdoc["x"] = safe1x;
      curposdoc["y"] = safe1y;
      curposdoc["z"] = safe1z;
      curposdoc["t"] = curtpos;
      curposdoc["spd"] = 0.15;
      serializeJson(curposdoc, Serial1);
      Serial1.println();
      delay(3000);
      // serializeJson(curposdoc, Serial);
      // Serial.println();
      curxpos = safe1x;
      curypos = safe1y;
      curzpos = safe1z;
      state = 4;
      running = false;
    }
    else {
      jstkctrl();
    }
  }
  digitalWrite(D0, LOW);
}

void stow1() { // Corresponds to D3
  Serial.println("stowing 1");
  digitalWrite(D0, HIGH);
  // Grab the saved positions
  Preferences SAVEDPOSITIONS;
  SAVEDPOSITIONS.begin("SAVEDPOSITIONS", false);

  // Make the current position the saved position
  String tempx = SAVEDPOSITIONS.getString("Stow1XPosition");
  String tempy = SAVEDPOSITIONS.getString("Stow1YPosition");
  String tempz = SAVEDPOSITIONS.getString("Stow1ZPosition");
  curtpos = 3.24;
  curxpos = tempx.toFloat();
  curypos = tempy.toFloat();
  curzpos = tempz.toFloat();

  delay(500);

  // Create the JSON Doc to be sent and move to the saved position
  StaticJsonDocument<200> curposdoc;
  curposdoc["T"] = 104;
  curposdoc["x"] = curxpos;
  curposdoc["y"] = curypos;
  curposdoc["z"] = curzpos;
  curposdoc["t"] = curtpos;
  curposdoc["spd"] = 0.2;
  serializeJson(curposdoc, Serial1);
  Serial1.println();
  delay(100);
  serializeJson(curposdoc, Serial);
  Serial.println();

  digitalWrite(D0, LOW);
  SAVEDPOSITIONS.end();
}

void stow2() { // Corresponds to D4
  Serial.println("stowing 2");
  digitalWrite(D0, HIGH);
  // Grab the saved positions
  Preferences SAVEDPOSITIONS;
  SAVEDPOSITIONS.begin("SAVEDPOSITIONS", false);

  // Make the current position the saved position
  String tempx = SAVEDPOSITIONS.getString("Stow2XPosition");
  String tempy = SAVEDPOSITIONS.getString("Stow2YPosition");
  String tempz = SAVEDPOSITIONS.getString("Stow2ZPosition");
  curtpos = 3.24;
  Serial.println(tempx);
  Serial.println(tempy);
  Serial.println(tempz);
  curxpos = tempx.toFloat();
  curypos = tempy.toFloat();
  curzpos = tempz.toFloat();
  delay(500);

  // Create the JSON Doc to be sent and move to the saved position
  StaticJsonDocument<200> curposdoc;
  curposdoc["T"] = 104;
  curposdoc["x"] = curxpos;
  curposdoc["y"] = curypos;
  curposdoc["z"] = curzpos;
  curposdoc["t"] = curtpos;
  curposdoc["spd"] = 0.2;
  serializeJson(curposdoc, Serial1);
  Serial1.println();
  delay(100);
  serializeJson(curposdoc, Serial);
  Serial.println();

  digitalWrite(D0, LOW);
  SAVEDPOSITIONS.end();
}

// void extrabutton() { // Corresponds to D5
  
//   digitalWrite(D0, HIGH);
//   // Grab the saved positions
//   Preferences SAVEDPOSITIONS;
//   SAVEDPOSITIONS.begin("SAVEDPOSITIONS", true);

//   // Make the current position the saved position
//   String tempx = SAVEDPOSITIONS.getString("ExtraXPosition");
//   String tempy = SAVEDPOSITIONS.getString("ExtraYPosition");
//   String tempz = SAVEDPOSITIONS.getString("ExtraZPosition");
//   curtpos = 3.24;
//   curxpos = tempx.toFloat();
//   curypos = tempy.toFloat();
//   curzpos = tempz.toFloat();

//   delay(500);

//   // Create the JSON Doc to be sent and move to the saved position
//   StaticJsonDocument<200> curposdoc;
//   curposdoc["T"] = 104;
//   curposdoc["x"] = curxpos;
//   curposdoc["y"] = curypos;
//   curposdoc["z"] = curzpos;
//   curposdoc["t"] = curtpos;
//   curposdoc["spd"] = 0.2;
//   serializeJson(curposdoc, Serial1);
//   Serial1.println();
//   delay(100);
//   serializeJson(curposdoc, Serial);
//   Serial.println();

//   digitalWrite(D0, LOW);
//   SAVEDPOSITIONS.end();
// }
void readButtonState(
  int pin,
  int &buttonStatePrevious,
  unsigned long &buttonLongPressMillis,
  bool &buttonStateLongPress,
  unsigned long &previousButtonMillis,
  unsigned long &buttonPressDuration,
  void (*onShortPress)(),
  void (*onLongPress)()
) {
  unsigned long currentMillis = millis();

  if (currentMillis - previousButtonMillis > debounceTimer) {
    int buttonState = digitalRead(pin);

    // Handle button press
    if (buttonState == LOW && buttonStatePrevious == HIGH && !buttonStateLongPress) {
      buttonLongPressMillis = currentMillis;
      buttonStatePrevious = LOW;
      Serial.print("Button on pin "); Serial.print(pin); Serial.println(" pressed");
    }

    // Measure duration
    buttonPressDuration = currentMillis - buttonLongPressMillis;

    // Handle long press
    if (buttonState == LOW && !buttonStateLongPress && buttonPressDuration >= minButtonHold) {
      buttonStateLongPress = true;
      Serial.print("Button on pin "); Serial.print(pin); Serial.println(" long pressed");
      delay(2000); // Optional, but you might want to replace this with non-blocking logic
      if (onLongPress) onLongPress();
    }

    // Handle release
    if (buttonState == HIGH && buttonStatePrevious == LOW) {
      buttonStatePrevious = HIGH;
      buttonStateLongPress = false;

      if (buttonPressDuration < minButtonHold) {
        Serial.print("Button on pin "); Serial.print(pin); Serial.println(" short press");
        if (onShortPress) onShortPress();
      }
    }

    previousButtonMillis = currentMillis;
  }
}
void onButton1ShortPress() {
  readyhome();
  Serial.println("Moving to Home");
}
void onButton1LongPress() {
  programreadyhome();
  Serial.println("Home position Stored");
}
void onButton2ShortPress() {
  drinkfunction();
  Serial.println("Moving to Home");
}
void onButton2LongPress() {
  programdrinkfunction();
  Serial.println("Home position Stored");
}
void onButton3ShortPress() {
  stow1();
  Serial.println("Moving to Home");
}
void onButton3LongPress() {
  programstow1();
  Serial.println("Home position Stored");
}
void onButton4ShortPress() {
  stow2();
  Serial.println("Moving to Home");
}
void onButton4LongPress() {
  programstow2();
  Serial.println("Home position Stored");
}

void readButtonState5(){    //Reads the fifth button state for turning on and off the end effector LED
  if(digitalRead(D5) == LOW && lightstate == true){
    StaticJsonDocument<200> lightdoc;
    lightdoc["T"] = 114;
    lightdoc["led"] = 150;
    serializeJson(lightdoc, Serial1);
    Serial1.println();
    delay(500);
    lightstate = false;
  }
  if (digitalRead(D5) == LOW && lightstate == false){
    StaticJsonDocument<200> darkdoc;
    darkdoc["T"] = 114;
    darkdoc["led"] = 0;
    serializeJson(darkdoc, Serial1);
    Serial1.println();
    delay(500);
    lightstate = true;
  }
}
// main
void readAllButtons(){
  readButtonState(D1,
    buttonStatePrevious1,
    buttonLongPressMillis1,
    buttonStateLongPress1,
    buttonPreviousMillis1,
    buttonPressDuration1,
    onButton1ShortPress,
    onButton1LongPress
  );  //read the button state 1
  readButtonState(D2,
    buttonStatePrevious2,
    buttonLongPressMillis2,
    buttonStateLongPress2,
    buttonPreviousMillis2,
    buttonPressDuration2,
    onButton2ShortPress,
    onButton2LongPress
  );  //read the button state 2
  readButtonState(D3,
    buttonStatePrevious3,
    buttonLongPressMillis3,
    buttonStateLongPress3,
    buttonPreviousMillis3,
    buttonPressDuration3,
    onButton3ShortPress,
    onButton3LongPress
  );  //read the button state 3
  readButtonState(D4,
    buttonStatePrevious4,
    buttonLongPressMillis4,
    buttonStateLongPress4,
    buttonPreviousMillis4,
    buttonPressDuration4,
    onButton4ShortPress,
    onButton4LongPress
  );  //read the button state 4
  readButtonState5();  //read the button state 5
}

bool isJoystickInDeadZone(){
  int x = analogRead(A0);
  int y = analogRead(A1);
  int z = analogRead(A2);
  int center = 1950;
  int threshold = 300;

  float distance = sqrt(sq(x - center) + sq(y - center) + sq(z - center));
  return distance < threshold;
}

void loop() {
  if (analogRead(A4) <= 1000) return; // If the system enable switch is off, end this iteration of loop

  bool joystickEnabled = analogRead(A3) > 1000; // Check the state of the joystick enable switch
  bool inDeadZone = isJoystickInDeadZone(); // Check whether we're in the deadzone or not

  if (!joystickEnabled) { //If the joystick is disabled, read the buttons only.
    readAllButtons();
  } 
  else if (joystickEnabled && inDeadZone) { //If we're in the deadzone (the joystick isnt being moved), read the jstk button and all other buttons
    jstkbtn();
    readAllButtons();
  } 
  else { // If the jyostick is out of the deadzone and enabled, go to joystick control
    jstkctrl();
  }
}
