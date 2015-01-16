/*
 */
boolean PAUSED = false;

const int verticalPotPin = A0;
const int horizontalPotPin = A1;
const int gripperPin = 4;
const int verticalForwardPin = 6;
const int verticalBackwardPin = 7;
const int horizontalForwardPin = 8;
const int horizontalBackwardPin = 9;

const int ledPin = 13;
int ledState = HIGH;

// Full commands should be of the form:
// "READ"
// "WRITE:<HORIZONTALVAL>,<VERTICALVAL>,<GRIPPERVAL>"
String commands[] = {
  "READ",
  "WRITE:"
  "STOP",
  "START"
  };
  
bool Contains(String s, String search) {
    int maxSearchLen = s.length() - search.length();

    for (int i = 0; i <= maxSearchLen; i++) {
        if (s.substring(i,search.length()) == search) 
          return true;
    }
    return false;
} 

const int epsilon = 5; // allowable delta for stopping LA's

int verticalPos = 200;
int horizontalPos = 200;
int gripRot = 50; // [-90,90]
int gripRotScale = 1; // convert to pulsewidth delta

const int centerAngle = 1500; // us pulsewidth

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
  pinMode(gripperPin, OUTPUT);
  digitalWrite(gripperPin,LOW);
  
  pinMode(verticalForwardPin, OUTPUT);
  pinMode(verticalBackwardPin, OUTPUT);
  digitalWrite(verticalForwardPin,LOW);
  digitalWrite(verticalBackwardPin,LOW);
  
  pinMode(horizontalForwardPin, OUTPUT);
  pinMode(horizontalBackwardPin, OUTPUT);
  digitalWrite(horizontalForwardPin,LOW);
  digitalWrite(horizontalBackwardPin,LOW);
  
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);
  
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

void loop() {
  // read the positions of the linear actuators
  int verticalVal = analogRead(verticalPotPin);  // potentiometer for LA
  int horizontalVal = analogRead(horizontalPotPin);  // potentiometer for LA
  // can use map(analog0/1, 0, 1023, 0, 255) if needed
  
  if (PAUSED == false) {    
    // output gripper pulse
    digitalWrite(gripperPin,HIGH);
    delayMicroseconds(centerAngle + gripRot*gripRotScale);
    digitalWrite(gripperPin,LOW);
    
    // move vertical LA
    if (abs(verticalPos-verticalVal) > epsilon) {
      if (verticalPos > verticalVal) {
        digitalWrite(verticalBackwardPin,LOW);
        digitalWrite(verticalForwardPin,HIGH);
      }
      else {
        digitalWrite(verticalForwardPin,LOW);
        digitalWrite(verticalBackwardPin,HIGH);
      }
    }
    else {
      digitalWrite(verticalForwardPin,LOW);
      digitalWrite(verticalBackwardPin,LOW);
    }
      
    // move horizontal LA
    if (abs(horizontalPos-horizontalVal) > epsilon) {
      if (horizontalPos > horizontalVal) {
        digitalWrite(horizontalBackwardPin,LOW);
        digitalWrite(horizontalForwardPin,HIGH);
      }
      else {
        digitalWrite(horizontalForwardPin,LOW);
        digitalWrite(horizontalBackwardPin,HIGH);
      }
    }
    else {
      digitalWrite(horizontalForwardPin,LOW);
      digitalWrite(horizontalBackwardPin,LOW);
    }
  } // end if(!PAUSED)
  
  // print the string when a newline arrives:
  if (stringComplete) {
    // check inputString here against commands
    //Serial.print(inputString);
    if (Contains(inputString, "READ")) {
      // "READ"
      Serial.print(verticalVal);
      Serial.print(",");
      Serial.print(horizontalVal);
      Serial.print("\n");
      
      if (ledState == HIGH)
       ledState = LOW;
      else
       ledState = HIGH;
    }
    if (Contains(inputString, "WRITE:")) {
      // "WRITE:<VERTICALPOS>,<HORIZONTALPOS>,<GRIPPERROT>"
      String valsString = inputString.substring(6);
      // parse the values from the string
      int firstComma = 0;
      //Serial.print(valsString);
      firstComma = valsString.indexOf(',');
      verticalPos = valsString.substring(0,firstComma).toInt();      
      valsString = valsString.substring(firstComma+1);
      
      firstComma = valsString.indexOf(',');
      horizontalPos = valsString.substring(0,firstComma).toInt();  
      valsString = valsString.substring(firstComma+1);
      
      gripRot = valsString.toInt();
      
      Serial.print("Got:");
      Serial.print(verticalPos);
      Serial.print(",");
      Serial.print(horizontalPos);
      Serial.print(",");
      Serial.print(gripRot);
      Serial.print("\n");
      if (ledState == HIGH)
       ledState = LOW;
      else
       ledState = HIGH;
    }
    if (inputString.substring(0,commands[2].length()).equals(commands[2])) {
      // "STOP"
      PAUSED = true;
    }
    if (inputString.substring(0,commands[3].length()).equals(commands[3])) {
      // "START"
      PAUSED = false;
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
    digitalWrite(ledPin,ledState);
  }
  delay(10);  // delay 10 ms
  
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } 
  }
}


