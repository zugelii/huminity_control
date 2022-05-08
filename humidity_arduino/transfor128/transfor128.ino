/*
  String length()

  Examples of how to use length() in a String.
  Open the Serial Monitor and start sending characters to see the results.

  created 1 Aug 2010
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/StringLengthTrim
*/

String txtMsg = "";                         // a string for incoming text
unsigned int lastStringLength = txtMsg.length();     // previous length of the String

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // send an intro:
  Serial.println("read from arduino slave message");
  Serial.println();

   // Open serial communications and wait for port to open:
  Serial1.begin(115200);
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  // add any incoming characters to the String:
  //Serial1.println("test");
  //delay(1000);
  
  while (Serial1.available() > 0) {
    char inChar = Serial1.read();
    Serial.print(inChar);
  }
  
}
