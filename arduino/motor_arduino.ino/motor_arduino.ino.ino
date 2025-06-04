const uint16_t speed1_default = 2048;
const uint16_t speed2_default = 2048;
String inputString = "";

void setup() {
  Serial.begin(115200);
  analogWriteResolution(12);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processInput();
      inputString = "";
    } else {
      inputString += c;
    }
  }
}

void processInput() {
    int firstComma = inputString.indexOf(',');
    int secondComma = inputString.indexOf(',', firstComma + 1);
    
    if (firstComma != -1 && secondComma != -1) {
        uint16_t speed1 = inputString.substring(0, firstComma).toInt();
        uint16_t speed2 = inputString.substring(firstComma + 1, secondComma).toInt();
        uint16_t receivedChecksum = inputString.substring(secondComma + 1).toInt();
        uint16_t calculatedChecksum = (speed1 + speed2) & 0xFFFF;
        
        if (receivedChecksum == calculatedChecksum) {
            analogWrite(A21, speed1);
            analogWrite(A22, speed2);
            Serial.println("OK");
        } else {
            Serial.println("CHECKSUM_ERROR");
        }
    } else {
        Serial.println("FORMAT_ERROR");
    }
}
