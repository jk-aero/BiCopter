// THIS PROGRAM recives  signal from the FS-iA10B iBus connected to Teensy Serial1 (pin 0)
// test code
#include <IBusBM.h>

IBusBM ibus;

void setup() {
  Serial.begin(115200);

  // FS-iA10B iBus connected to Teensy Serial1 (pin 0)
  ibus.begin(Serial1);

  Serial.println("Reading FlySky iBUS data...");
}

void loop() {

  // Read 10 channels (FS-iA10B has 10 PWM but iBus has 14 channels internally)
  for (int ch = 0; ch < 10; ch++) {
    int value = ibus.readChannel(ch);   // returns 1000–2000 range

    Serial.print("CH");
    Serial.print(ch + 1);
    Serial.print(": ");
    Serial.print(value);
    Serial.print("\t");
  }

  Serial.println();
  delay(50);
}
