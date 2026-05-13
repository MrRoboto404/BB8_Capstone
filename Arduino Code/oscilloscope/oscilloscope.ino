#include <stdio.h>
#define OSC_PIN 16

bool osc_state = false;

void setup() {
    Serial.begin(115200);

    pinMode(OSC_PIN, OUTPUT);
    digitalWriteFast(OSC_PIN, LOW);
}

void loop() {
    osc_state = !osc_state;

    digitalWriteFast(OSC_PIN, osc_state);

    delay(5.88);
}