#include <Bounce2.h>

#define SWITCH_PIN 14

Bounce debouncer = Bounce();
bool control_run = false;

void setup() {
    Serial.begin(115200);

    pinMode(SWITCH_PIN, INPUT_PULLUP);

    debouncer.attach(SWITCH_PIN);
    debouncer.interval(2); // debounce time in ms
}

void loop() {
    debouncer.update();

    // detect state change
    if (debouncer.changed()) {
        // INPUT_PULLUP: LOW = pressed
        control_run = (debouncer.read() == LOW);
    }

    // where the control code will be going
    if (control_run){
        Serial.println("Pretend running");
    }
    else if (!control_run){
        Serial.println("Not running");
    }

    delay(10);
}