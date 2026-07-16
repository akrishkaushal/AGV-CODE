#include <Arduino.h>

// --- Pin Allocations ---
// Left Side (Driver Outputs 1 & 2)
const uint8_t LEFT_A = 2;    
const uint8_t LEFT_B = 4;    
const uint8_t LEFT_PWM = 6;  
const uint8_t LEFT_DIR1 = 7; 
const uint8_t LEFT_DIR2 = 8; 

// Right Side (Driver Outputs 3 & 4)
const uint8_t RIGHT_A = 3;    
const uint8_t RIGHT_B = 5;    
const uint8_t RIGHT_PWM = 9;  
const uint8_t RIGHT_DIR1 = 10;
const uint8_t RIGHT_DIR2 = 11;

// --- Volatile variables for ISR tracking ---
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

unsigned long previousTime = 0;
const float LOOP_TIME_SEC = 0.05; // Fast 50ms (20Hz) update rate for the Pi

// --- Interrupt Service Routines ---
void leftEncoderISR() {
    if (digitalRead(LEFT_A) == digitalRead(LEFT_B)) { leftEncoderCount--; } 
    else { leftEncoderCount++; }
}

void rightEncoderISR() {
    // Mirror-inverted logic so forward movement gives positive counts on both sides
    if (digitalRead(RIGHT_A) == digitalRead(RIGHT_B)) { rightEncoderCount++; } 
    else { rightEncoderCount--; }
}

void setup() {
    Serial.begin(115200);

    // Initialize Encoder Pins
    pinMode(LEFT_A, INPUT_PULLUP);
    pinMode(LEFT_B, INPUT_PULLUP);
    pinMode(RIGHT_A, INPUT_PULLUP);
    pinMode(RIGHT_B, INPUT_PULLUP);

    // Initialize Motor Pins
    pinMode(LEFT_PWM, OUTPUT);
    pinMode(LEFT_DIR1, OUTPUT);
    pinMode(LEFT_DIR2, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
    pinMode(RIGHT_DIR1, OUTPUT);
    pinMode(RIGHT_DIR2, OUTPUT);

    // Attach Hardware Interrupts
    attachInterrupt(digitalPinToInterrupt(LEFT_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightEncoderISR, CHANGE);

    previousTime = millis();
}

void loop() {
    // 1. Receive direct PWM driving instructions from the Raspberry Pi
    // Expected incoming format over serial: "leftPWM,rightPWM\n" (values between -255 and 255)
    if (Serial.available() > 0) {
        int cmdLeftPWM = Serial.parseInt();
        int cmdRightPWM = Serial.parseInt();
        
        // Flush remaining buffer characters
        while(Serial.available() > 0) Serial.read();

        // Control Left Motor Direction & Speed
        if (cmdLeftPWM >= 0) {
            digitalWrite(LEFT_DIR1, HIGH);
            digitalWrite(LEFT_DIR2, LOW);
        } else {
            digitalWrite(LEFT_DIR1, LOW);
            digitalWrite(LEFT_DIR2, HIGH);
        }
        analogWrite(LEFT_PWM, constrain(abs(cmdLeftPWM), 0, 255));

        // Control Right Motor Direction & Speed
        if (cmdRightPWM >= 0) {
            digitalWrite(RIGHT_DIR1, HIGH);
            digitalWrite(RIGHT_DIR2, LOW);
        } else {
            digitalWrite(RIGHT_DIR1, LOW);
            digitalWrite(RIGHT_DIR2, HIGH);
        }
        analogWrite(RIGHT_PWM, constrain(abs(cmdRightPWM), 0, 255));
    }

    // 2. Continuous State Feedback Loop (Send counts to Pi)
    unsigned long currentTime = millis();
    if ((currentTime - previousTime) >= (LOOP_TIME_SEC * 1000.0)) {
        previousTime = currentTime;

        long currentLeft;
        long currentRight;

        // Atomic copy of variables
        noInterrupts();
        currentLeft = leftEncoderCount;
        currentRight = rightEncoderCount;
        interrupts();

        // Print raw encoder counts to Pi: "left_count,right_count\n"
        Serial.print(currentLeft);
        Serial.print(",");
        Serial.println(currentRight);
    }
}