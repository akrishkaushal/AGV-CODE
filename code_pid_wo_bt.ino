#include <Arduino.h>

/**
 * @class WheelController
 * @brief Handles individual wheel encoder tracking, PID velocity control, and motor actuation.
 */
class WheelController {
private:
    uint8_t pinA, pinB, pinPwm, pinDir1, pinDir2;
    float cpr, circumference;
    
    // PID Parameters
    float kp, ki, kd;
    float eIntegral = 0.0, ePrev = 0.0;
    
    // Tracking variables
    volatile long count = 0;
    long prevCount = 0;
    float currentVelocity = 0.0;
    float targetVelocity = 0.0;
    bool reverseDirection = false;

public:
    WheelController(uint8_t pA, uint8_t pB, uint8_t pPwm, uint8_t pDir1, uint8_t pDir2, 
                    float countsPerRev, float wheelDiam, bool reverse = false)
        : pinA(pA), pinB(pB), pinPwm(pPwm), pinDir1(pDir1), pinDir2(pDir2), 
          cpr(countsPerRev), reverseDirection(reverse) {
        circumference = PI * wheelDiam;
    }

    void init() {
        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);
        pinMode(pinPwm, OUTPUT);
        pinMode(pinDir1, OUTPUT);
        pinMode(pinDir2, OUTPUT);
    }

    // Must be called from the hardware ISR on CHANGE event for pinA
    void handleInterrupt() {
        if (digitalRead(pinA) == digitalRead(pinB)) {
            reverseDirection ? count++ : count--;
        } else {
            reverseDirection ? count-- : count++;
        }
    }

    void setPIDGains(float p, float i, float d) {
        kp = p; ki = i; kd = d;
    }

    void setTargetVelocity(float target) {
        targetVelocity = target;
    }

    float getVelocity() const { return currentVelocity; }
    long getCount() { return count; }

    /**
     * @brief Computes measured velocity and updates the motor output via PID loop.
     * @param dt Time step in seconds
     */
    void update(float dt) {
        // Atomic read of volatile counter
        noInterrupts();
        long currentCount = count;
        interrupts();

        long deltaCount = currentCount - prevCount;
        prevCount = currentCount;

        // Calculate linear velocity (m/s)
        currentVelocity = (deltaCount / cpr) * circumference / dt;

        // PID Math
        float error = targetVelocity - currentVelocity;
        eIntegral += error * dt;
        
        // Anti-windup protection (clamp integral term)
        eIntegral = constrain(eIntegral, -400.0, 400.0); 
        
        float errorDeriv = (error - ePrev) / dt;
        float pidOutput = (kp * error) + (ki * eIntegral) + (kd * errorDeriv);
        ePrev = error;

        // Actuate Motor Drivers
        int pwmValue = constrain(abs(pidOutput), 0, 255);
        if (pidOutput >= 0) {
            digitalWrite(pinDir1, HIGH);
            digitalWrite(pinDir2, LOW);
        } else {
            digitalWrite(pinDir1, LOW);
            digitalWrite(pinDir2, HIGH);
        }
        analogWrite(pinPwm, pwmValue);
    }
};

/**
 * @class DifferentialDriveKinematics
 * @brief Manages forward/inverse kinematics and track odometry coordinates (X, Y, Theta).
 */
class DifferentialDriveKinematics {
private:
    float wheelBase;
    float x = 0.0, y = 0.0, theta = 0.0; // Robot pose

public:
    DifferentialDriveKinematics(float baseWidth) : wheelBase(baseWidth) {}

    /**
     * @brief Converts overall target v and w to individual wheel velocities
     */
    void inverseKinematics(float targetV, float targetOmega, float &outTargetVL, float &outTargetVR) {
        outTargetVL = targetV - (targetOmega * wheelBase / 2.0);
        outTargetVR = targetV + (targetOmega * wheelBase / 2.0);
    }

    /**
     * @brief Updates robot odometry pose tracking based on actual wheel readings
     */
    void forwardKinematics(float vL, float vR, float dt) {
        float v = (vL + vR) / 2.0;
        float omega = (vR - vL) / wheelBase;
        float deltaTheta = omega * dt;

        // Runge-Kutta 2nd Order Integration
        x += v * cos(theta + (deltaTheta / 2.0)) * dt;
        y += v * sin(theta + (deltaTheta / 2.0)) * dt;
        theta += deltaTheta;

        // Angle normalization [-PI, PI]
        if (theta > PI) theta -= 2.0 * PI;
        else if (theta < -PI) theta += 2.0 * PI;
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getTheta() const { return theta; }
};

// --- Hardware and Configuration Definitions ---
const float GEAR_RATIO_CPR = 4680.0; // 2X decoding calculation 
const float WHEEL_DIAMETER = 0.1;    // meters
const float WHEEL_BASE = 0.24;       // meters

// Create Instances
WheelController leftWheel(2, 4, 6, 7, 8, GEAR_RATIO_CPR, WHEEL_DIAMETER, false);
WheelController rightWheel(3, 5, 9, 10, 11, GEAR_RATIO_CPR, WHEEL_DIAMETER, true); // Swapped direction matrix
DifferentialDriveKinematics robot(WHEEL_BASE);

// Global ISR wrappers pointing to class instances
void leftEncoderISR()  { leftWheel.handleInterrupt(); }
void rightEncoderISR() { rightWheel.handleInterrupt(); }

unsigned long previousTime = 0;
const float LOOP_TIME_SEC = 0.2; // 200ms execution step

void setup() {
    Serial.begin(115200);

    leftWheel.init();
    rightWheel.init();

    // Tune your PID gains here
    leftWheel.setPIDGains(180.0, 60.0, 3.0);
    rightWheel.setPIDGains(180.0, 60.0, 3.0);

    attachInterrupt(digitalPinToInterrupt(2), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), rightEncoderISR, CHANGE);

    previousTime = millis();
}

void loop() {
    // === TEMPORARY TUNING MODE ===
    // Force a constant target velocity profile (0.2 m/s straight line)
    // This removes the need for Raspberry Pi serial inputs during tuning
    float targetVL = 1;
    float targetVR = 1;
    
    leftWheel.setTargetVelocity(targetVL);
    rightWheel.setTargetVelocity(targetVR);

    // Continuous control and state execution loop
    unsigned long currentTime = millis();
    float dt = (currentTime - previousTime) / 1000.0;

    if (dt >= LOOP_TIME_SEC) {
        previousTime = currentTime;

        // Process loops
        leftWheel.update(dt);
        rightWheel.update(dt);
        robot.forwardKinematics(leftWheel.getVelocity(), rightWheel.getVelocity(), dt);

        // === CLEAN PLOTTER OUTPUT ===
        // Stream out ONLY Target vs. Actual for a single wheel (e.g., Left Wheel)
        // Labels tell the Arduino Serial Plotter exactly how to render the legend
        Serial.print("Target_V:");
        Serial.print(targetVL, 3);
        Serial.print(",");
        Serial.print("Actual_V:");
        Serial.println(leftWheel.getVelocity(), 3); // Ends with a single newline
    }
}