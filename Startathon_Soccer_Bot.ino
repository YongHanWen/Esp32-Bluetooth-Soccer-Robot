#include <BluetoothSerial.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"

// Motor control pins
#define ENA 15
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19
#define ENB 21
#define DEFAULT_SPEED 255  // Maximum analog value

BluetoothSerial bluetooth; // Use built-in Bluetooth

// To track buttons pressed
bool forwardActive = false;
bool backwardActive = false;
bool leftActive = false;
bool rightActive = false;
bool backwardTurnedOff = false;
int x = 1;  // x is set to 1 by default

void setupMotors() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void printBluetoothMacAddress() {
    const uint8_t* mac = esp_bt_dev_get_address();
    if (mac) {
        Serial.printf("Bluetooth MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        bluetooth.printf("Bluetooth MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        Serial.println("Failed to get Bluetooth MAC address");
    }
}

void setup() {
    Serial.begin(115200); 
    bluetooth.begin("ESP32-WheelControl"); // Set Bluetooth name
    
    // Get and print MAC address
    printBluetoothMacAddress();

    Serial.println("Bluetooth Ready. Waiting for commands...");
    setupMotors();
}

void loop() {
    if (bluetooth.available()) {
        char command = bluetooth.read();
        Serial.print("Received command: ");
        Serial.println(command);
        
        bluetooth.print("ESP32 received: ");
        bluetooth.println(command);

        switch (command) {
            case 'F': forwardActive = true; break;
            case 'f': forwardActive = false; break;
            case 'B': backwardActive = true; x = 0; backwardTurnedOff = false; break;
            case 'b': backwardActive = false; backwardTurnedOff = true; break;
            case 'L': leftActive = true; backwardTurnedOff = false; break;
            case 'l': leftActive = false; if (!backwardActive) x = 1; break;
            case 'R': rightActive = true; backwardTurnedOff = false; break;
            case 'r': rightActive = false; if (!backwardActive) x = 1; break;
            case 'S': stopMotors(); forwardActive = backwardActive = leftActive = rightActive = false; x = 1; break;
        }
    }

    if (backwardTurnedOff && !leftActive && !rightActive) {
        x = 1;
        backwardTurnedOff = false;
    }

    // Combined commands take priority
    if (forwardActive && leftActive) {
        moveForwardLeft(DEFAULT_SPEED);
    } else if (forwardActive && rightActive) {
        moveForwardRight(DEFAULT_SPEED);
    } else if (backwardActive && leftActive) {
        moveBackwardLeft(DEFAULT_SPEED);
    } else if (backwardActive && rightActive) {
        moveBackwardRight(DEFAULT_SPEED);
    } else if (forwardActive) {
        moveForward(DEFAULT_SPEED);
    } else if (backwardActive) {
        moveBackward(DEFAULT_SPEED);
    } else if (leftActive) {
        if (x == 1) rotateLeft();
        else rotateRight();
    } else if (rightActive) {
        if (x == 1) rotateRight();
        else rotateLeft();
    } else {
        stopMotors();
    }
}

void moveForward(int speed) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
}

void moveBackward(int speed) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed);
}

void rotateLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 50);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 255);
}

void rotateRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 50);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, 255);
}

void moveForwardLeft(int speed) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed / 2);  // Left motor slower (gentle turn)
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);      // Right motor at full speed (forward motion)
}

void moveForwardRight(int speed) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);      // Left motor at full speed (forward motion)
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed / 2);  // Right motor slower (gentle turn)
}

void moveBackwardLeft(int speed) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speed / 2);  // Left motor slower (gentle turn)
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed);      // Right motor at full speed (backward motion)
}

void moveBackwardRight(int speed) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speed);      // Left motor at full speed (backward motion)
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed / 2);  // Right motor slower (gentle turn)
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}