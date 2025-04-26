#include<Adafruit_BNO08x.h>
#include<Wire.h>

Adafruit_BNO08x bno085;

void setup(){
//Step 0: Enable Dynamic calibration
// Example: Enable dynamic calibration for Accel, Gyro, Mag
// Bit 0 = Accel, Bit 1 = Gyro, Bit 2 = Mag, Bit 3 = Planar Accel, Bit 4 = On-Table
uint8_t dynamicCalFlags = 0x07;  // Accel=1, Gyro=1, Mag=1

uint8_t meCalibCommand[12] = {
0xF2,  // Command Request
0x07,  // ME Calibration command
0x01,  // Subcommand = Configure
dynamicCalFlags, 
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

// Send this over I2C to BNO08x (address typically 0x4A)
Wire.beginTransmission(0x4A);
Wire.write(meCalibCommand, sizeof(meCalibCommand));
Wire.endTransmission();

//Step 1: Enable Rotation Vector
bno085.enableReport(SH2_ROTATION_VECTOR, 5000);
//Step 2: Calibrate Magnetometer
Serial.println("Rotate the device in a figure 8");
delay(10000);
//Step 3: Calibrate accelerometer
Serial.println("Position the device in 4-6 unique orientations");
delay(10000);
//Step 4: Calibrate Gyroscope ZRO
Serial.println("Set the device down for a few seconds");
delay(6000);
Serial.println("Calibration complete. Align the device");
//Step 5: Align the device
delay(3000);
//Step 6: Send Tare now command
uint8_t cmd[] = {0xF2, 0x00, 0x03, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
Wire.beginTransmission(0x4A);
Wire.write(cmd, sizeof(cmd));
Wire.endTransmission();
delay(2000);
//Step 7: Send Persist Tare command
uint8_t cmdp[] = {0xF2, 0x00, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
Wire.beginTransmission(0x4A);
Wire.write(cmdp, sizeof(cmdp));
Wire.endTransmission();
Serial.println("Tare Done");
}

void loop(){
}