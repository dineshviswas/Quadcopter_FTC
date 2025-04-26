#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Servo.h>
#include <FreeRTOS.h>
#include <task.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- WIFI DEFS ---
#ifndef STASSID
#define STASSID                                     "Mi 11X"
#define STAPSK                                      "12345678"
#define WIFI_PORT                                   80
#endif

// --- Constants and Globals ---
#define BNO08X_RESET -1
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2500;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

Servo motor[4];
int motorPins[4] = {10, 11, 12, 13};  // 10 -- FR(CW); 11 -- FL(CCW); 12 -- RL(CW); 13 -- RR(CCW) 

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

float Kproll = 2.8e-5;
float Kppitch = 1.1e-4;

volatile bool controlEnabled = false;

unsigned long startTime = 0;

WiFiUDP udp;
WiFiServer server(WIFI_PORT);
WiFiClient client;

// --- Command enums ---
enum control {
  START = 0,          // msg length = 0B           - Enable control loop
  KILL,               // msg length = 0B           - Emergency stop
  HOVER,              // msg length = 0B           - Level hover mode (pitch = roll = 0)
  SET_KP_ROLL,        // msg length = 4B (float)   - Set Kp for roll
  SET_KP_PITCH,       // msg length = 4B (float)   - Set Kp for pitch
  CMD_SIZE            //                           - Total command count
};

TaskHandle_t controlTaskHandle = NULL;

// --- Function Prototypes ---
// void initMotors();
// void initIMU();
// void updateAttitude();
// void mixMotors();
// void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false);
// void setReports(sh2_SensorId_t reportType, long report_interval);

// --- Setup ---
void setup() {
  Serial.begin(115200);
  delay(10000);
  initWiFi();
  delay(3000);
  initIMU();
  startTime = millis();

  xTaskCreate(vSensorTask, 
            "SensorTask", 
            1024, 
            NULL, 
            2,
            NULL);

  xTaskCreate(vNetTask,
            "NetTask",
            1024, 
            NULL, 
            1, 
            NULL);

  xTaskCreate(vControlTask, 
            "ControlTask", 
            1024, 
            NULL, 
            1, 
            &controlTaskHandle);  // Pass the handle
}

// --- Main Loop ---
void loop() {
}

// --- Motor Initialization ---
void initMotors() {
  for (int i = 0; i < 4; i++) {
    motor[i].attach(motorPins[i]);
    motor[i].writeMicroseconds(1000);
  }
}

// --- WiFi Initialization ---
void initWiFi()
{
  WiFi.begin(STASSID, STAPSK);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  udp.begin(WIFI_PORT);
}


// --- IMU Initialization ---
void initIMU() {
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x");
    while (1);
  }
  Serial.println("BNO08x Found!");
  setReports(reportType, reportIntervalUs);
  delay(100);
}

// --- Set IMU Report Type ---
void setReports(sh2_SensorId_t reportType, long report_interval) {
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable report");
  }
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

// --- Attitude Update ---
void updateAttitude() {
  if (bno08x.wasReset()) {
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GYRO_INTEGRATED_RV) {
      quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
      // Serial.print("YPR: ");
      // Serial.print(ypr.yaw); Serial.print("\t");
      // Serial.print(ypr.pitch); Serial.print("\t");
      // Serial.println(ypr.roll);
    }
  }
}

// --- Motor Mixing with P Controller for Roll & Pitch ---
void mixMotors() {
  float desiredRoll = 0.0;
  float desiredPitch = 0.0;

  float rollError = desiredRoll - ypr.roll;
  float pitchError = desiredPitch - ypr.pitch;

  float rollCorrection = Kproll * rollError;
  float pitchCorrection = Kppitch * pitchError;

  int hoverPWM = 1550;

  int motorPWMs[4];

  // Motor mixing for quad in 'X' config
  motorPWMs[0] = hoverPWM + pitchCorrection - rollCorrection;  
  motorPWMs[1] = hoverPWM + pitchCorrection + rollCorrection;  
  motorPWMs[2] = hoverPWM - pitchCorrection + rollCorrection;  
  motorPWMs[3] = hoverPWM - pitchCorrection - rollCorrection;

  // Constrain and write to motors
  for (int i = 0; i < 4; i++) {
    motorPWMs[i] = constrain(motorPWMs[i], 1000, 2000);
    motor[i].writeMicroseconds(motorPWMs[i]);
  }
}


// --- Quaternion to Euler Conversion ---
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void vSensorTask(void* pvParameters) {
  for (;;) {
    updateAttitude();
    vTaskDelay(1);  // Slight delay to avoid tight loop
  }
}

void vControlTask(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(2.5); // 400 Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    if (!controlEnabled) {
      for (int i = 0; i < 4; i++) {
        motor[i].writeMicroseconds(1000); // Kill motors
      }
      vTaskSuspend(NULL); // Stop this task
    }
    else{
      mixMotors();
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void vNetTask(void* pvParameters) {
  char incomingPacket[256];
  for (;;) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(incomingPacket, sizeof(incomingPacket));
      if (len > 0) incomingPacket[len] = 0;

      uint8_t cmd = incomingPacket[0];  // First byte is command
      switch (cmd) {
        case START:
          initMotors();
          Serial.println("START command received.");
          break;

        case KILL:
          controlEnabled = false;
          for (int i = 0; i < 4; i++) motor[i].writeMicroseconds(1000);
          Serial.println("KILL command received.");
          break;

        case HOVER:
          controlEnabled = true;
          if(controlTaskHandle != NULL){
            vTaskResume(controlTaskHandle);
          }
          Serial.println("HOVER command received.");
          break;

        case SET_KP_ROLL:
          if (len >= 5) {
            float newKp;
            memcpy(&newKp, &incomingPacket[1], 4);
            Kproll = newKp;
            // Optional: Add a bounds check
            Serial.print("Updated Kp_roll: ");
            Serial.println(newKp, 7);
          }
          break;

        case SET_KP_PITCH:
          if (len >= 5) {
            float newKp;
            memcpy(&newKp, &incomingPacket[1], 4);
            Kppitch = newKp;
            Serial.print("Updated Kp_pitch: ");
            Serial.println(newKp, 7);
          }
          break;

        default:
          Serial.print("Unknown command: ");
          Serial.println(cmd);
          break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}