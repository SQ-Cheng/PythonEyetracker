/*
 * sensor_integrated.ino
 *
 * Firmware for direct sensor reading on a single board.
 * Hardware connections:
 *   - I2C: for MAX30105, ICM_20948 sensors
 *   - Serial: 1000000 baud (for PC communication)
 *
 * Data output via Serial:
 *   - Protocol: "s" to start, "e" to stop, "t" to sync timestamp,
 *               "SYNC_START" to enter repeated LED sync mode,
 *               "SYNC_STOP" to exit sync mode
 *   - Binary packets: Serial.write(64-byte buffer) at 4ms intervals (~250 Hz)
 *   - 14-float data: [Red, IR, Green, accX, accY, accZ,
 *                     gyrX, gyrY, gyrZ, magX, magY, magZ, temp, timestamp]
 *   - Statistics reporting on stop
 *
 * Packet format (64 bytes, little-endian):
 *   Bytes  0-3:   Sync marker (0x55 0xAA 0x55 0xAA)
 *   Bytes  4-7:   PPG Red
 *   Bytes  8-11:  PPG IR
 *   Bytes 12-15:  PPG Green
 *   Bytes 16-19:  accX (scaled by 0.01)
 *   Bytes 20-23:  accY (scaled by 0.01)
 *   Bytes 24-27:  accZ (scaled by 0.01)
 *   Bytes 28-31:  gyrX
 *   Bytes 32-35:  gyrY
 *   Bytes 36-39:  gyrZ
 *   Bytes 40-43:  magX
 *   Bytes 44-47:  magY
 *   Bytes 48-51:  magZ
 *   Bytes 52-55:  temperature (currently 0.0)
 *   Bytes 56-59:  timestamp (millis() as float)
 *   Bytes 60-63:  padding (zeros)
 */

#include <Wire.h>
#include <MAX30105.h>
#include "ICM_20948.h"

// ===================== Packet Constants =====================
const size_t PACKET_SIZE = 64;
const uint8_t SYNC_BYTES[] = {0x55, 0xAA, 0x55, 0xAA};

// ===================== Data Buffers =====================
uint8_t txBuffer[PACKET_SIZE];
float sensorData[14];  // [Red, IR, Green, accX-Z, gyrX-Z, magX-Z, temp, timestamp]

// ===================== Sensor Objects =====================
MAX30105 particleSensor;
ICM_20948_I2C icm;

// ===================== Control Variables =====================
bool pollingActive = false;
bool syncModeActive = false;
bool syncPulseOn = false;
unsigned long lastPollTime = 0;
unsigned long startTime = 0;
unsigned long endTime = 0;
const unsigned long pollInterval = 4;  // ~250 Hz
const byte LED_OFF = 0x00;
const byte NORMAL_LED_AMPLITUDE = 0x80;
const byte SYNC_BASE_LED_AMPLITUDE = LED_OFF;
const byte SYNC_LED_AMPLITUDE = 0xFF;
const unsigned long SYNC_LED_PULSE_MS = 120;
const unsigned long SYNC_FLASH_INTERVAL_MS = 1500;  // Must be longer than the PC 500 ms pairing timeout.
const unsigned long SYNC_INITIAL_DELAY_MS = 1000;
unsigned long syncPulseEndMs = 0;
unsigned long nextSyncFlashMs = 0;
uint32_t syncPulseCounter = 0;
int sampleCounter = 0;
int nanCounter = 0;

// ===================== ICM_20948 Initialization =====================

void init_icm() {
  bool initialized = false;
  while (!initialized) {
    icm.begin(Wire, 0);
    icm.startupDefault(false);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(icm.statusString());
    if (icm.status != ICM_20948_Stat_Ok) {
      Serial.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
  }

  // SW reset to make sure the device starts in a known state
  icm.swReset();
  if (icm.status != ICM_20948_Stat_Ok) {
    Serial.print(F("Software Reset returned: "));
    Serial.println(icm.statusString());
  }
  delay(250);

  // Wake the sensor up
  icm.sleep(false);
  icm.lowPower(false);

  // Set Gyro and Accelerometer to continuous sample mode
  icm.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (icm.status != ICM_20948_Stat_Ok) {
    Serial.print(F("setSampleMode returned: "));
    Serial.println(icm.statusString());
  }

  // Set full scale ranges
  ICM_20948_fss_t myFSS;
  myFSS.a = gpm16;
  myFSS.g = dps2000;
  icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (icm.status != ICM_20948_Stat_Ok) {
    Serial.print(F("setFullScale returned: "));
    Serial.println(icm.statusString());
  }

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;
  myDLPcfg.a = acc_d246bw_n265bw;
  myDLPcfg.g = gyr_d196bw6_n229bw8;
  icm.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  if (icm.status != ICM_20948_Stat_Ok) {
    Serial.print(F("setDLPcfg returned: "));
    Serial.println(icm.statusString());
  }

  // Disable DLPF
  ICM_20948_Status_e accDLPEnableStat = icm.enableDLPF(ICM_20948_Internal_Acc, false);
  ICM_20948_Status_e gyrDLPEnableStat = icm.enableDLPF(ICM_20948_Internal_Gyr, false);
  Serial.print(F("Enable DLPF for Accelerometer returned: "));
  Serial.println(icm.statusString(accDLPEnableStat));
  Serial.print(F("Enable DLPF for Gyroscope returned: "));
  Serial.println(icm.statusString(gyrDLPEnableStat));

  // Disable FIFO
  icm.enableFIFO(false);

  // Start the magnetometer
  icm.startupMagnetometer();
  if (icm.status != ICM_20948_Stat_Ok) {
    Serial.print(F("startupMagnetometer returned: "));
    Serial.println(icm.statusString());
  }

  Serial.println();
  Serial.println(F("ICM_20948 Initialized!"));
}

// ===================== MAX30105 Initialization =====================

void setMaxLedAmplitude(byte amplitude) {
  particleSensor.setPulseAmplitudeRed(amplitude);
  particleSensor.setPulseAmplitudeIR(amplitude);
  particleSensor.setPulseAmplitudeGreen(amplitude);
}

void init_max() {
  byte ledBrightness = 0xFF;
  byte sampleAverage = 1;
  byte ledMode = 3;           // Red + IR + Green
  int sampleRate = 400;
  int pulseWidth = 411;
  int adcRange = 16384;

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST, 0x57)) {
    Serial.println("ERROR: MAX30105 not found.");
    while (1)
      ;
  }

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  setMaxLedAmplitude(LED_OFF);
  Serial.println("MAX30101 Initialized!");
}

// ===================== Sensor Initialization =====================

void initializeSensors() {
  Wire.begin();

  // Initialize MAX30101 sensor
  init_max();

  // Initialize ICM20948 sensor
  init_icm();
}

// ===================== Read All Sensors =====================

void readSensors() {
  // Read MAX30105 PPG - skip to latest sample to minimize I2C overhead
  particleSensor.check();
  while (particleSensor.available() > 1) {
    particleSensor.nextSample();  // Skip stale samples
  }
  if (particleSensor.available()) {
    sensorData[0] = particleSensor.getFIFORed();
    sensorData[1] = particleSensor.getFIFOIR();
    sensorData[2] = particleSensor.getFIFOGreen();
    particleSensor.nextSample();
  }

  // Temperature placeholder (TMP117 not connected)
  sensorData[12] = 0.0;

  // Read ICM_20948 IMU (dataReady() removed to save ~0.3ms I2C overhead)
  icm.getAGMT();
  sensorData[3] = icm.accX() * 0.01;
  sensorData[4] = icm.accY() * 0.01;
  sensorData[5] = icm.accZ() * 0.01;

  sensorData[6] = icm.gyrX();
  sensorData[7] = icm.gyrY();
  sensorData[8] = icm.gyrZ();

  sensorData[9] = icm.magX();
  sensorData[10] = icm.magY();
  sensorData[11] = icm.magZ();
}

// ===================== Send Data Packet =====================

void sendDataPacket() {
  // Clear buffer
  memset(txBuffer, 0, PACKET_SIZE);

  // Write sync marker (bytes 0-3)
  memcpy(txBuffer, SYNC_BYTES, 4);

  // Set timestamp as the last sensor data value
  sensorData[13] = (float)millis();

  // Copy sensor data (14 floats = 56 bytes) starting at byte 4
  memcpy(&txBuffer[4], sensorData, sizeof(sensorData));

  // Check for NaN in PPG Red (indicates invalid reading)
  if (isnan(sensorData[0])) {
    nanCounter++;
  }

  sampleCounter++;

  // Send packet
  Serial.write(txBuffer, PACKET_SIZE);
}

// ===================== Repeated Sync Flash Mode =====================

void stopSyncMode(bool report = true) {
  bool wasSyncing = syncModeActive || syncPulseOn;
  syncModeActive = false;
  syncPulseOn = false;
  setMaxLedAmplitude(LED_OFF);
  if (report && wasSyncing) {
    Serial.println("SYNC_MODE_STOPPED");
  }
}

void startSyncMode() {
  pollingActive = false;
  syncModeActive = true;
  syncPulseOn = false;
  sampleCounter = 0;
  nanCounter = 0;
  startTime = millis();
  lastPollTime = 0;
  syncPulseCounter = 0;
  nextSyncFlashMs = millis() + SYNC_INITIAL_DELAY_MS;
  setMaxLedAmplitude(SYNC_BASE_LED_AMPLITUDE);

  Serial.print("SYNC_MODE_STARTED ");
  Serial.println(SYNC_FLASH_INTERVAL_MS);
}

void updateSyncLed(unsigned long currentTime) {
  if (syncPulseOn && (long)(currentTime - syncPulseEndMs) >= 0) {
    syncPulseOn = false;
    setMaxLedAmplitude(SYNC_BASE_LED_AMPLITUDE);
  }

  if (!syncPulseOn && (long)(currentTime - nextSyncFlashMs) >= 0) {
    syncPulseOn = true;
    syncPulseCounter++;
    syncPulseEndMs = currentTime + SYNC_LED_PULSE_MS;
    nextSyncFlashMs = currentTime + SYNC_FLASH_INTERVAL_MS;
    setMaxLedAmplitude(SYNC_LED_AMPLITUDE);
  }
}

// ===================== Start Data Collection =====================

void startDataCollection() {
  if (!pollingActive) {
    stopSyncMode(false);
    setMaxLedAmplitude(NORMAL_LED_AMPLITUDE);
    pollingActive = true;
    sampleCounter = 0;
    nanCounter = 0;
    startTime = millis();
    lastPollTime = 0;  // Will trigger immediate first poll
    Serial.println("Data collection started.");
  }
}

// ===================== Stop Data Collection =====================

void stopDataCollection() {
  if (pollingActive) {
    pollingActive = false;

    // Calculate duration and sampling rate
    endTime = millis();
    unsigned long durationMs = endTime - startTime;
    float durationSec = durationMs / 1000.0;
    float samplingRate = (sampleCounter / durationSec);
    float realSamplingRate = (sampleCounter - nanCounter) / durationSec;
    float nanPercentage = (sampleCounter > 0) ? (nanCounter * 100.0 / sampleCounter) : 0;

    // Print results
    Serial.println("===== Data Collection Summary =====");
    Serial.print("Total samples: ");
    Serial.println(sampleCounter);
    Serial.print("Valid samples: ");
    Serial.println(sampleCounter - nanCounter);
    Serial.print("Sampling rate: ");
    Serial.print(samplingRate, 2);
    Serial.println(" Hz");
    Serial.print("Valid rate: ");
    Serial.print(realSamplingRate, 2);
    Serial.println(" Hz");
    Serial.print("Duration: ");
    Serial.print(durationSec, 2);
    Serial.println(" s");
    Serial.print("NaN count: ");
    Serial.println(nanCounter);
    Serial.print("NaN percentage: ");
    Serial.print(nanPercentage, 2);
    Serial.println(" %");
    Serial.println("===================================");

    Serial.println("Data collection stopped.");
    setMaxLedAmplitude(LED_OFF);
  }
}

// ===================== Setup =====================

void setup() {
  // Serial at 1000000 baud
  Serial.begin(1000000);
  while (!Serial)
    ;

  // Initialize sensors
  initializeSensors();

  // Clear data buffer
  memset(sensorData, 0, sizeof(sensorData));

  Serial.println("Ring");
}

// ===================== Loop =====================

void loop() {
  // Handle Serial commands
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "s") {
      startDataCollection();
    } else if (input == "e") {
      stopDataCollection();
      stopSyncMode();
    } else if (input == "t") {
      // Timestamp sync: respond with current millis() immediately
      Serial.print("T");
      Serial.println(millis());
    } else if (input == "SYNC_START") {
      startSyncMode();
    } else if (input == "SYNC_STOP") {
      stopSyncMode();
    } else {
      Serial.println("Invalid input. Use 's', 'e', 't', 'SYNC_START', or 'SYNC_STOP'.");
    }
  }

  // Poll sensors at the configured interval
  unsigned long currentTime = millis();
  if (syncModeActive) {
    updateSyncLed(currentTime);

    if (currentTime - lastPollTime >= pollInterval) {
      lastPollTime = currentTime;

      // Read sensor data
      readSensors();

      // Build and send standard 64-byte data packet
      sendDataPacket();
    }
  } else if (pollingActive && (currentTime - lastPollTime >= pollInterval)) {
    lastPollTime = currentTime;

    // Read sensor data
    readSensors();

    // Build and send data packet
    sendDataPacket();
  }
}
