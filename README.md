# Real-Time-Temperature-and-RPM-monitoring-using-Google-Firebase
This  code snippet is a comprehensive implementation that integrates  temperature and rotary sensors and modules to collect and transmit data.

```cpp
#include <Wire.h>
#include <mcp2515.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Firebase_ESP_Client.h>
#include <ESP8266WiFi.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define SENDER_CAN_ID 0x124

const int spiCS = D8;
MCP2515 mcp2515Sender(spiCS);

const char *WIFI_SSID = "Airtel-MyWiFi-AMF-311WW-CF14";
const char *WIFI_PASSWORD = "12ac4bc8";
const char *API_KEY = "AIzaSyA1V05Jh53vRDa9SHQO4IdoSk2e0HQts10";
const char *DATABASE_URL = "https://fir-1673d-default-rtdb.firebaseio.com/";

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;

#define ENC_A D0
#define ENC_B D1
#define ENC_Z D2

const int ENC_COUNT_REV = 4096;
const int MAX_SPEED = 5000;
const float radius = 0.05;

volatile long encoderValue = 0;
unsigned long rpmInterval = 1000;
unsigned long rpmPreviousMillis = 0;
float rpm = 0;

#define SENSOR_2_BUS D3
#define SENSOR_3_BUS D4

OneWire sensor2Wire(SENSOR_2_BUS);
OneWire sensor3Wire(SENSOR_3_BUS);

DallasTemperature sensor2(&sensor2Wire);
DallasTemperature sensor3(&sensor3Wire);

struct can_frame canMsg;

enum State {
  WAIT_FOR_DATA,
  PRINT_RPM,
  PRINT_TEMP,
  TEMPORARY_STOP
};

State currentState = WAIT_FOR_DATA;
unsigned long lastTempPrintMillis = 0;
unsigned long rpmStopMillis = 0;
const unsigned long tempPrintInterval = 10000;

void ICACHE_RAM_ATTR updateEncoder() {
  encoderValue++;
}

float readTemperature(DallasTemperature &sensor);

void setup() {
  Serial.begin(9600);

  mcp2515Sender.reset();
  if (mcp2515Sender.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    while (1);
  }
  mcp2515Sender.setNormalMode();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
  }

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    signupOK = true;
  }

  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_Z, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_B), updateEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_Z), updateEncoder, FALLING);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - rpmPreviousMillis > rpmInterval) {
    rpmPreviousMillis = currentMillis;
    rpm = (float)(encoderValue * 60.0 / ENC_COUNT_REV);

    if (rpm > MAX_SPEED) {
      rpm = MAX_SPEED;
    }

    currentState = PRINT_RPM;
    encoderValue = 0;

    if (currentMillis - rpmStopMillis >= tempPrintInterval) {
      currentState = PRINT_TEMP;
      rpmStopMillis = currentMillis;
    }
  }

  switch (currentState) {

    case PRINT_RPM: {
      float speed = (2 * PI * radius * rpm) / 60.0;

      canMsg.can_id = SENDER_CAN_ID;
      canMsg.can_dlc = 8;
      memcpy(&canMsg.data[0], &rpm, sizeof(rpm));
      memcpy(&canMsg.data[4], &speed, sizeof(speed));
      mcp2515Sender.sendMessage(&canMsg);

      if (Firebase.ready() && signupOK) {
        String path_RPM = "RPM";

        if (Firebase.RTDB.setFloat(&fbdo, path_RPM.c_str(), rpm)) {
          Serial.println(rpm);
        } else {
          Serial.println(fbdo.errorReason());
        }
      }

      Serial.print("Sent RPM: ");
      Serial.print(rpm);
      Serial.print(" RPM, Speed: ");
      Serial.print(speed);
      Serial.println(" m/s");

      break;
    }

    case PRINT_TEMP: {
      float temp1 = readTemperature(sensor2);
      prepareAndSendTempCANMessage(temp1, "Sensor 2");

      if (Firebase.ready() && signupOK) {
        String path_T1 = "T1";

        if (Firebase.RTDB.setFloat(&fbdo, path_T1.c_str(), temp1)) {
          Serial.println(temp1);
        } else {
          Serial.println(fbdo.errorReason());
        }
      }

      float temp2 = readTemperature(sensor3);
      prepareAndSendTempCANMessage(temp2, "Sensor 3");

      if (Firebase.ready() && signupOK) {
        String path_T2 = "T2";

        if (Firebase.RTDB.setFloat(&fbdo, path_T2.c_str(), temp2)) {
          Serial.println(temp2);
        } else {
          Serial.println(fbdo.errorReason());
        }
      }

      currentState = PRINT_RPM;
      break;
    }
  }
}

float readTemperature(DallasTemperature &sensor) {
  sensor.requestTemperatures();
  float tempC = sensor.getTempCByIndex(0);
  Serial.println(tempC);
  return tempC;
}

void prepareAndSendTempCANMessage(float temp, const char* sensorName) {
  canMsg.can_id = SENDER_CAN_ID;
  canMsg.can_dlc = 4;
  memcpy(&canMsg.data[0], &temp, sizeof(temp));
  mcp2515Sender.sendMessage(&canMsg);

  Serial.print("Sent ");
  Serial.print(sensorName);
  Serial.print(" Temperature: ");
  Serial.println(temp);
}
```
### **Includes and Definitions**

```cpp
#include <Wire.h>
#include <mcp2515.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Firebase_ESP_Client.h>
#include <ESP8266WiFi.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
```

- **Libraries**: These libraries provide functionalities for:
  - `Wire.h`: I2C communication.
  - `mcp2515.h`: CAN bus communication using the MCP2515 CAN controller.
  - `OneWire.h` and `DallasTemperature.h`: Interface with DS18B20 temperature sensors.
  - `Firebase_ESP_Client.h`: Interfacing with Firebase.
  - `ESP8266WiFi.h`: Managing Wi-Fi connections on the ESP8266.
  - `TokenHelper.h` and `RTDBHelper.h`: Additional Firebase functionalities (e.g., token management and real-time database helpers).

```cpp
#define SENDER_CAN_ID 0x124
```

- **CAN ID Definition**: Defines the CAN ID used for sending messages over the CAN bus.

```cpp
const int spiCS = D8;
MCP2515 mcp2515Sender(spiCS);
```

- **MCP2515 CAN Module**: Specifies the chip select (CS) pin for the MCP2515 CAN controller and initializes the `mcp2515Sender` object for communication.

### **Wi-Fi and Firebase Configuration**

```cpp
const char *WIFI_SSID = "Airtel-MyWiFi-AMF-311WW-CF14";
const char *WIFI_PASSWORD = "12ac4bc8";
const char *API_KEY = "AIzaSyA1V05Jh53vRDa9SHQO4IdoSk2e0HQts10";
const char *DATABASE_URL = "https://fir-1673d-default-rtdb.firebaseio.com/";

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;
```

- **Wi-Fi Credentials**: Stores the SSID and password for connecting to a Wi-Fi network.
- **Firebase Credentials**: Stores the API key and database URL for Firebase authentication and real-time database operations.
- **Firebase Objects**: Initializes Firebase data, authentication, and configuration objects.
- **Signup Flag**: Indicates whether Firebase signup was successful.

### **Rotary Encoder Setup**

```cpp
#define ENC_A D0
#define ENC_B D1
#define ENC_Z D2

const int ENC_COUNT_REV = 4096;
const int MAX_SPEED = 5000;
const float radius = 0.05;

volatile long encoderValue = 0;
unsigned long rpmInterval = 1000;
unsigned long rpmPreviousMillis = 0;
float rpm = 0;
```

- **Encoder Pins**: Defines the pins connected to the rotary encoder.
- **Encoder Specifications**: Specifies the encoder pulses per revolution and the maximum speed in RPM.
- **Measurement Variables**: Variables to store encoder value, RPM interval, previous RPM calculation time, and current RPM.

### **Temperature Sensor Setup**

```cpp
#define SENSOR_2_BUS D3
#define SENSOR_3_BUS D4

OneWire sensor2Wire(SENSOR_2_BUS);
OneWire sensor3Wire(SENSOR_3_BUS);

DallasTemperature sensor2(&sensor2Wire);
DallasTemperature sensor3(&sensor3Wire);
```

- **Sensor Pins**: Defines the pins connected to DS18B20 temperature sensors.
- **Sensor Initialization**: Sets up OneWire and DallasTemperature objects to interface with the temperature sensors.

### **CAN Message Structure**

```cpp
struct can_frame canMsg;
```

- **CAN Frame Structure**: Initializes a `can_frame` structure to store CAN message data.

### **State Machine Setup**

```cpp
enum State {
  WAIT_FOR_DATA,
  PRINT_RPM,
  PRINT_TEMP,
  TEMPORARY_STOP
};

State currentState = WAIT_FOR_DATA;
unsigned long lastTempPrintMillis = 0;
unsigned long rpmStopMillis = 0;
const unsigned long tempPrintInterval = 10000;
```

- **State Machine**: Defines states for managing different operations: waiting for data, printing RPM, printing temperature, and a temporary stop.
- **State Variables**: Variables to store the current state and time intervals for RPM and temperature data processing.

### **Interrupt Service Routine (ISR)**

```cpp
void ICACHE_RAM_ATTR updateEncoder() {
  encoderValue++;
}
```

- **ISR for Encoder**: Updates the encoder value when an interrupt is triggered by the encoder signals.

### **Function Declaration**

```cpp
float readTemperature(DallasTemperature &sensor);
```

- **Function Declaration**: Declares a function to read the temperature from a specified DallasTemperature sensor.

### **Setup Function**

```cpp
void setup() {
  Serial.begin(9600);

  Serial.println("Initializing MCP2515 Sender...");
  mcp2515Sender.reset();
  if (mcp2515Sender.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    Serial.println("Error setting bitrate!");
    while (1);
  }
  mcp2515Sender.setNormalMode();
  Serial.println("MCP2515 Sender Initialized Successfully!");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", ""))
  {
    Serial.println("ok");
    signupOK = true;
  }
  else
  {
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_Z, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A), updateEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_B), updateEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_Z), updateEncoder, FALLING);
}
```

- **Serial Communication**: Initializes serial communication at 9600 baud rate.
- **CAN Module Initialization**: Sets up and initializes the MCP2515 CAN controller.
- **Wi-Fi Connection**: Connects to the specified Wi-Fi network and prints the assigned IP address.
- **Firebase Setup**: Configures Firebase with API key and database URL, performs signup, and starts Firebase services.
- **Pin Configuration**: Sets the encoder pins to input pull-up mode and attaches interrupts for the encoder signals.

### **Main Loop**

```cpp
void loop() {
  unsigned long currentMillis = millis();

  // Update RPM value every second
  if (currentMillis - rpmPreviousMillis > rpmInterval) {
    rpmPreviousMillis = currentMillis;

    // Calculate RPM
    rpm = (float)(encoderValue * 60.0 / ENC_COUNT_REV);

    // Handle overflow to avoid incorrect RPM calculation
    if (rpm > MAX_SPEED) {
      rpm = MAX_SPEED;
    }

    currentState = PRINT_RPM;
    encoderValue = 0;

    // Check if it's time to stop RPM sending and switch to temperature printing
    if (currentMillis - rpmStopMillis >= tempPrintInterval) {
      currentState = PRINT_TEMP;
      rpmStopMillis = currentMillis;
    }
  }

  // State machine logic
  switch (currentState) {

    case PRINT_RPM: {
      // Calculate speed based on RPM and radius
      float speed = (2 * PI * radius * rpm) / 60.0;

      // Prepare CAN message with RPM and speed data
      canMsg.can_id = SENDER_CAN_ID;
      canMsg.can_dlc = 8;
      memcpy(&canMsg.data[0], &rpm, sizeof(rpm));
      memcpy(&canMsg.data[4], &speed, sizeof(speed));
      mcp2515Sender.sendMessage(&canMsg);

      if (Firebase.ready() && signupOK) {
        String path_RPM = "RPM";
        if (Firebase.RTDB.setFloat(&fbdo, path_RPM.c_str(), rpm)) {
          Serial.print("RPM: ");
          Serial.println(rpm);
        } else {
          Serial.println("FAILED RPM");
          Serial.println("REASON: " + fbdo.errorReason());
        }
      }

      // Print sent data on Serial Monitor
      Serial.print("Sent RPM: ");
      Serial.print(rpm);
      Serial.print(" RPM, Speed: ");
      Serial.print(speed);
      Serial.println(" m/s");

      break;
    }

    case PRINT_TEMP: {
      // Read and send temperature from sensor 2
      float temp1 = readTemperature(sensor2);
      prepareAndSendTempCANMessage(temp1, "Sensor 2");

      if (Firebase.ready() && signupOK) {
        String path_T1 = "T1";
        if (Firebase.RTDB.setFloat(&fbdo, path_T1.c_str(), temp1)) {
          Serial.print("Temperature T1: ");
          Serial.println(temp1);
        } else {
          Serial.println("FAILED T1");
          Serial.println("REASON: " + fbdo

.errorReason());
        }
      }

      // Read and send temperature from sensor 3
      float temp2 = readTemperature(sensor3);
      prepareAndSendTempCANMessage(temp2, "Sensor 3");

      if (Firebase.ready() && signupOK) {
        String path_T2 = "T2";
        if (Firebase.RTDB.setFloat(&fbdo, path_T2.c_str(), temp2)) {
          Serial.print("Temperature T2: ");
          Serial.println(temp2);
        } else {
          Serial.println("FAILED T2");
          Serial.println("REASON: " + fbdo.errorReason());
        }
      }

      currentState = PRINT_RPM; // Switch back to sending RPM
      break;
    }
  }
}
```

- **Main Loop**: Handles the periodic updating and sending of RPM and temperature data based on the state machine.
- **RPM Calculation**: Calculates RPM and speed every second and prepares the CAN message.
- **State Machine**: Switches between states to either print RPM or temperature data. Also manages Firebase data updates and CAN message transmissions.

### **Utility Functions**

```cpp
float readTemperature(DallasTemperature &sensor) {
  sensor.requestTemperatures();
  float tempC = sensor.getTempCByIndex(0);

  // Print temperature data for debugging
  Serial.print("Temperature: ");
  Serial.println(tempC);

  return tempC;
}

void prepareAndSendTempCANMessage(float temp, const char* sensorName) {
  // Prepare CAN message with temperature data
  canMsg.can_id = SENDER_CAN_ID;
  canMsg.can_dlc = 4;
  memcpy(&canMsg.data[0], &temp, sizeof(temp));
  mcp2515Sender.sendMessage(&canMsg);

  // Print sent temperature data on Serial Monitor
  Serial.print("Sent ");
  Serial.print(sensorName);
  Serial.print(" Temperature: ");
  Serial.print(temp);
  Serial.println(" °C");
}
```

- **Temperature Reading**: Reads temperature from a given DallasTemperature sensor and returns the value in Celsius.
- **CAN Message Preparation**: Prepares and sends CAN messages containing temperature data.

---
