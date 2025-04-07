#include <ESC.h>
#include <ESP32Servo.h>

#include <esp_now.h>
#include <WiFi.h>

#include <Adafruit_BMP280.h>
#include <SimpleKalmanFilter.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <Arduino_JSON.h>

#include "SPIFFS.h"

bool start_Drone = false;
bool initiation = true;
//-----------------------------Wifi Server---------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Replace with your network credentials
const char* ssid = "X13";
const char* password = "Xemnas13";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

unsigned long lastTimeDataSent = 0;  
unsigned long dataDelay = 200;

//-----------------------------Battery---------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
float max_volt = 8.7;
float min_volt = 7.2;
float resistor = 1651.51;
float real_resistor = 1620;
int battery_level;
float battery_voltage;
//-----------------------------KalmanFilter---------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
SimpleKalmanFilter altitudeKalmanFilter(1, 0.2, 0.3);
//-----------------------------BMP280---------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Adafruit_BMP280 bmp;
//-----------------------------MPU6050---------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------Brushless Motors--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#define ESC_PIN1 (18) // define motors pines
#define ESC_PIN2 (23)
#define ESC_PIN3 (15)
#define ESC_PIN4 (17)

#define LED_BUILTIN (2) // not defaulted properly for ESP32s/you must define it

#define MIN_SPEED 1000
#define MAX_SPEED 2000

ESC motor1 (ESC_PIN1, MIN_SPEED, MAX_SPEED, 500); // ESC_Name (PIN, Minimum Value, Maximum Value, Arm Value)
ESC motor2 (ESC_PIN2, MIN_SPEED, MAX_SPEED, 500);
ESC motor3 (ESC_PIN3, MIN_SPEED, MAX_SPEED, 500);
ESC motor4 (ESC_PIN4, MIN_SPEED, MAX_SPEED, 500);

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------PID------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
float deltaT;
long currentT = 0;
long prevT = 0;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------HEIGHT------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
float abs_altitude;
float current_height = 0;
int throttle = 0;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------PID_YAW------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
float kp_yaw = 0;
float ki_yaw = 0;
float kd_yaw = 0;

float e_yaw;
float eprev_yaw;
float eint_yaw;
float ederv_yaw;

float u_yaw;

float currentAngle_yaw;
int targetAngle_yaw = 0;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------PID_PITCH------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
float kp_pitch = 0;
float ki_pitch = 0;
float kd_pitch = 0;

float e_pitch;
float eprev_pitch;
float eint_pitch;
float ederv_pitch;

float u_pitch;

float currentAngle_pitch;
int targetAngle_pitch = 0;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------PID_ROLL------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
float kp_roll = 0;
float ki_roll = 0;
float kd_roll = 0;

float e_roll;
float eprev_roll;
float eint_roll;
float ederv_roll;

float u_roll;

float currentAngle_roll;
int targetAngle_roll = 0;
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  initWiFi();
  initSPIFFS();
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------Brushless_motors-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
  itialize_motors();
  delay(500);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------MPU6050---------------------------------------------------------------------------------------------------------------------------------------------------------------
  mpuInitialize();
  delay(500);
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------BMP280---------------------------------------------------------------------------------------------------------------------------------------------------------------
  bmp_initialize();
  delay(500);
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  web_server_initialize();
  delay(1500);
}

void loop() {
  get_Angle();
  get_altitude();
  PID();
  battery_status();
  update_web_server();
  // Serial.print(current_height);
  // Serial.print(" ");
  // Serial.print(currentAngle_yaw);
  // Serial.print(" ");
  // Serial.print(currentAngle_pitch);
  // Serial.print(" ");
  // Serial.print(currentAngle_roll);
  // Serial.print(" ");
  // Serial.print(u_h);
  // Serial.print(" ");
  // Serial.print(u_yaw);
  // Serial.print(" ");
  // Serial.print(u_pitch);
  // Serial.print(" ");
  // Serial.print(u_roll);
  // Serial.print(" ");
  // Serial.println(battery_level);
  delay(20);
}

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.println(WiFi.localIP());
}

void itialize_motors()
{
  pinMode(ESC_PIN1, OUTPUT);
  pinMode(ESC_PIN2, OUTPUT);
  pinMode(ESC_PIN3, OUTPUT);
  pinMode(ESC_PIN4, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(LED_BUILTIN, HIGH); // set led to on to indicate arming
  delay(500);
  motor1.arm(); // Send the Arm command to ESC
  motor2.arm();
  motor3.arm();
  motor4.arm();

  delay(5000); // Wait a while

  digitalWrite(LED_BUILTIN, LOW); // led off to indicate arming completed

  // the following loop turns on the motor slowly, so get ready
  for (int i=0; i < 450; i++){
    motor1.speed(MIN_SPEED-200+i);
    motor2.speed(MIN_SPEED-200+i); 
    motor3.speed(MIN_SPEED-200+i);
    motor4.speed(MIN_SPEED-200+i);
    delay(10);
  }
}

void mpuInitialize()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    delay(1500);
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(124.00000);
    mpu.setYGyroOffset(-20.00000);
    mpu.setZGyroOffset(49.00000);
    mpu.setZAccelOffset(2440.00000); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  delay(1000);
}

void bmp_initialize()
{
  Serial.println(F("BMP280 Forced Mode Test."));

  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  abs_altitude = bmp.readAltitude(1013.25);
}

void web_server_initialize()
{
  // Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request){
    start_Drone = true;
    request->send(200, "text/plain", "Start");
  });
  server.on("/end", HTTP_GET, [](AsyncWebServerRequest *request){
    start_Drone = false;
    request->send(200, "text/plain", "End");
  });

  server.on("/throttle", HTTP_GET, [](AsyncWebServerRequest *request) {
    if(request->hasParam("key")) {
      const AsyncWebParameter* p = request->getParam("key");
      String valueStr = p->value();
      
      // validate number
      bool isNumber = true;
      for(unsigned int i = 0; i < valueStr.length(); i++) {
        if(!isDigit(valueStr.charAt(i))) {
          isNumber = false;
          break;
        }
      }
      
      if(!isNumber) {
        request->send(400, "text/plain", "Error: non numeric value");
        return;
      }
      
      int value = valueStr.toInt();
      throttle = constrain(value, MIN_SPEED, MAX_SPEED);
      
      request->send(200, "text/plain", "value: " + String(value));
    } else {
      request->send(400, "text/plain", "Error: no 'key' parameter");
    }
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
}

void get_Angle()
{
  if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            currentAngle_yaw = ypr[0] * 180/M_PI;
            currentAngle_pitch = ypr[1] * 180/M_PI;
            currentAngle_roll = ypr[2] * 180/M_PI;
        #endif

    }
}

void get_altitude()
{
  if (bmp.takeForcedMeasurement())
  {
    current_height = altitudeKalmanFilter.updateEstimate(bmp.readAltitude(1013.25)) - abs_altitude;
  }
}

void PID()
{
  currentT = millis();
  deltaT = ((float)(currentT-prevT))/1.0e3;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------YAW-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  e_yaw = (float)targetAngle_yaw - currentAngle_yaw;
  ederv_yaw = (e_yaw - eprev_yaw)/deltaT;
  eint_yaw = eint_yaw + e_yaw*deltaT;
  //eint = eint + ((e - eprev)*deltaT)/2;
  u_yaw = kp_yaw*e_yaw + ki_yaw*eint_yaw + kd_yaw*ederv_yaw;

  eprev_yaw = e_yaw;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------PITCH------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  e_pitch = (float)targetAngle_pitch - currentAngle_pitch;
  ederv_pitch = (e_pitch - eprev_pitch)/deltaT;
  eint_pitch = eint_pitch + e_pitch*deltaT;
  //eint = eint + ((e - eprev)*deltaT)/2;
  u_pitch = kp_pitch*e_pitch + ki_pitch*eint_pitch + kd_pitch*ederv_pitch;

  eprev_pitch = e_pitch;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------ROLL------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  e_roll = (float)targetAngle_roll - currentAngle_roll;
  ederv_roll = (e_roll - eprev_roll)/deltaT;
  eint_roll = eint_roll + e_roll*deltaT;
  //eint = eint + ((e - eprev)*deltaT)/2;
  u_roll = kp_roll*e_roll + ki_roll*eint_roll + kd_roll*ederv_roll;

  eprev_roll = e_roll;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  prevT = currentT;
  
  if(start_Drone)
  {
    float value_motor1 = throttle - u_roll - u_pitch - u_yaw;
    float value_motor2 = throttle - u_roll + u_pitch + u_yaw;
    float value_motor3 = throttle + u_roll + u_pitch - u_yaw;
    float value_motor4 = throttle + u_roll - u_pitch + u_yaw;
    if (value_motor1 > MAX_SPEED) 
    {
      value_motor1 = MAX_SPEED;
    }
    else if (value_motor1 < MIN_SPEED) 
    {
      value_motor1 = MIN_SPEED;
    }
    if (value_motor2 > MAX_SPEED) 
    {
      value_motor2 = MAX_SPEED;
    }
    else if (value_motor2 < MIN_SPEED) 
    {
      value_motor2 = MIN_SPEED;
    }
    if (value_motor3 > MAX_SPEED) 
    {
      value_motor3 = MAX_SPEED;
    }
    else if (value_motor3 < MIN_SPEED) 
    {
      value_motor3 = MIN_SPEED;
    }
    if (value_motor4 > MAX_SPEED) 
    {
      value_motor4 = MAX_SPEED;
    }
    else if (value_motor4 < MIN_SPEED) 
    {
      value_motor4 = MIN_SPEED;
    }
    motor1.speed(value_motor1);
    motor2.speed(value_motor2);
    motor3.speed(value_motor3);
    motor4.speed(value_motor4);
    initiation = true;
  }
  else if(initiation)
  {
    end();
  }
  else
  {
    motor1.speed(0);
    motor2.speed(0); // motor starts up about half way through loop
    motor3.speed(0);
    motor4.speed(0);
  } 
}

void end()
{
  for(int i = throttle; i >= MIN_SPEED; i=-20)
  {
    motor1.speed(i);
    motor2.speed(i);
    motor3.speed(i);
    motor4.speed(i);
    delay(500);
  }
  initiation = false;
}

void battery_status()
{
  battery_voltage = (float) analogRead(36)/4096 * max_volt * (resistor/real_resistor);
  //battery_level = (int) map(voltage, min_volt, max_volt, 0, 100);
}

void update_web_server()
{
    if (millis() - lastTimeDataSent > dataDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(get_angle_to_send().c_str(),"mpu6050_readings",millis());
    events.send(String(battery_voltage).c_str(),"battery_reading",millis());
    events.send(String(current_height).c_str(),"bmp280_reading",millis());
    events.send(String(throttle).c_str(),"throttle_reding",millis());
    lastTimeDataSent = millis();
  }
}

String get_angle_to_send()
{
  readings["yaw"] = String(ypr[0]);
  readings["pitch"] = String(ypr[1]);
  readings["roll"] = String(ypr[2]);

  String jsonString = JSON.stringify(readings);
  return jsonString;
}
