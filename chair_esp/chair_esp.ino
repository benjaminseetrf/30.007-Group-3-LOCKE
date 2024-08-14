#include "Adafruit_VL53L0X.h"
#include <AccelStepper.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ezButton.h>

// Define pin connections
#define PUL_PIN 25
#define DIR_PIN 26
#define ENA_PIN 27

// Limit switch setup
ezButton outerlimitSwitch(13);  // Attach to pin GPIO26 for limit switch
ezButton innerlimitSwitch(14); 


/*
#define BUTTON1 26
#define BUTTON2 13
*/

// address for all 4 sensor
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33

/*
#define LOX5_ADDRESS 0x34
#define LOX6_ADDRESS 0x35
*/
int sensor1,sensor2,sensor3,sensor4;


// set the pins to shutdown for all 4 sensors
#define SHT_LOX1 16
#define SHT_LOX2 17
#define SHT_LOX3 5
#define SHT_LOX4 18
/*
#define SHT_LOX5 19
#define SHT_LOX6 3
*/
/*
#define FORCE_SENSOR_PIN1 4
#define FORCE_SENSOR_PIN2 2
#define FORCE_SENSOR_PIN3 15
#define FORCE_SENSOR_PIN4 12
#define FORCE_SENSOR_PIN5 14
#define FORCE_SENSOR_PIN6 27 
*/

#define MotorInterfaceType 1


// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x84, 0xFC, 0xE6, 0x6D, 0x73, 0xE8};

// Create an instance of the AccelStepper class
AccelStepper stepper = AccelStepper(MotorInterfaceType, PUL_PIN, DIR_PIN);

// Adjustable Variables
const unsigned long runTime = 1000; // 10000 milliseconds = 10 seconds, ADJUSTABLE
int MotorSpeed = 1000; // Steps per Second
int InverseSpeed = -MotorSpeed;
int DelayTime = 1000; // 1000 milliseconds = 1 second, ADJUSTABLE
bool outermotorActive = true; // State variable to track motor status
bool innermotorActive = true;

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
/*
Adafruit_VL53L0X lox5 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox6 = Adafruit_VL53L0X();
*/
// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;
/*
VL53L0X_RangingMeasurementData_t measure5;
VL53L0X_RangingMeasurementData_t measure6;
*/

typedef struct struct_message {

    int force_sensor1;
    int force_sensor2;
    int force_sensor3;
    int force_sensor4;
    int force_sensor5;
    int force_sensor6;

    bool button1_pressed; 
    bool button2_pressed;

    float prox1_value;
    int prox1_status;
    float prox2_value;
    int prox2_status;
    float prox3_value;
    int prox3_status;
    float prox4_value;
    int prox4_status;
    float prox5_value;
    int prox5_status;
    float prox6_value;
    int prox6_status;


} struct_message;

struct_message buttons;

struct_message screen;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&buttons, incomingData, sizeof(buttons));
  Serial.print("Bytes received: ");
  Serial.println(len);

  Serial.print(buttons.button1_pressed);
  Serial.print(buttons.button2_pressed);

  Serial.println();
  outerlimitSwitch.loop(); // MUST call the loop() function first to update the limit switch state
  innerlimitSwitch.loop();
  pressed();

}

void ButtonMotorControl() {
  if (buttons.button1_pressed == HIGH && outermotorActive) { // Check if button is pressed and motor is active
    digitalWrite(ENA_PIN, LOW); // Enable the motor driver
    stepper.setSpeed(MotorSpeed);
    unsigned long startTime = millis();
    while (millis() - startTime < runTime) {
      stepper.runSpeed();
      outerlimitSwitch.loop();
      if (outerlimitSwitch.isPressed()) {
        digitalWrite(ENA_PIN, HIGH); // Disable motor if limit switch is pressed during operation
        outermotorActive = false;
        Serial.println("Limit switch activated during operation. Motor stopped.");
        break;
      }
    }
    digitalWrite(ENA_PIN, HIGH); // Disable the motor driver after operation
  }
  
  
  
  
  
    // Update the state of the limit switch
/*  if (buttons.button1_pressed == HIGH) { // Button is pressed (assuming active low)
    limitSwitch.loop();  // Update the state of the limit switch      
    // Enable the motor driver
      if (limitSwitch.isPressed()) {
        Serial.println("Limit switch activated. Stopping motor.");
        stepper.setSpeed(0);
        digitalWrite(ENA_PIN, HIGH);
        return;
      }
      else{
    digitalWrite(ENA_PIN, LOW);
    stepper.setSpeed(MotorSpeed);
      }
    // Move forward for runTime duration
    unsigned long startTime = millis();
    while (millis() - startTime < runTime) { // Run for runTime duration
    if (limitSwitch.isPressed()) {
        Serial.println("Limit switch activated. Stopping motor.");
        stepper.setSpeed(0);
        digitalWrite(ENA_PIN, HIGH);
      }
      stepper.runSpeed();
    }

    // Disable the motor driver
    digitalWrite(ENA_PIN, HIGH);

    // Delay for DelayTime
    //delay(DelayTime);
  } 
  */

  else if (buttons.button2_pressed == HIGH && innermotorActive) { // Button is pressed (assuming active low)
    digitalWrite(ENA_PIN, LOW); // Enable the motor driver
    stepper.setSpeed(InverseSpeed);
    unsigned long startTime = millis();
    while (millis() - startTime < runTime) {
      stepper.runSpeed();
      innerlimitSwitch.loop();
      if (innerlimitSwitch.isPressed()) {
        digitalWrite(ENA_PIN, HIGH); // Disable motor if limit switch is pressed during operation
        innermotorActive = false;
        Serial.println("Limit switch activated during operation. Motor stopped.");
        break;
      }
    }
    digitalWrite(ENA_PIN, HIGH); // Disable the motor driver after operation
  } 
 
 
 
/*    // Enable the motor drive
    // Set speed to negative value to reverse direction
    digitalWrite(ENA_PIN, LOW);
    stepper.setSpeed(InverseSpeed);

    // Move backward for runTime duration
    unsigned long startTime = millis();
    while (millis() - startTime < runTime) { // Run for runTime duration 
      stepper.runSpeed();
      if (limitSwitch.isPressed()) {
        Serial.println("Limit switch activated. Stopping motor.");
        stepper.setSpeed(0);
        digitalWrite(ENA_PIN, HIGH);
      }
    }

    // Disable the motor driver
    digitalWrite(ENA_PIN, HIGH);
    */

    // Delay for DelayTime
    //delay(DelayTime);
  
}


void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);    
  digitalWrite(SHT_LOX4, LOW);  
  /*
  digitalWrite(SHT_LOX5, LOW);    
  digitalWrite(SHT_LOX6, LOW); 
*/
  delay(10);

  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  digitalWrite(SHT_LOX4, HIGH);  
  /*
  digitalWrite(SHT_LOX5, HIGH);
  digitalWrite(SHT_LOX6, HIGH);
*/
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  /*
  digitalWrite(SHT_LOX5, LOW);
  digitalWrite(SHT_LOX6, LOW);
*/
  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  ///************************* sensor 2 activation 
  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
  
   ///************************* sensor3 activation  
  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX3
  if(!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while(1);
  }  
  
   ///************************* sensor4 activation  
  // activating LOX4
  digitalWrite(SHT_LOX4, HIGH);
  delay(10);

  //initing LOX4
  if(!lox4.begin(LOX4_ADDRESS)) {
    Serial.println(F("Failed to boot fourth VL53L0X"));
    while(1);
  }
/*
   ///************************* sensor5 activation  
  // activating LOX5
  digitalWrite(SHT_LOX5, HIGH);
  delay(10);

  //initing LOX5
  if(!lox5.begin(LOX5_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }  
  
   ///************************* sensor6 activation  
  // activating LOX6
  digitalWrite(SHT_LOX6, HIGH);
  delay(10);

  //initing LOX6
  if(!lox6.begin(LOX6_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }  */
}

void read_hexa_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!
  lox3.rangingTest(&measure3, false); // pass in 'true' to get debug data printout!
  lox4.rangingTest(&measure4, false); // pass in 'true' to get debug data printout!
  /*
  lox5.rangingTest(&measure5, false); // pass in 'true' to get debug data printout!
  lox6.rangingTest(&measure6, false); // pass in 'true' to get debug data printout!
*/
  screen.prox1_value = measure1.RangeMilliMeter;
  screen.prox1_status = measure1.RangeStatus;
  screen.prox2_value = measure2.RangeMilliMeter;
  screen.prox2_status = measure2.RangeStatus;
  screen.prox3_value = measure3.RangeMilliMeter;
  screen.prox3_status = measure3.RangeStatus;
  screen.prox4_value = measure4.RangeMilliMeter;
  screen.prox4_status = measure4.RangeStatus;
  /*
  screen.prox5_value = measure5.RangeMilliMeter;
  screen.prox5_status = measure5.RangeStatus;
  screen.prox6_value = measure6.RangeMilliMeter;
  screen.prox6_status = measure6.RangeStatus;
  */
  /*
 // print sensor one reading
  Serial.print("sensor front / 1: ");
  if(measure1.RangeStatus != 4) {     // if not out of range
    sensor1 = measure1.RangeMilliMeter;    
    Serial.print(sensor1);
    Serial.print("mm");    
  } else {
    Serial.print("Out of range");
  }
  
  Serial.print(" ;");

  // print sensor two reading
  Serial.print("sensor left / 2: ");
  if(measure2.RangeStatus != 4) {
    sensor2 = measure2.RangeMilliMeter;
    Serial.print(sensor2);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }
 

   Serial.print(" ;");

  // print sensor three reading
  Serial.print("sensor back / 3: ");
  if(measure3.RangeStatus != 4) {
    sensor3 = measure3.RangeMilliMeter;
    Serial.print(sensor3);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }
  
  
  Serial.print(" ;");

  // print sensor four reading
  Serial.print("sensor right / 4: ");
  if(measure4.RangeStatus != 4) {
    sensor4 = measure4.RangeMilliMeter;
    Serial.print(sensor4);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }  

  Serial.print(" ;");

  // print sensor fifth reading
  Serial.print("sensor top: ");
  if(measure5.RangeStatus != 4) {
    sensor5 = measure5.RangeMilliMeter;
    Serial.print(sensor5);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }
 
   Serial.print(" ;");

  // print sensor sixth reading
  Serial.print("sensor bottom: ");
  if(measure6.RangeStatus != 4) {
    sensor6 = measure6.RangeMilliMeter;
    Serial.print(sensor6);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }

    Serial.println(); 
    */
}
/*
void read_pressure_sensors() {
  int sensor1 = analogRead(FORCE_SENSOR_PIN1);
  int sensor2 = analogRead(FORCE_SENSOR_PIN2);
  int sensor3 = analogRead(FORCE_SENSOR_PIN3);
  int sensor4 = analogRead(FORCE_SENSOR_PIN4);
  int sensor5 = analogRead(FORCE_SENSOR_PIN5);
  int sensor6 = analogRead(FORCE_SENSOR_PIN6);

  screen.force_sensor1 = sensor1;
  screen.force_sensor2 = sensor2;
  screen.force_sensor3 = sensor3;
  screen.force_sensor4 = sensor4;
  screen.force_sensor5 = sensor5;
  screen.force_sensor6 = sensor6;
  
  Serial.print("pin1 =");
  Serial.println(sensor1);
  Serial.print("pin2 =");
  Serial.println(sensor2);
  Serial.print("pin3 =");
  Serial.println(sensor3);
  Serial.print("pin4 =");
  Serial.println(sensor4);
  Serial.print("pin5 =");
  Serial.println(sensor5);
  Serial.print("pin6 =");
  Serial.println(sensor6);
  
} */

void start_motors() {
  // Set the enable pin mode and initially disable the motor
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH); // HIGH to disable, change if your driver needs LOW to disable

  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(1000); // Adjust as needed
  stepper.setAcceleration(100); // Adjust as needed

  // Set initial speed
  stepper.setSpeed(MotorSpeed);
}

void pressed(){
    if (outerlimitSwitch.isPressed()) {
    digitalWrite(ENA_PIN, HIGH); // Disable motor immediately
    outermotorActive = false; // Set motor state to inactive
    Serial.println("Limit switch activated. Motor stopped.");
  } else {
    if (outerlimitSwitch.isReleased()) { // Check if the motor was previously stopped
      outermotorActive = true; // Reactivate motor control after the switch is released
      Serial.println("Limit switch released. Motor can run.");
    }
    ButtonMotorControl(); // Proceed with normal operation only if the motor is active
  }

  if (innerlimitSwitch.isPressed()) {
    digitalWrite(ENA_PIN, HIGH); // Disable motor immediately
    innermotorActive = false; // Set motor state to inactive
    Serial.println("Limit switch activated. Motor stopped.");
  } else {
    if (innerlimitSwitch.isReleased()) { // Check if the motor was previously stopped
      innermotorActive = true; // Reactivate motor control after the switch is released
      Serial.println("Limit switch released. Motor can run.");
    }
    ButtonMotorControl(); // Proceed with normal operation only if the motor is active
  }
}


void setup() {
  Serial.begin(115200);
  outerlimitSwitch.setDebounceTime(50); // Debounce time for the limit switch
  innerlimitSwitch.setDebounceTime(50);
  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);
  pinMode(SHT_LOX4, OUTPUT);
  /*
  pinMode(SHT_LOX5, OUTPUT);
  pinMode(SHT_LOX6, OUTPUT);   
*/
  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  digitalWrite(SHT_LOX4, LOW);
  /*
  digitalWrite(SHT_LOX5, LOW);
  digitalWrite(SHT_LOX6, LOW);
*/
  Serial.println("All four in reset mode...(pins are low)");
  
  
  Serial.println("Starting...");
  setID();
  /*
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  */
  start_motors();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
   
  read_hexa_sensors();
  //read_pressure_sensors();

/*    if (limitSwitch.isPressed()) {
        digitalWrite(ENA_PIN, HIGH);  // Stop the motor
        Serial.println("Limit switch activated. Motor stopped.");
    } else {
        // Regular operation
        ButtonMotorControl();  // Function controlling motor based on button presses
    }
*/

  Serial.print("sensor front / 1: ");
  if(measure1.RangeStatus != 4) {     // if not out of range
    sensor1 = measure1.RangeMilliMeter;    
    Serial.print(sensor1);
    Serial.print("mm");    
  } else {
    Serial.print("Out of range");
  }
  
  Serial.print(" ;");

  // print sensor two reading
  Serial.print("sensor left / 2: ");
  if(measure2.RangeStatus != 4) {
    sensor2 = measure2.RangeMilliMeter;
    Serial.print(sensor2);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }
 

   Serial.print(" ;");

  // print sensor three reading
  Serial.print("sensor back / 3: ");
  if(measure3.RangeStatus != 4) {
    sensor3 = measure3.RangeMilliMeter;
    Serial.print(sensor3);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }
  
  
  Serial.print(" ;");

  // print sensor four reading
  Serial.print("sensor right / 4: ");
  if(measure4.RangeStatus != 4) {
    sensor4 = measure4.RangeMilliMeter;
    Serial.print(sensor4);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }  

  Serial.print(" ;");
/*
  // print sensor fifth reading
  Serial.print("sensor top: ");
  if(measure5.RangeStatus != 4) {
    sensor5 = measure5.RangeMilliMeter;
    Serial.print(sensor5);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }
 
   Serial.print(" ;");

  // print sensor sixth reading
  Serial.print("sensor bottom: ");
  if(measure6.RangeStatus != 4) {
    sensor6 = measure6.RangeMilliMeter;
    Serial.print(sensor6);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }*/

    Serial.println();

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &screen, sizeof(screen));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(1000); 
}