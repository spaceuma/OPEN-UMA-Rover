#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Define the pins for the UART communication
const int RX_PIN = 18;
const int TX_PIN = 19;

// Create a software serial object to communicate with the sensor
SoftwareSerial sensorSerial(RX_PIN, TX_PIN);

// Create an object to communicate with the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  // Start the serial communication with the computer
  Serial.begin(9600);

  // Start the serial communication with the sensor
  sensorSerial.begin(9600);

  // Initialize the BNO055 sensor
  if(!bno.begin())
  {
    Serial.println("Failed to initialize BNO055!");
    while(1);
  }
  // Set the operation mode to IMU (no magnetometer)
  bno.setMode(OPERATION_MODE_IMUPLUS);
}

void loop() {
  // Read the orientation data from the sensor
  sensors_event_t event;
  bno.getEvent(&event);

  // Print the orientation data to the serial monitor
  Serial.print("Orientation: ");
  Serial.print(event.orientation.x);
  Serial.print(",");
  Serial.print(event.orientation.y);
  Serial.print(",");
  Serial.println(event.orientation.z);

  // Wait a short time before reading again
  delay(100);
}