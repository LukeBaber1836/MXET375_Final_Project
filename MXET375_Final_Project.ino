#include <Adafruit_INA260.h>

// Motor control
#define DIR 2 
#define EN  A0
#define encoder  3
int Speed(int newSpeed);
int currentSpeed = 0;

// Data collection
void print_labes();
void publishData();
// INA260 is connected to the SDA (A4) and SCL (A5) ports
Adafruit_INA260 ina260 = Adafruit_INA260(); // voltage,current, power sensor
unsigned long freq;
unsigned long rpm;
float startTime;
float seconds;
float testTime = 1; //Time in seconds that the motor will run
float voltage;
float current;


// Digital input
const int button = 4;
int buttonState = LOW;

void setup() {
  // ------------------ Motor Setup -----------------
  pinMode(DIR, OUTPUT);
  pinMode(EN, OUTPUT);
  
  // Initialize our direction and speed to 0
  Speed(currentSpeed);
  delay(10);   // Wait for things to settle

  // ------------------ Data Setup ------------------
  pinMode(encoder, INPUT);
  
  Serial.begin(9600); // Baud Rate ENSURE this is the same in the Python Script

  // Check if serial port is open
  while (!Serial) { delay(10); }

  Serial.println("Searching for INA260...");

  // Check that INA260 chip is connected
  if (!ina260.begin()) {
    Serial.println("Error: No INA260 chip");
    while (1);
  }
  Serial.println("Found INA260");

  // -------------- Digital Input Setup -------------
  pinMode(button, INPUT);
}

void loop() {
  buttonState = digitalRead(button); // Read the state of the button
  
  if (buttonState == HIGH) {
     // Turn on the motor
     Speed(255);
     
     // Begin Testing
     startTime = millis();
     seconds = 0;

     // Print top row labels
     print_labes();
     
     // Gather data for testTime number of seconds
     while (seconds <= testTime) {
      // Data is taken in mV and mA, results converted to V and Amps
      current = (ina260.readCurrent()) / 1000;
      voltage = (ina260.readBusVoltage()) / 1000;
      
      //Frequency in Hz
      freq = 100000 / checkRPM();
      rpm = (freq * 60);

      // Time in seconds
      seconds = ((millis() - startTime)/1000);

      // Output data to the serial monitor
      publishData();
     }
     
     // Turn off motor, prepare for next test
     Speed(0);
     delay(1000);
  }
}

// -------------------- Motor Control Function -------------------------
// Set the speed of the motor
int Speed(int newSpeed) {
  // Ensure that new speed is within bounds
  if (newSpeed > 255) newSpeed = 255;
  else if (newSpeed < -255) newSpeed = -255;
  
  // Set our new speed
  currentSpeed = newSpeed;
  
  // Get the absolute value
  if (newSpeed < 0) newSpeed = -newSpeed;
  
  // Set our pin to output the new speed
  analogWrite(EN, newSpeed);
  
  return currentSpeed;
}

// -------------------- Data Collection Functions ---------------------

// Check motor RPM
int checkRPM() {
  int highPulse;
  int lowPulse;
  int pulseTotal;
  
  highPulse = pulseIn(encoder, HIGH);
  pulseTotal = highPulse*2;

  return pulseTotal;
}

void print_labes() {
  Serial.print("Time (seconds)\t");
  Serial.print("Voltage (volts)\t");
  Serial.print("Current (Amps)\t");
  Serial.print("RPMs (Motor before gearbox)\n");
}

// Write the recorded data over serial
void publishData() {
  // Time (seconds)
  Serial.print(seconds);
  Serial.print("\t");
  delay(10); //This delay allows the time to only be recorded every 10 ms
  
  // Voltage (volts)
  Serial.print(voltage);
  Serial.print("\t");
  
  // Current (Amps)
  Serial.print(current);
  Serial.print("\t");

  // Revolutions (rpm)
  Serial.println(rpm);
}
