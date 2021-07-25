


/*
  
  FastBob's Rotor Controller V3
  Initially based on the Leamington_G-2800DXA code.

  Bob Bownes July 10, 2021

  Converted to run on Pro Micro
  
  Uses I2C LCD display
  Adapted to other rotors.
  
  Interface to rotor control box
  
    Output: rotate_left (TTL - Active LOW)
    Output: rotate_right (TTL - Active LOW)
      
    Output: MotorSpeed (PWM 25%, 50%, 75%, 100%)
    Output: brake_bit (TTL - Active LOW)

    Rotor direction and brake control outputs are tied to base of 2N2222 to ground 
      the output pin and activate the rotor
    Speed is PWM output to an RC filter to convert to voltage.
  
    Input: Position (0-5V corresponds to 0-450 degrees rotation)
  
  Uses Arduino A/D and D/A functions for Speed and Position.  

  Emulates Yeasu GS-232A Azimuth Controller

  Remaining todo's:
  
  1) Convert the last of the id(debug) statments to #ifdefs to conserve code space
  2) Add elevaton control and El ADC.
  3) Clean up and use return variables rather than globals
  4) Modify to use all 450 degrees of rotation
  5) Move volts to degree conversion and read_adc to a single function
  6) Make motor speed selection persistant when changed
  7) Make tolerant of 2 digit heading input
  8) clean up display routine to add leading zero
  9) Clean up the relay code to fully remove the L/R relays from the K commands
  10) re-enable calibration switch
  
  
  
   
*/

#include <LCD_I2C.h>

// Set up the LCD

LCD_I2C lcd(0x3f);  // set the LCD address to 0x27 for a 16 chars and 2 line display

#define VERSION 3.2

// #define DEBUG
// #define BRAKE


// Debug Mode must be off to communicate with Ham Radio Deluxe

#define debug_mode 0  // Set to 1 for debug data on Serial Port - THIS SHOULD GO AWAY

#include <EEPROM.h>  // Include the EEPROM Library

#define relay_1  6     // Define Relay 1 as Pin 6 - Available for other use
#define relay_2  7     // Define Relay 2 as Pin 7 - Available for other use

#define relay_3  8     // Define Relay 3 as Pin 8 - Rotate Left (CCW)
#define relay_4  9     // Define Relay 4 as Pin 9 - Rotate Right (CW)

// #define brake_bit 5    // Define Brake On/Off as pin 5 (Active LOW)

#define rotate_left 8  // Define Rotate Left as Pin 7 (Active LOW)
#define rotate_right 9 // Define Rotate Right as Pin 6 (Active LOW)
#define MotorSpeed 10  // Define Rotator Speed Control as Pin 11 (PWM - 25,50,75,100%)
#define AZ_Position A0    // Define Rotator Position Pot (0-5V : 0-450 degrees)
#define EL_Position A1    // Define Rotator Elevation Position Pot (0-5V : 0-180 degrees)


#define EEP_Enable 12  // Enable EE_PROM writes (Active LOW)

#define BAUD_RATE 9600  // Set the Serial Port Baud rate to 9600

#define EEPROM_ID_BYTE 1     // EEPROM ID to validate EEPROM data location
#define EEPROM_ID  56        // EEPROM ID Value
#define EEPROM_AZ_CAL_0 2    // Azimuth Zero Calibration EEPROM location   
#define EEPROM_AZ_CAL_MAX 12 // Azimuth Max Calibration Data EEPROM location 

#define AZ_CAL_0_DEFAULT     2  // Preset the Azimuth Zero Calibration Point to 0
#define AZ_CAL_MAX_DEFAULT 1020  // Preset the Azimuth Max (450 degree) Calibration Point

#define AZ_Tolerance 1  // Set the Azimuth Accuracy Tolerance (degrees)
#define AZ_Coast 2      // Set the Azimuth Coast distance (degrees)
#define AZ_Window 4     // Set the Azimuth Deceleration window (degrees)

#define Speed_1 63      // Motor speed 1
#define Speed_2 127     // Motor speed 2
#define Speed_3 191     // Motor speed 3
#define Speed_4 255     // Motor speed 4

#define SERIAL_BUFFER_MAX  50 // Max size of the serial buffer fo boundary checking

//variables


byte inByte = 0;  // incoming serial byte
byte serial_buffer[SERIAL_BUFFER_MAX];  // incoming serial byte buffer
int serial_buffer_index = 0;  // The index pointer variable for the Serial buffer
int set_AZ;  // Azimuth set value
int current_AZ;  // Current Azimuth raw value
int current_EL;  // Current Elevation raw value
String Serial_Send_Data; // Data to send to Serial Port
int AZ_0;  // Azimuth Zero Value from EEPROM
int AZ_MAX; // Azimuth Max Value from EEPROM
int AZ_Degrees; // mapped AZ ADC value to Degrees
String Requested_AZ; // RS232 Requested Azimuth - M and short W command
int AZ_To; // Requested AZ Move
int AZ_Distance; // Distance to move AZ
int current_degrees;


void setup()
{

  lcd.begin();                      // initialize the lcd 


// Print a message to the LCD.
  lcd.backlight();
  lcd.noCursor();
  lcd.noBlink();
  lcd.setCursor(0,0);
  lcd.print("FastBob's Rotor");
  lcd.setCursor(0,1);
  lcd.print("Controller ");
  lcd.print(VERSION);
  delay(5000);
  lcd.clear();


  
  pinMode(relay_1, OUTPUT);
  pinMode(relay_2, OUTPUT);
  pinMode(relay_3, OUTPUT);
  pinMode(relay_4, OUTPUT);

#ifdef BRAKE
  pinMode(brake_bit,OUTPUT);
#endif

  pinMode(rotate_left, OUTPUT); // Define the Control Pins as Outputs
  pinMode(rotate_right, OUTPUT);
  pinMode(MotorSpeed, OUTPUT);
  pinMode(AZ_Position, INPUT);
  pinMode(EL_Position, INPUT);

  pinMode(EEP_Enable, INPUT);

  digitalWrite(relay_1, LOW);       // Turn off all the relays just to be sure
  digitalWrite(relay_2, LOW);       // Turn off all the relays just to be sure
  digitalWrite(relay_3, LOW);       // Turn off all the relays just to be sure  
  digitalWrite(relay_4, LOW);       // Turn off all the relays just to be sure

#ifdef BRAKE
  digitalWrite(brake_bit, LOW);       // Turn off the brake
#endif

  digitalWrite(EEP_Enable, HIGH);   // EE_Prom Enable pullup
  Serial.begin(BAUD_RATE);          // initialize serial communication

  set_AZ = -1;                      // Preset the Azimuth Move Variable
  az_rotate_stop();                 // Halt rotation
  read_eeprom_cal_data();           // Read the Azimuth Calibration Values from EEPROM

}  // End Setup Loop



//////////////////////////////
//                          //
// Main Loop Starts Here    //
//                          //
//////////////////////////////

void loop()
{
  check_serial(); // Check the Serial Port for Data
  check_move();   // Check to see if executing move command
  update_display();  // Update the display
} // End Main Loop




// Functions

void read_eeprom_cal_data()  // Function to Read the Azimuth Calibration Data
{
  if (EEPROM.read(EEPROM_ID_BYTE) == EEPROM_ID) // Verify the EEPROM has valid data
  {
    AZ_0   = (EEPROM.read(EEPROM_AZ_CAL_0)   * 256) + EEPROM.read(EEPROM_AZ_CAL_0 + 1);    // Read the Azimuth Zero Calibration Value from EEPROM
    AZ_MAX = (EEPROM.read(EEPROM_AZ_CAL_MAX) * 256) + EEPROM.read(EEPROM_AZ_CAL_MAX + 1);  // Read the Azimuth Maximum Calibration Value from EEPROM

#ifdef DEBUG // If in Debug Mode Print the Calibration Values
      Serial.println("Read EEPROM Calibration Data Valid ID");
      Serial.println(AZ_0  , DEC);
      Serial.println(AZ_MAX, DEC);
#endif

  }
  else // initialize eeprom to default values
  {

#ifdef DEBUG
      Serial.println("Read EEPROM Calibration Data Invalid ID - setting to defaults");
#endif

    AZ_0 = AZ_CAL_0_DEFAULT;  // Set the Calibration To Default Values
    AZ_MAX = AZ_CAL_MAX_DEFAULT;
    write_eeprom_cal_data();  // Write the Default Values to EEPROM
  }
}

void write_eeprom_cal_data() // Function to Write the Calibration Values to EEPROM
{
  Serial.println("Writing EEPROM Calibration Data");
  Serial.println(AZ_0  , DEC);
  Serial.println(AZ_MAX, DEC);
  EEPROM.write(EEPROM_ID_BYTE, EEPROM_ID); //   Write the EEPROM ID
  EEPROM.write(EEPROM_AZ_CAL_0, highByte(AZ_0)); // Write the Azimuth Zero Calibration High Order Byte
  EEPROM.write(EEPROM_AZ_CAL_0 + 1, lowByte(AZ_0));   // Write the Azimuth Zero Calibration Low Order Byte
  EEPROM.write(EEPROM_AZ_CAL_MAX, highByte(AZ_MAX)); // Write the Azimuth Max Calibration High Order Byte
  EEPROM.write(EEPROM_AZ_CAL_MAX + 1, lowByte(AZ_MAX)); // Write the Azimuth Max Calibration Low Order Byte
}


void check_serial() // Function to check for data on the Serial port
{
  if (Serial.available() > 0) // Get the Serial Data if available
  {
    inByte = Serial.read();  // Get the Serial Data
    
    // You may need to uncomment the following line if your PC software
    // will not communicate properly with the controller
    // Serial.print(char(inByte));  // Echo back to the PC
    if (inByte == 10)  // ignore Line Feeds
    {
      return;
    }
    if (inByte != 13) // Add to buffer if not CR
    {
      serial_buffer[serial_buffer_index] = inByte;
      if (debug_mode) // Print the Character received if in Debug mode
      {
        Serial.print("Received = ");
        Serial.println(serial_buffer[serial_buffer_index]);
      }
      serial_buffer_index++;  // Increment the Serial Buffer pointer & boundary check
      if(serial_buffer_index > SERIAL_BUFFER_MAX)
        serial_buffer_index = SERIAL_BUFFER_MAX;
    }
    else   // It's a Carriage Return, execute command
    {
      if ((serial_buffer[0] > 96) && (serial_buffer[0] < 123))  //If first character of command is lowercase, convert to uppercase
        serial_buffer[0] = serial_buffer[0] - 32;

      // Decode first character of command
      switch (serial_buffer[0])
      {
// A Command - Stop the Azimuth Rotation
        case 65:
          if (debug_mode)
            Serial.println("A Command Received");
          az_rotate_stop();
          break;

// C - Return current azimuth
        case 67:
          if (debug_mode)   // Return the Buffer Index Pointer in Debug Mode
          {
            Serial.println("C Command Received");
            Serial.println(serial_buffer_index);
          }
          send_current_az();  // Return Azimuth if C Command
          break;

// F - Set the Max Calibration
        case 70:
//          if (digitalRead(EEP_Enable))
//            Serial.println("Calibration Disabled");
//          else
          {
            if (serial_buffer_index == 2) // It's a secondary calibration F0=0, F5=450, F?=ask
            {
              if (serial_buffer[1] == 48) // It's an F0 - Calibrate the 0 degree value
              {
                if (debug_mode)
                  Serial.println("F0 command Received");
                set_0_cal();
                break;
              }
              if (serial_buffer[1] == 53) // It's an F5 - Calibrate the 450 degree value
              {
                if (debug_mode)
                  Serial.println("F5 command Received");
                set_max_az_cal();  // F - Set the Max Azimuth Calibration
                break;
              }
              if (serial_buffer[1] == 63) // It's an F? - Report calibration values
              {
                if (debug_mode)
                  Serial.println("F? command Received");
                Serial.println(AZ_0  , DEC);
                Serial.println(AZ_MAX, DEC);
                break;
              }
            }
            else
              break;
          }
          
// K - Control Relays
        case 75:
          if (debug_mode)
          {
            Serial.println("K Command Received");
            Serial.println(serial_buffer_index);
          }
          if (serial_buffer_index == 2)
          {
            if (serial_buffer[1] == 48)
              // Disable Relays
            {
              if (debug_mode)
                Serial.println("K0 command Received");
              digitalWrite(relay_1, LOW);       // Turn off relay 1
              digitalWrite(relay_2, LOW);       // Turn off relay 2
//            digitalWrite(relay_3, LOW);       // Turn off relay 3
//            digitalWrite(relay_4, LOW);       // Turn off relay 4
              break;
            }
            if (serial_buffer[1] == 49)
              // Enable Relay 1
            {
              if (debug_mode)
                Serial.println("K1 command Received");
              digitalWrite(relay_2, LOW);       // Turn off relay 2
 //             digitalWrite(relay_3, LOW);       // Turn off relay 3
//              digitalWrite(relay_4, LOW);       // Turn off relay 4
              digitalWrite(relay_1, HIGH);      // Turn on  relay 1
              break;
            }
            if (serial_buffer[1] == 50)
              // Enable Relay 2
            {
              if (debug_mode)
                Serial.println("K2 command Received");
              digitalWrite(relay_1, LOW);       // Turn off relay 1
//              digitalWrite(relay_3, LOW);       // Turn off relay 3
//              digitalWrite(relay_4, LOW);       // Turn off relay 4
              digitalWrite(relay_2, HIGH);      // Turn on  relay 2
              break;
            }
            if (serial_buffer[1] == 51)
              // Enable Relay 3
            {
              if (debug_mode)
                Serial.println("K3 command Received");
              digitalWrite(relay_1, LOW);       // Turn off relay 1
              digitalWrite(relay_2, LOW);       // Turn off relay 2
//              digitalWrite(relay_4, LOW);       // Turn off relay 4
 //             digitalWrite(relay_3, HIGH);      // Turn on  relay 3
              break;
            }
            if (serial_buffer[1] == 52)
              // Enable Relay 4
            {
              if (debug_mode)
                Serial.println("K4 command Received");
              digitalWrite(relay_1, LOW);       // Turn off relay 1
              digitalWrite(relay_2, LOW);       // Turn off relay 2
//              digitalWrite(relay_3, LOW);       // Turn off relay 3
//              digitalWrite(relay_4, HIGH);      // Turn on  relay 4
              break;
            }
          }
          else
            break;
          break;

// L - Rotate Azimuth CCW
        case 76:
          if (debug_mode)
            Serial.println("L Command Received");
          analogWrite(MotorSpeed, Speed_4); // Restore speed
          rotate_az_ccw();  // Call the Rotate Azimuth CCW Function
          break;

// M - Rotate to Set Point
        case 77:
          if (debug_mode)
            Serial.println("M Command Received");
          analogWrite(MotorSpeed, Speed_4); // Restore speed
          rotate_to();  // Call the Rotate to Set Point Command
          break;

// R - Rotate Azimuth CW
        case 82:
          if (debug_mode)
            Serial.println("R Command Received");
          analogWrite(MotorSpeed, Speed_4); // Restore speed
          rotate_az_cw();  // Call the Rotate Azimuth CW Function
          break;

// S - Stop All Rotation
        case 83:
          if (debug_mode)
            Serial.println("S Command Received");
          az_rotate_stop();  // Call the Stop Azimith Rotation Function
          break;

// X - Motor Speed Control
        case 88:
          if (serial_buffer_index == 2)
          {
            if (serial_buffer[1] == 49)
            {
              if (debug_mode)
                Serial.println("X1 command Received");
              analogWrite(MotorSpeed, Speed_1);
              break;
            }
            if (serial_buffer[1] == 50)
            {
              if (debug_mode)
                Serial.println("X2 command Received");
              analogWrite(MotorSpeed, Speed_2);
              break;
            }
            if (serial_buffer[1] == 51)
            {
              if (debug_mode)
                Serial.println("X3 command Received");
              analogWrite(MotorSpeed, Speed_3);
              break;
            }
            if (serial_buffer[1] == 52)
            {
              if (debug_mode)
                Serial.println("X4 command Received");
              analogWrite(MotorSpeed, Speed_4);
              break;
            }
            else if (debug_mode)
              Serial.println("Invalid X command Received");
            break;
          }
          if (debug_mode)
            Serial.println("Invalid command Received");
      }
      serial_buffer_index = 0;  // Clear the Serial Buffer and Reset the Buffer Index Pointer
      serial_buffer[0] = 0;
    }
  }
}

void send_current_az() // Send the Current Azimuth Function
{
  read_adc();  // Read the ADC
// Map Azimuth to degrees
  if (debug_mode)
    Serial.println(current_AZ);
  AZ_Degrees = Degrees(current_AZ);
  if (debug_mode)
    Serial.println(AZ_Degrees);
// Send it back via serial
  Serial_Send_Data = "";
  if (AZ_Degrees < 100)  // pad with 0's if needed
    Serial_Send_Data = "0";
  if (AZ_Degrees < 10)
    Serial_Send_Data = "00";
  Serial_Send_Data = "+0" + Serial_Send_Data + String(AZ_Degrees);  // Send the Azimuth in Degrees
  Serial.println(Serial_Send_Data);  // Return value via RS-232 port
}

void set_0_cal() // Set the 0 degree Azimuth Calibration Function
{
  Serial.println("Calibrate 0 Degree AZ Function");
  read_adc();  // Read the ADC
// save current Azimuth value to EEPROM - 0 degree Calibration
  Serial.println(current_AZ);
  AZ_0 = current_AZ;  // Set the Azimuth 90 degree Calibration to Current Azimuth Reading
  write_eeprom_cal_data();  // Write the Calibration Data to EEPROM
  Serial.println("0 degree Azimuth Calibration Complete");
}

void set_max_az_cal() // Set the Max Azimuth (450 degree) Calibration Function
{
  Serial.println("Calibrate Max AZ (450 Degree) Function");
  read_adc();  // Read the ADC
// save current az and el values to EEPROM - Zero Calibration
  Serial.println(current_AZ);
  AZ_MAX = current_AZ;  // Set the Azimuth Maximum (450 degree) Calibration to Current Azimuth Reading
  write_eeprom_cal_data();  // Write the Calibration Data to EEPROM
  Serial.println("Max Azimuth (450 degree) Calibration Complete");
}

void rotate_az_ccw() // Function to Rotate Azimuth CCW
{
#ifdef BRAKE
  brake_off();
#endif
  digitalWrite(rotate_left,  LOW);  // Set the Rotate Left Pin Low (active)
  digitalWrite(rotate_right, HIGH);  // Make sure the Rotate Right Pin is High (inactive)
}

void rotate_az_cw() // Function to Rotate Azimuth CW
{
#ifdef BRAKE
  brake_off();
#endif
  digitalWrite(rotate_right, LOW);    // Set the Rotate Right Pin Low
  digitalWrite(rotate_left, HIGH);    // Make sure the Rotate Left Pin High
}

void az_rotate_stop() // Function to Stop Azimuth Rotation
{
  digitalWrite(rotate_right, LOW);  // Turn off the Rotate Right Pin
  digitalWrite(rotate_left, LOW);   // Turn off the Rotate Left Pin
  analogWrite(MotorSpeed, Speed_4);
  set_AZ = -1;
#ifdef BRAKE
  brake_on();
#endif
}

void rotate_to() // Begin rotation to target azimuth
{
  int target,current;
  
#ifdef DEBUG
    Serial.println("M Command -  Rotate Azimuth To Function");
#endif

// Decode Command - Format Mxxx - xxx = Degrees to Move to
// NOTE: MUST be full 3 digits. Valuse < 100 must have leading 0

#ifdef DEBUG
    Serial.println(serial_buffer_index);
 #endif
    
  if (serial_buffer_index == 4)  // Verify the Command is the proper length
  {

#ifdef DEBUG
      Serial.println("Value in [1] to [3]?");
#endif

    Requested_AZ = (String(char(serial_buffer[1])) + String(char(serial_buffer[2])) + String(char(serial_buffer[3])));  // Decode the Azimuth Value
    AZ_To = (Requested_AZ.toInt()); // Target location as integer
    
    if (AZ_To < 0) // Make sure we don't go below 0 degrees
      AZ_To = 0;
    if (AZ_To > 360) // Make sure we don't go over 450 degrees
      AZ_To = 360;
      
 #ifdef DEBUG
      Serial.println(Requested_AZ);
      Serial.println(AZ_To);
 #endif

// Use set_AZ=-1 as stop indicator. if set_AZ >= 0, we are moving    
// set the move flag and start

    read_adc();  // Read the ADC to get current location
    
// Map it to degrees

#ifdef DEBUG
      Serial.println(current_AZ);
#endif

    AZ_Degrees = Degrees(current_AZ);

#ifdef DEBUG
      Serial.println(AZ_Degrees);
#endif
      
     current = AZ_Degrees;

     set_AZ = AZ_To;
     target = AZ_To;

//We know where we are going and where we are, figure out how to get there with a rotor break @ 180

    if(current <= 180) {    // If the current location is in the right hemisphere
      if(target <current) { // and the target is closer to 0
        rotate_az_ccw();    // turn ccw
      } else                // Otherwise if the target is in the left hemisphere
      if(target > 180) {
        rotate_az_ccw();    // turn left past 0
      } else {
        rotate_az_cw();     // Otherwise it is between us and 180, so turn cw
      }
    }
    else {   // current >180 and we are in the left hemisphere
      if (target > current) {  // Target is closer to 360
        rotate_az_cw();         // turn cw 
      } else {
        if (target >180) {    // if target is in this hemisphere and < current location
          rotate_az_ccw();    // turn left
        } else {  
          rotate_az_cw();     // otherwise turn right past 0
        }
      } 
    }
  }
}

void read_adc() // Read the ADC
{
  current_AZ = analogRead(AZ_Position);  // Read ADC Channel 0
#ifdef DEBUG
 
    Serial.println("Read ADC Function 0 ");    Serial.println(current_AZ);
      current_degrees = Degrees(current_AZ);
      Serial.print(current_degrees);
#endif
}

void check_move() // Check to see if we are moving and, if necessary, stop
{
  if (set_AZ != -1)   // We're moving - check and stop as needed
  {
    read_adc();  // Read the ADC
// Map AZ to degrees

#ifdef DEBUG
      Serial.println(current_AZ);
#endif

    AZ_Degrees = Degrees(current_AZ);
    
#ifdef DEBUG
Serial.println(AZ_Degrees);
#endif
   AZ_Distance = set_AZ - AZ_Degrees;  // Check how far we have to move
   if (abs(AZ_Distance) <= AZ_Window)  // Approaching target - slow down
     analogWrite(MotorSpeed, Speed_1);
   if (abs(AZ_Distance) <= AZ_Coast)  // Turn off early to allow coast
   {
     az_rotate_stop();  // Stop the Azimuth Rotation
     set_AZ = -1;  // Turn off the Azimuth Move Command
   }
  }
}

int Degrees(int az_value) // Convert the ADC value to degrees
{
  int corrected_degrees;
  if (az_value <= AZ_0)
    corrected_degrees = 0;
  if ((az_value >= AZ_0) && (az_value <= AZ_MAX))
    corrected_degrees = map(az_value, AZ_0, AZ_MAX, 0, 450);
  if (az_value > AZ_MAX)
    corrected_degrees = 450;
  
  if (corrected_degrees >360) {
    corrected_degrees = corrected_degrees - 180;
    return(corrected_degrees);
  }
  if(corrected_degrees < 180) {
    corrected_degrees = corrected_degrees + 180;
  }
  else
    corrected_degrees = corrected_degrees - 180;
  return (corrected_degrees);
}

void update_display()
{
 
  // update current pointing
  lcd.setCursor(0,0);
  lcd.print(F("Current: "));
  read_adc();
  current_degrees = Degrees(current_AZ);
    if(current_degrees < 100)             // Fill in leading zeros
      lcd.print("0");
    if(current_degrees < 10)
      lcd.print("0");
  lcd.print(current_degrees);
  //update target pointing
  lcd.setCursor(0,1);
  lcd.print(F("Target:  "));
  if (set_AZ <0) {
    if(current_degrees < 100) // Fill in leading zeros
      lcd.print("0");
    if(current_degrees < 10)
      lcd.print("0");
    lcd.print(current_degrees);
    display_stopped();
  }
  else {
    if(set_AZ < 100)  // Fill in leading zeros
      lcd.print("0");
    if(set_AZ < 10)
      lcd.print("0");
    lcd.print(set_AZ);
    display_moving();
  }
}

void display_stopped()      // Put an indicator on the LCD that we are stopped
{
  lcd.setCursor(15,0);
//  setTextColor(WHITE, BLACK);
  lcd.print("B");
//  setTextColor(BLACK,WHITE); 
}

void display_moving() //Put an indicator on the LCD that we are moving
{
  lcd.setCursor(15,0);
//  setTextColor(WHITE, BLACK);
  lcd.print("M");
//  lcd.setTextColor(BLACK,WHITE);
}



#ifdef BRAKE
void brake_on() // Turn on the brake
{
  digitalWrite(brake_bit,LOW);
  lcd.setCursor(11,1);
//  setTextColor(WHITE, BLACK);
  lcd.print("B");
//  setTextColor(BLACK,WHITE);
}

void brake_off()
{
  digitalWrite(brake_bit,HIGH);
  lcd.setCursor(15,1);
//  setTextColor(WHITE, BLACK);
  lcd.print(" ");
 // setTextColor(BLACK,WHITE); 
}
#endif BRAKE


