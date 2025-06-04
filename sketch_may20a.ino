#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

#define buzzer 4        // Buzzer 
#define LedGreen 7      // Green LED
#define LedRed 6        // Red LED
#define Reed2 5         // reed 2
#define LedYellow 3     // Yellow LED
#define Reed1 8         // reed 1
#define Temppin A0      // Temp pin

LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 columns, 2 rows

const byte ROWS = 4; // no of the Rows
const byte COLS = 1; // no of the col 
char keys[ROWS][COLS] = {
  {'2'},
  {'5'},
  {'8'},
  {'0'}
};
byte rowPins[ROWS] = {13, 12, 11, 10}; // pin for the row
byte colPins[COLS] = {9}; // pin of the col
Keypad myKeypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

unsigned long startTime = 0; // start time 
bool timingStarted = false; // timing
bool systemEnabled = false;  // flag to track if '2' was pressed
float temp_c;                // varabile for storing temp in cel
float temp_f;                // vatraible for storing temp in f
int analog_output;           // store the output 
float rho_in;

void setup() {
  Serial.begin(9600); //  serial begin 

  pinMode(buzzer, OUTPUT);  // buzzer output
  pinMode(LedGreen, OUTPUT);  // Ledgreen output 
  pinMode(LedRed, OUTPUT);  // ledred output
  pinMode(LedYellow, OUTPUT);  // ledyellow output 
  pinMode(Reed1, INPUT_PULLUP);  // reed 1 input 
  pinMode(Reed2, INPUT_PULLUP);  // reed 2 input 

  lcd.init(); // initialize the lcd 
  lcd.backlight(); // 
  lcd.clear(); // clear the lcd 
  lcd.setCursor(0, 0); //  set the cursor 
  lcd.print("Men in fluid"); 
  delay(2000); // delay 2000ms 
  lcd.clear(); //  clear the lcd
  lcd.print("Press 5 to Start"); 

  digitalWrite(LedRed, HIGH);       // Red ON
  digitalWrite(LedGreen, LOW);      // Green OFF
  digitalWrite(LedYellow, LOW);     // Yellow OFF
}

void loop() {
  // Wait for '2' key press to start system
  if (!systemEnabled) {
    char key = myKeypad.getKey(); // get the button 
    if (key == '5') {
      systemEnabled = true; // enable the system 
      digitalWrite(LedGreen,HIGH);  // make the green led on (high)
      digitalWrite(LedRed,LOW);
      lcd.clear(); //  clear the lcd
      lcd.print("Ready for magnet");
      int analogValue = analogRead(Temppin);
      float voltage = analogValue * (5.0 / 1023.0);
      Serial.println(analogValue);  
      Serial.println("System Enabled");
    }
    return;  // Skip rest of loop until '2' is pressed
  }

  int reed1 = digitalRead(Reed1); // get the reed 1 response 1 is high and no agnet is detected and 0 is low and magnet detcted  
  int reed2 = digitalRead(Reed2); // get the reed 2 response 1 is high and no agnet is detected and 0 is low and magnet detcted  

  if (reed1 == LOW && !timingStarted) {
    startTime = millis(); // start the timer 
    timingStarted = true;  // make the state true 
    Serial.print(reed1); // plot the reed signal 
    

    digitalWrite(LedRed, LOW);       // Red OFF
    digitalWrite(LedGreen, HIGH);    // Green ON
    digitalWrite(LedYellow, HIGH);   // Yellow ON

    lcd.clear(); // clear the lcd 
    lcd.setCursor(0, 0);  // set the cursor to the 0,0
    lcd.print("Magnet detected");  // 
    Serial.println("Magnet detected - Timer started");  // 

    tone(buzzer, 1000, 100);  // make the buzzer 1000 khz 
    delay(300); // delay 300ms
  }

  if (reed2 == LOW && timingStarted) {
    Serial.println("Reed2"); //
    unsigned long elapsed = millis() - startTime; // time taken 
    timingStarted = false;  // make timer stop

    digitalWrite(LedGreen, LOW);     // Green OFF
    digitalWrite(LedYellow, LOW);    // Yellow OFF
    digitalWrite(LedRed, HIGH);  // Red ON
    tone(buzzer,1000); // let the buzzer 1000khz 
    delay(100); // delay 100ms
    noTone(buzzer); //make the buzzer have no sound 

    lcd.clear();  // make the lcd clear
    lcd.setCursor(0, 0); // set the cursor 0,0
    lcd.print("Elapsed time:");  //
    lcd.setCursor(0, 1); // set the cursor 
    lcd.print(elapsed); // print the time elapsed 
    lcd.print(" ms"); //

    Serial.print("Elapsed time: ");
    Serial.print(elapsed);
    Serial.println(" ms");

    // Viscosity calculation here:
    // Viscosity calculation here:
// Measured time and distance
    float timeSeconds = elapsed / 1000.0; // time in seconds 
    float distanceMeters = 0.12;  // distance of the constant velocity
    float velocity = distanceMeters / timeSeconds;  // terminal velocity (m/s)

    // Constants
    float g = 9.8;            // gravity
    float rho_s = 5100.0;     // piston density (kg/m^3)
    float rho_f = 850.00;      // fluid density (kg/m^3)
    float R = 0.016;          // piston radius (m)
    float h = 0.03;           // piston height (m)

    float delta_rho = rho_s - rho_f;  // differnece of the density 
    float h_over_R = h / R;  
    float correction = h_over_R / log(1.0 + h_over_R);  // ln in Arduino is log()

    float viscosity = ((delta_rho * g * R * R) / (4.0 * velocity))/40004* correction;
    lcd.clear(); // make the lcd clear
    lcd.setCursor(0, 0); // set the cursor 
    lcd.print("Viscosity:"); // 
    lcd.setCursor(0, 1); //set the cursor to 0,1
    lcd.print(viscosity, 4); //
    lcd.print(" Pa.s"); // 




    delay(3000);  // show elapsed time first
    int analogValue = analogRead(Temppin); // take the analog signal from temp sensor
    float voltage = analogValue * (5.0 / 1023.0); // convert the analog value to a voltage 
    float temperatureC = ((voltage - 0.5) * 100.0) - 9;  // convert volatge to temp in celsuis

    lcd.clear(); // clear lcd 
    lcd.setCursor(0, 0); // set the cursor to 0,0
    lcd.print("Temp:"); 
    lcd.print(temperatureC, 1); // print the temp 
    lcd.print((char)223);
    lcd.print("C");

    delay(3000); // delay 3000ms


    lcd.clear(); // make the lcd clear
    lcd.print("Press 5 to start");  
    tone(buzzer, 1500, 200); // make the buzzer make 1500khz 
  }
}
