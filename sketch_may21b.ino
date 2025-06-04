#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

#define buzzer 4        // Buzzer pin at pin 4 Digital
#define LedGreen 7      // Green LED pin at 7 Digital
#define LedRed 6        // Red LED pin at 6 Digital
#define Reed2 5         // second reed at pin 5 
#define LedYellow 3     // Yellow LED pin at 3 
#define Reed1 8         // first reed at pin 8

LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 columns, 2 rows

char liquids [10][100]; // array of liquids 

const byte ROWS = 4; // no of bytes for row
const byte COLS = 1; // no of col 
char keys[ROWS][COLS] = {
  {'2'},
  {'5'},
  {'8'},
  {'0'}
};
byte rowPins[ROWS] = {13, 12, 11, 10}; // row pins : 1st row at 13 , 2nd row at 12 , 3rd row at 11  , 4th row at 10
byte colPins[COLS] = {9}; // col pins : 2nd col at 9
Keypad myKeypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS); //create keypad object 

unsigned long startTime = 0;  // time start
bool timingStarted = false;  // inintial state of timer
bool systemEnabled = false;  // flag to track if '2' was pressed

void Appendstring(const char* s)
{
  int i = 0;
  if (count < 10)
  {
    while(s[i] != '\0' && i < 100){
      liquids[count][i];
      i++;
      }
    strings[count][i] = '\0';  // Null-terminate
    count++;
  }  
}

void setup() {
  Serial.begin(9600); // begin serial 

  pinMode(buzzer, OUTPUT); // buzzer mode output
  pinMode(LedGreen, OUTPUT); // ledgreen output
  pinMode(LedRed, OUTPUT); // ledred output
  pinMode(LedYellow, OUTPUT); // ledyellow output
  pinMode(Reed1, INPUT_PULLUP); // reed1 input
  pinMode(Reed2, INPUT_PULLUP); // reed2 input

  lcd.init(); // initialize lcd 
  lcd.backlight();
  lcd.clear(); // clear lcd 
  lcd.setCursor(0, 0); // set cursor to (0,0)
  lcd.print("Men in fluid");
  delay(2000); // delay 2000 ms
  lcd.clear(); // clear lcd 
  lcd.print("Press 2 to Start");

  digitalWrite(LedRed, HIGH);       // Red ON
  digitalWrite(LedGreen, LOW);      // Green OFF
  digitalWrite(LedYellow, LOW);     // Yellow OFF
}

void loop() {
  // Wait for '2' key press to start system
  if (!systemEnabled) {
    char key = myKeypad.getKey();
    if (key == '2') {
      systemEnabled = true; // system is on
      lcd.clear(); // clear lcd 
      lcd.print("Ready for magnet");
      Serial.println("System Enabled");
    }
    return;  // Skip rest of loop until '2' is pressed
  }

  int reed1 = digitalRead(Reed1); // read reed1 return value 1 0r 0 (if 1 itis high no magnet)
  int reed2 = digitalRead(Reed2); // read reed1 return value 1 0r 0 

  if (reed1 == LOW && !timingStarted) {
    startTime = millis();
    timingStarted = true;

    digitalWrite(LedRed, LOW);       // Red OFF
    digitalWrite(LedGreen, HIGH);    // Green ON
    digitalWrite(LedYellow, HIGH);   // Yellow ON

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Magnet detected");
    Serial.println("Magnet detected - Timer started");

    tone(buzzer, 1000, 100);
    delay(300);
  }

  if (reed2 == LOW && timingStarted) {
    Serial.println("Reed2");
    unsigned long elapsed = millis() - startTime;
    timingStarted = false;

    digitalWrite(LedGreen, LOW);     // Green OFF
    digitalWrite(LedYellow, LOW);    // Yellow OFF
    digitalWrite(LedRed, HIGH);  // Red ON
    tone(buzzer,1000);
    delay(100);
    noTone(buzzer);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Elapsed time:");
    lcd.setCursor(0, 1);
    lcd.print(elapsed);
    lcd.print(" ms");

    Serial.print("Elapsed time: ");
    Serial.print(elapsed);
    Serial.println(" ms");

    // Viscosity calculation here:
    float timeSeconds = elapsed / 1000.0;      // convert ms to s
    float distanceMeters = 0.12;                // 12 cm = 0.12 m
    float radius = 0.015;                       // 3 cm diameter → 1.5 cm radius → 0.015 m
    float g = 9.81;                            // gravity m/s^2
    float rho_s = 7500.0;                      // density magnet kg/m^3 (neodymium approx)
    float rho_f = 1000.0;                      // density fluid (water) kg/m^3

    float velocity = distanceMeters / timeSeconds;  // m/s

    // Stokes Law: eta = (2 * r^2 * g * (rho_s - rho_f)) / (9 * v)
    float viscosity = (2 * radius * radius * g * (rho_s - rho_f)) / (9 * velocity);

    Serial.print("Viscosity: ");
    Serial.print(viscosity, 4);
    Serial.println(" Pa.s");

    delay(2000);  // show elapsed time first

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Viscosity:");
    lcd.setCursor(0, 1);
    lcd.print(viscosity, 4);
    lcd.print(" Pa.s");

    delay(3000);  // show viscosity

    lcd.clear();
    lcd.print("Ready for magnet");

    tone(buzzer, 1500, 200);
  }
}
