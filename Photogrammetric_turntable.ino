//============================================================================================
/*
  Reworked the original code from Brian Brocken

  Using an ESP32 D1 R32 board for Bluetooth connectivity
  Reworked the Menu system
  Added saving defaults to EEPROM and retrieving then at startup
  Added BT HID Keyboard operation to eliminate the servo that operates BT Remote shutter
*/
//============================================================================================

#include <Wire.h>  // needed for the LCD I2C library.
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>  // i2c LCD i/o class header
#include <EEPROM.h>                         // We will get/put the default settings from/to te EEPROM
#include <Stepper.h>                        // Lib for the stepper motors
#include <BleKeyboard.h>                    // Lib for Bluetooth keyboard emulation

// Joystick Pins
#define X_PIN 36  // analog pin connected to X output
#define Y_PIN 39  // analog pin connected to Y output
#define SW_PIN 4  // digital pin connected to switch output

// Stepper Pins
#define IN1_PIN 16
#define IN2_PIN 17
#define IN3_PIN 25
#define IN4_PIN 26

// Menu-system
#define MENUITEM_LENGTH 16   // max length of a menuitem
#define LOW_THRESHOLD 500    // below this analog value, joystick is considered low
#define HIGH_THRESHOLD 3500  // above this analog value, joystick is considered high
#define SLOW_FASTCHANGEDELAY 600
#define FASTCHANGEDELAY 200

#define DEFAULTS_EYECATCHER 0b10101010
#define DEFAULTS_VERSION 0x01

// Get the number of elements in a given array.
// Use like `len(myvar)`
template<class T, size_t N> constexpr size_t len(const T (&)[N]) {
  return N;
}

// Stepper parameters
const int stepsPerRevolution = 2048;                                        // change this to fit the number of steps per revolution
int FullRev = 7 * stepsPerRevolution;                                       // 1 full revolution of the big gear -> Small-Big gear ratio is 7:1
Stepper myStepper(stepsPerRevolution, IN1_PIN, IN3_PIN, IN2_PIN, IN4_PIN);  // Use these pins for the stepper motor

// We use a 16x2 with I2C backpack
hd44780_I2Cexp lcd;  // Assumes default I2C address of 0x27

// LCD geometry
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

// We will emulate a BLE Keyboard
BleKeyboard myKB("Bluetooth shutter", "Espressif", 100);

// Global variables for joystick operation
int SwValue = 1;
int SwPrevValue = 1;
int XValue = 512;
int XPrevValue = 512;
int YValue = 512;
int YPrevValue = 512;
bool prevDirection = 0;

// Global variables for menu system
int MainMenuPos = 0;
int SubMenuPos = 0;
char MainMenu[6][MENUITEM_LENGTH] = {
  "Main Menu      ",
  "Photogrammetric",
  "Cinematic      ",
  "Manual mode    ",
  "Defaults       "
};
char PhotoMenu[7][MENUITEM_LENGTH] = {
  "> Photogrametry",
  "#P#",  // placeholder nmbr of photos
  "#S#",  // placeholder motor speed
  "#B#",  // placeholder delay before
  "#A#",  // placeholder delay after
  "Press to start ",
  "Return to Main "
};
char CineMenu[5][MENUITEM_LENGTH] = {
  "> Cinematic    ",
  "#T#",  // placeholder nmbr of turns
  "#S#",  // placeholder motor speed
  "Press to start ",
  "Return to Main "
};
char ManualMenu[5][MENUITEM_LENGTH] = {
  "> Manual mode  ",
  "#S#",  // placeholder motor speed
  "#Z#",  // placeholder step size
  "Press to start ",
  "Return to Main "
};
char DefaultsMenu[11][MENUITEM_LENGTH] = {
  "> Defaults     ",
  "#P#",  // placeholder nmbr of photos
  "#T#",  // placeholder nmbr of turns
  "#S#",  // placeholder motor speed
  "#Z#",  // placeholder step size
  "#D#",  // placeholder key to send
  "#K#",  // placeholder key to send
  "#B#",  // placeholder delay before
  "#A#",  // placeholder delay after
  "Press to save  ",
  "Return to Main "
};

// FastChange variables
const unsigned long FastDelay = 1000;  // delay mode time (before values change fast)
int FastChng = 0;                      // indicates fast change value mode.  0 = off, 1 = up mode, -1 = down mode
unsigned long SetTime = 0;             // time value for fast change & button cancel modes.  Used to calculate time intervals

// Hardcoded operation parameter limits
const int MAX_PHOTOS = 200;
const int MIN_PHOTOS = 2;
const int MAX_TURNS = 200;
const int MIN_TURNS = 1;
const int MAX_SPEED = 17;
const int MIN_SPEED = 1;
const int MAX_SIZE = 150;
const int MIN_SIZE = 1;
const int MAX_KEY = 2;
const int MIN_KEY = 1;
const int MAX_BEFORE = 5;
const int MIN_BEFORE = 0;
const int MAX_AFTER = 10;
const int MIN_AFTER = 1;

// The current values of the operation parameters
int CurTurns;
int CurSpeed;
int CurPhotos;
int CurStepSize;
char CurDirection;
int CurDelayBefore;
int CurDelayAfter;
int CurKey2Send;

// The structure used to store defaults to the EEPROM
struct MyDefaults_struct {
  uint8_t EyeCatcher;
  uint8_t Version;
  int DefaultGearRatio;
  int DefaultTurns;
  int DefaultSpeed;
  int DefaultPhotos;
  int DefaultStepSize;
  int DefaultKey2Send;
  int DefaultDelayBefore;
  int DefaultDelayAfter;
  char DefaultDirection;
};
int EEPROM_SIZE = sizeof(MyDefaults_struct);
// The global instance of the default-structure
MyDefaults_struct MyDefaults;
const int eeAddress = 0;  // We expect the defaults to be at address 0 in the eeprom
const int BAUDRATE = 9600;

// enum rows_t {
//   row1,
//   row2
// };

// enum alignment_t {
//   left,
//   middle,
//   right
// };

// struct startup_screen_struct {
//   rows_t row = row1;
//   alignment_t alignment = left;
//   char text[];
// };

// Max chars per line is 40, total chars for 16x2 display is 80.
// If both lines are used and both exceed 40 chars, will exceed
// display data RAM (DDRAM) and override the other line.
// void startupScreen(startup_screen_struct* phrases[], size_t delay_ms) {
//   int len = 0;
//   int offset = 0;

//   for (startup_screen_struct& phrase : phrases) {
//     len = len(phrase.text);
//     if (len > MENUITEM_LENGTH) {
//       phrase.alignment = left;
//     }

//     switch (phrase.alignment) {
//       case left:
//         // lcd.setCursor(offset, phrase.row);
//         break;
//       case middle:
//         offset = (MENUITEM_LENGTH - len) / 2 break;
//       case right:
//         offset = (MENUITEM_LENGTH - len) break;
//     }
//     lcd.setCursor(offset, phrase.row);
//     lcd.autoscroll();
//     lcd.lineWrap();
//   }
// }

//============================================================================================
// setup
//============================================================================================
void setup() {
  int status;
  status = lcd.begin(LCD_COLS, LCD_ROWS);
  if (status)  // non zero status means it was unsuccesful
  {
    // begin() failed so blink error code using the onboard LED if possible
    hd44780::fatalError(status);  // does not return
  }
  lcd.backlight();                // LCD turn backlight on
  pinMode(SW_PIN, INPUT_PULLUP);  // Joystick pushbutton as input
  EEPROM.begin(EEPROM_SIZE);
  Serial.begin(BAUDRATE);

  // Ready to connect via Bluetooth
  myKB.begin();
  Serial.println("Ready to connect over Bluetooth...");

  // Read the defaults from the EEPROM
  Serial.println("Reading defaults from EEPROM...");
  EEPROM.get(eeAddress, MyDefaults);

  if ((MyDefaults.EyeCatcher != DEFAULTS_EYECATCHER) || (MyDefaults.Version != DEFAULTS_VERSION)) {
    // EEPROM does not have saved default values, so use hardcoded ones instead
    Serial.println("No defaults found in EEPROM...");
    Serial.println("Using hardcoded default values...");
    MyDefaults.DefaultGearRatio = 1;
    MyDefaults.DefaultTurns = 1;
    MyDefaults.DefaultSpeed = 15;
    MyDefaults.DefaultPhotos = 8;
    MyDefaults.DefaultStepSize = 2;
    MyDefaults.DefaultKey2Send = 1;
    MyDefaults.DefaultDelayBefore = 1;  // 1=Vol.Down (Android), 2=Vol.Up (IOS)
    MyDefaults.DefaultDelayAfter = 1;
    MyDefaults.DefaultDirection = 'L';  // L=Left (Clockwise), R=Right (Counterclockwise)
    MyDefaults.EyeCatcher = DEFAULTS_EYECATCHER;
    MyDefaults.Version = DEFAULTS_VERSION;
  } else {
    Serial.println("Defaults found in EEPROM...");
    Serial.print("Defaults version is ");
    Serial.println(MyDefaults.Version, DEC);
    Serial.print("Default gear ratio= ");
    Serial.println(MyDefaults.DefaultGearRatio, DEC);
    Serial.print("Default turns= ");
    Serial.println(MyDefaults.DefaultTurns, DEC);
    Serial.print("Default manual step-size= ");
    Serial.println(MyDefaults.DefaultStepSize, DEC);
    Serial.print("Default direction= ");
    Serial.println(MyDefaults.DefaultDirection);
    Serial.print("Default photos= ");
    Serial.println(MyDefaults.DefaultPhotos, DEC);
    Serial.print("Default key to send= ");
    Serial.print(MyDefaults.DefaultKey2Send, DEC);
    Serial.println(" (1=Enter; 2=Volume Up)");
    Serial.print("Default delay before= ");
    Serial.println(MyDefaults.DefaultDelayBefore, DEC);
    Serial.print("Default delay after= ");
    Serial.println(MyDefaults.DefaultDelayAfter, DEC);
  }
  // Startup Screens
  lcd.setCursor(4, 0);
  lcd.print("Welcome!");
  lcd.setCursor(0, 1);
  lcd.print("ESP32 Soft.V1.0.");
  delay(3000);
  lcd.clear();
  lcd.print("Designed by");
  lcd.setCursor(0, 1);
  lcd.print("Brian Brocken");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Software by");
  lcd.setCursor(0, 1);
  lcd.print("Marco64");
  delay(3000);
  lcd.clear();
  // lcd.autoscroll("Bug fixes by swirle13")
}

//============================================================================================
// keyPressed - determines if the joystick is pressed
//============================================================================================
bool keyPressed() {
  if ((SwValue == 0) && (SwPrevValue != SwValue)) {
    SetTime = 0;
    FastChng = 0;
    return (1);
  } else {
    return (0);
  }
}

//============================================================================================
// rightPressed - determines if the joystick is moved to the right
//============================================================================================
bool rightPressed() {
  if ((XValue <= LOW_THRESHOLD) && (XPrevValue > LOW_THRESHOLD)) {
    prevDirection = 1;
    FastChng = 0;
    SetTime = 0;
    FastChng = 0;
    return (1);
  } else {
    return (0);
  }
}

//============================================================================================
// leftPressed - determines if the joystick is moved to the left
//============================================================================================
bool leftPressed() {
  if ((XValue >= HIGH_THRESHOLD) && (XPrevValue < HIGH_THRESHOLD)) {
    prevDirection = 1;
    FastChng = 0;
    SetTime = 0;
    FastChng = 0;
    return (1);
  } else {
    return (0);
  }
}

//============================================================================================
// upPressed - determines if the joystick is moved up
//============================================================================================
bool upPressed() {
  if ((YValue <= LOW_THRESHOLD) && (YPrevValue > LOW_THRESHOLD)) {
    SetTime = millis();
    prevDirection = 1;
    FastChng = 0;
    return (1);
  } else {
    if ((YValue <= LOW_THRESHOLD) && (FastChng == 0) && ((millis() - SetTime) > FastDelay)) {
      FastChng = 1;
      Serial.println("Entering fastchange mode");
    }
    return (0);
  }
}

//============================================================================================
// downPressed - determines if the joystick is moved down
//============================================================================================
bool downPressed() {
  if ((YValue >= HIGH_THRESHOLD) && (YPrevValue < HIGH_THRESHOLD)) {
    SetTime = millis();
    prevDirection = 1;
    FastChng = 0;
    return (1);
  } else {
    if ((YValue >= HIGH_THRESHOLD) && (FastChng == 0) && ((millis() - SetTime) > FastDelay)) {
      FastChng = -1;
      Serial.println("Entering fastchange mode");
    }
    return (0);
  }
}

//============================================================================================
// noDirection - determines if the joystick is in 'neutral' position
//============================================================================================
bool noDirection() {
  if ((YValue < HIGH_THRESHOLD) && (YValue > LOW_THRESHOLD) && (XValue < HIGH_THRESHOLD) && (XValue > LOW_THRESHOLD) && !keyPressed() && prevDirection == 1) {
    SetTime = 0;
    FastChng = 0;
    prevDirection = 0;
    return (1);
  } else {
    return (0);
  }
}

//============================================================================================
// readInputs - read the analog values from the joystick
//============================================================================================
void readInputs() {
  // save previous values
  SwPrevValue = SwValue;
  XPrevValue = XValue;
  YPrevValue = YValue;
  // Read all inputs
  XValue = analogRead(X_PIN);     // Read the analog value from The X-axis from the joystick
  YValue = analogRead(Y_PIN);     // Read the analog value from The Y-axis from the joystick
  SwValue = digitalRead(SW_PIN);  // Read the digital value from The Button from the joystick
}

//============================================================================================
// placeholderPrint - print menuitems with placeholders and their values to the LCD screen
//============================================================================================
void placeholderPrint(char* iPlaceholder) {

  lcd.setCursor(0, 1);
  if (strcmp(iPlaceholder, "#P#") == 0) {
    lcd.print("# Photos:      ");
    lcd.setCursor(13, 1);
    lcd.print(CurPhotos);
  } else if (strcmp(iPlaceholder, "#T#") == 0) {
    lcd.print("# Turns:        ");
    lcd.setCursor(13, 1);
    lcd.print(CurTurns);
  } else if (strcmp(iPlaceholder, "#S#") == 0) {
    lcd.print("Motor Speed:    ");
    lcd.setCursor(13, 1);
    lcd.print(CurSpeed);
  } else if (strcmp(iPlaceholder, "#D#") == 0) {
    lcd.print("Direction:      ");
    lcd.setCursor(13, 1);
    lcd.print(CurDirection);
  } else if (strcmp(iPlaceholder, "#K#") == 0) {
    lcd.print("Shutter Key:    ");
    lcd.setCursor(13, 1);
    lcd.print(CurKey2Send);
  } else if (strcmp(iPlaceholder, "#B#") == 0) {
    lcd.print("Delay Before:   ");
    lcd.setCursor(13, 1);
    lcd.print(CurDelayBefore);
  } else if (strcmp(iPlaceholder, "#A#") == 0) {
    lcd.print("Delay After:    ");
    lcd.setCursor(13, 1);
    lcd.print(CurDelayAfter);
  } else if (strcmp(iPlaceholder, "#Z#") == 0) {
    lcd.print("Step size:      ");
    lcd.setCursor(13, 1);
    lcd.print(CurStepSize);
  } else {
    lcd.print(iPlaceholder);
  }
}

//============================================================================================
// placeholderInc - increment the variable using its placeholder
//============================================================================================
void placeholderInc(char* iPlaceholder) {

  if (strcmp(iPlaceholder, "#P#") == 0) {
    CurPhotos += 2;  // step by 2 at a time
    if (CurPhotos > MAX_PHOTOS) CurPhotos = MAX_PHOTOS;
    if (FastChng == 1) delay(FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#T#") == 0) {
    CurTurns++;
    if (CurTurns > MAX_TURNS) CurTurns = MAX_TURNS;
    if (FastChng == 1) delay(FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#S#") == 0) {
    CurSpeed++;
    if (CurSpeed > MAX_SPEED) CurSpeed = MAX_SPEED;
    if (FastChng == 1) delay(SLOW_FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#K#") == 0) {
    CurKey2Send++;
    if (CurKey2Send > MAX_KEY) CurKey2Send = MAX_KEY;
    if (FastChng == 1) delay(SLOW_FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#B#") == 0) {
    CurDelayBefore++;
    if (CurDelayBefore > MAX_BEFORE) CurDelayBefore = MAX_BEFORE;
    if (FastChng == 1) delay(SLOW_FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#A#") == 0) {
    CurDelayAfter++;
    if (CurDelayAfter > MAX_AFTER) CurDelayAfter = MAX_AFTER;
    if (FastChng == 1) delay(SLOW_FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#Z#") == 0) {
    CurStepSize++;
    if (CurStepSize > MAX_SIZE) CurStepSize = MAX_SIZE;
    if (FastChng == 1) delay(FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#D#") == 0) {
    if (CurDirection == 'L') CurDirection = 'R';
    else if (CurDirection == 'R') CurDirection = 'L';
    if (FastChng == 1) delay(SLOW_FASTCHANGEDELAY);
  }
}

//============================================================================================
// placeholderDec - decrement the variable using its placeholder
//============================================================================================
void placeholderDec(char* iPlaceholder) {

  if (strcmp(iPlaceholder, "#P#") == 0) {
    CurPhotos -= 2;  // step by 2 at a time
    if (CurPhotos < MIN_PHOTOS) CurPhotos = MIN_PHOTOS;
    if (FastChng == -1) delay(FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#T#") == 0) {
    CurTurns--;
    if (CurTurns < MIN_TURNS) CurTurns = MIN_TURNS;
    if (FastChng == -1) delay(FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#S#") == 0) {
    CurSpeed--;
    if (CurSpeed < MIN_SPEED) CurSpeed = MIN_SPEED;
    if (FastChng == -1) delay(SLOW_FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#K#") == 0) {
    CurKey2Send--;
    if (CurKey2Send < MIN_KEY) CurKey2Send = MIN_KEY;
    if (FastChng == -1) delay(SLOW_FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#B#") == 0) {
    CurDelayBefore--;
    if (CurDelayBefore < MIN_BEFORE) CurDelayBefore = MIN_BEFORE;
    if (FastChng == -1) delay(SLOW_FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#A#") == 0) {
    CurDelayAfter--;
    if (CurDelayAfter < MIN_AFTER) CurDelayAfter = MIN_AFTER;
    if (FastChng == -1) delay(SLOW_FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#Z#") == 0) {
    CurStepSize--;
    if (CurStepSize < MIN_SIZE) CurStepSize = MIN_SIZE;
    if (FastChng == -1) delay(FASTCHANGEDELAY);
  } else if (strcmp(iPlaceholder, "#D#") == 0) {
    if (CurDirection == 'L') CurDirection = 'R';
    else if (CurDirection == 'R') CurDirection = 'L';
    if (FastChng == 1) delay(SLOW_FASTCHANGEDELAY);
  }
}

//============================================================================================
// showMenu
//============================================================================================
// Generic routine to display menu and return the choice made
// The menuItems to display are passed using an array
// The first entry of the array is the name of the menu
//============================================================================================
int showMenu(int iItemsCount, char iMenuItems[][MENUITEM_LENGTH]) {

  int MenuPos = 1;

  Serial.println("Entering showMenu");
  Serial.print("Menu to display:");
  Serial.println(iMenuItems[0]);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(iMenuItems[0]);
  lcd.setCursor(0, 1);
  if (iMenuItems[MenuPos][0] == '#') {
    placeholderPrint(iMenuItems[MenuPos]);
  } else lcd.print(iMenuItems[MenuPos]);
  readInputs();
  while (!keyPressed()) {
    if (!noDirection()) {
      if (leftPressed()) {
        MenuPos++;
        if (MenuPos > iItemsCount - 1) MenuPos = 1;
        lcd.setCursor(0, 1);
        if (iMenuItems[MenuPos][0] == '#') {
          placeholderPrint(iMenuItems[MenuPos]);
        } else lcd.print(iMenuItems[MenuPos]);
      }
      if (rightPressed()) {
        MenuPos--;
        if (MenuPos < 1) MenuPos = iItemsCount - 1;
        lcd.setCursor(0, 1);
        if (iMenuItems[MenuPos][0] == '#') {
          placeholderPrint(iMenuItems[MenuPos]);
        } else lcd.print(iMenuItems[MenuPos]);
      }
      if (upPressed() || FastChng == 1) {
        if (iMenuItems[MenuPos][0] == '#') {
          placeholderInc(iMenuItems[MenuPos]);
          placeholderPrint(iMenuItems[MenuPos]);
        }
      }
      if (downPressed() || FastChng == -1) {
        if (iMenuItems[MenuPos][0] == '#') {
          placeholderDec(iMenuItems[MenuPos]);
          placeholderPrint(iMenuItems[MenuPos]);
        }
      }
    }
    readInputs();
  }
  return (MenuPos);
}

//============================================================================================
// takePhoto - send the correct key to the smartphone over BLE to take a picture
//============================================================================================
void takePhoto() {
  if (myKB.isConnected()) {
    if (CurKey2Send == 2) {
      Serial.println("Sending Volume Up key for IOS devices ...");
      myKB.write(KEY_MEDIA_VOLUME_UP);
    } else {
      Serial.println("Sending Volume Down key for Android devices ...");
      myKB.write(KEY_MEDIA_VOLUME_DOWN);
    }
  }
}

//============================================================================================
// photoDialog - the dialog of the Photogrammetric mode
//============================================================================================
void photoDialog() {
  int MenuChoice = 0;
  while (MenuChoice != 6) {
    MenuChoice = showMenu(7, PhotoMenu);

    switch (MenuChoice) {
      case (5):
        {
          // Start Photogrammetric mode
          // Show number of photos on LCD
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Running...");
          myStepper.setSpeed(CurSpeed);  //Set RPM of steppermotor
          //Calculate number of steps between photo's
          int numSteps = FullRev / CurPhotos;
          // Determine direction
          if (CurDirection == 'L') numSteps = -numSteps;

          int i = 1;
          do {
            lcd.setCursor(0, 1);
            lcd.print("Photo ");
            lcd.print(i);
            lcd.print(" of ");
            lcd.print(CurPhotos);
            myStepper.step(numSteps);
            delay(CurDelayBefore * 1000);
            takePhoto();
            delay(CurDelayAfter * 1000);
            i++;
          } while (i <= CurPhotos);
          digitalWrite(16, LOW);  // logic level is low
          digitalWrite(17, LOW);  // logic level is low
          break;
        }
      default:
        {
          break;
        }
    }
  }
  return;
}

//============================================================================================
// cineDialog - the dialog of the Cinematic mode
//============================================================================================
void cineDialog() {
  int MenuChoice = 0;
  while (MenuChoice != 4) {
    MenuChoice = showMenu(5, CineMenu);

    switch (MenuChoice) {
      case (3):
        {
          // Start Cinematic mode
          // Show number of turns on LCD
          int i = 1;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Running...");
          myStepper.setSpeed(CurSpeed);  //Set RPM of steppermotor
          int numSteps = FullRev;
          // Determine direction
          if (CurDirection == 'L') numSteps = -numSteps;
          do {
            lcd.setCursor(0, 1);
            lcd.print("Turn ");
            lcd.print(i);
            lcd.print(" of ");
            lcd.print(CurTurns);
            myStepper.step(numSteps);
            i++;
          } while (i <= CurTurns);
          digitalWrite(16, LOW);  // logic level is low
          digitalWrite(17, LOW);  // logic level is low
          break;
        }
      default:
        {
          break;
        }
    }
  }
  return;
}

//============================================================================================
// manualDialog - the dialog of the Manual mode
//============================================================================================
void manualDialog() {
  int MenuChoice = 0;
  while (MenuChoice != 4) {
    MenuChoice = showMenu(5, ManualMenu);

    switch (MenuChoice) {
      case (3):
        {
          // Start Manual mode
          // Show instructions on LCD
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Press to leave.");
          myStepper.setSpeed(CurSpeed);  //Set RPM of steppermotor
          do {
            readInputs();
            lcd.setCursor(0, 1);
            lcd.print("<- direction ->");
            // Move in the desired direction
            if (rightPressed()) {
              myStepper.step(-CurStepSize * 100);
            }
            if (leftPressed()) {
              myStepper.step(CurStepSize * 100);
            }
          } while (!keyPressed());
          digitalWrite(16, LOW);  // logic level is low
          digitalWrite(17, LOW);  // logic level is low
          break;
        }
      default:
        {
          break;
        }
    }
  }
  return;
}

//============================================================================================
// defaultsDialog - the dialog of the operation defaults ( saving to EEPROM )
//============================================================================================
void defaultsDialog() {
  int MenuChoice = 0;
  while (MenuChoice != 10) {
    MenuChoice = showMenu(11, DefaultsMenu);

    switch (MenuChoice) {
      case (9):
        {
          //Save the defaults to EEPROM
          MyDefaults.DefaultTurns = CurTurns;
          MyDefaults.DefaultPhotos = CurPhotos;
          MyDefaults.DefaultSpeed = CurSpeed;
          MyDefaults.DefaultStepSize = CurStepSize;
          MyDefaults.DefaultKey2Send = CurKey2Send;
          MyDefaults.DefaultDelayBefore = CurDelayBefore;
          MyDefaults.DefaultDelayAfter = CurDelayAfter;
          MyDefaults.DefaultDirection = CurDirection;
          EEPROM.put(eeAddress, MyDefaults);
          EEPROM.commit();
          break;
        }
      default:
        {
          break;
        }
    }
  }
  return;
}

//============================================================================================
// mainMenu - the main menu dialog
//============================================================================================
void mainMenu() {
  int MenuChoice;

  MenuChoice = showMenu(5, MainMenu);

  switch (MenuChoice) {
    case (1):
      {
        photoDialog();
        break;
      }
    case (2):
      {
        cineDialog();
        break;
      }
    case (3):
      {
        manualDialog();
        break;
      }
    case (4):
      {
        defaultsDialog();
        break;
      }
  }
}

//============================================================================================
// loop - the main loop
//============================================================================================
void loop() {
  CurTurns = MyDefaults.DefaultTurns;
  CurSpeed = MyDefaults.DefaultSpeed;
  CurPhotos = MyDefaults.DefaultPhotos;
  CurStepSize = MyDefaults.DefaultStepSize;
  CurKey2Send = MyDefaults.DefaultKey2Send;
  CurDelayBefore = MyDefaults.DefaultDelayBefore;
  CurDelayAfter = MyDefaults.DefaultDelayAfter;
  CurDirection = MyDefaults.DefaultDirection;
  myStepper.setSpeed(CurSpeed);  //Set RPM of steppermotor
  mainMenu();
}
