#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Button Pins
#define BUTTON_UP 12
#define BUTTON_DOWN 11
#define BUTTON_OK 10
#define BUTTON_CANCEL 9

// Buzzer Pin
#define BUZZER_PIN 8

enum MenuState { MAIN_MENU, SUB_MENU, ABOUT_SCREEN };
MenuState menuState = MAIN_MENU;

const char* mainMenuItems[] = {"Search Path", "Play Path", "Reset Path", "About"};
const char* searchSubMenu[] = {"Right mode search", "Left mode search"};

int selectedItem = 0;
const int mainMenuCount = 4;
const int subMenuCount = 2;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 300;

void setup() {
  Serial.begin(9600);

  // Initialize buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  startupBeep();  // Startup sound

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_OK, INPUT_PULLUP);
  pinMode(BUTTON_CANCEL, INPUT_PULLUP);
}

void loop() {
  handleButtons();
  drawMenu();
}

// Buzzer control functions
void shortBeep() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(10);
  digitalWrite(BUZZER_PIN, LOW);
}

void startupBeep() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }
}

void handleButtons() {
  if ((millis() - lastDebounceTime) < debounceDelay) return;

  if (menuState == ABOUT_SCREEN) {
    if (digitalRead(BUTTON_CANCEL) == LOW || digitalRead(BUTTON_OK) == LOW) {
      menuState = MAIN_MENU;
      shortBeep();
      lastDebounceTime = millis();
    }
    return;
  }

  if (digitalRead(BUTTON_UP) == LOW) {
    selectedItem--;
    if (selectedItem < 0) selectedItem = (menuState == MAIN_MENU) ? mainMenuCount - 1 : subMenuCount - 1;
    shortBeep();
    lastDebounceTime = millis();
  }
  else if (digitalRead(BUTTON_DOWN) == LOW) {
    selectedItem++;
    int maxItems = (menuState == MAIN_MENU) ? mainMenuCount - 1 : subMenuCount - 1;
    if (selectedItem > maxItems) selectedItem = 0;
    shortBeep();
    lastDebounceTime = millis();
  }
  else if (digitalRead(BUTTON_OK) == LOW) {
    shortBeep();
    handleOK();
    lastDebounceTime = millis();
  }
  else if (digitalRead(BUTTON_CANCEL) == LOW) {
    shortBeep();
    if (menuState == SUB_MENU) {
      menuState = MAIN_MENU;
      selectedItem = 0;
    }
    lastDebounceTime = millis();
  }
}

// Remaining functions unchanged from original code...
// [Keep all other functions (handleOK, drawMenu, drawCenteredMenu, showAbout, etc.) exactly as they were]
// ... (Rest of the code remains unchanged)

void handleOK() {
  if (menuState == MAIN_MENU) {
    switch(selectedItem) {
      case 0: 
        menuState = SUB_MENU;
        selectedItem = 0;
        break;
      case 1: playPath(); break;
      case 2: resetPath(); break;
      case 3: 
        menuState = ABOUT_SCREEN;
        break;
    }
  }
  else if (menuState == SUB_MENU) {
    switch(selectedItem) {
      case 0: rightModeSearch(); break;
      case 1: leftModeSearch(); break;
    }
    menuState = MAIN_MENU;
  }
}

void drawMenu() {
  display.clearDisplay();
  
  if (menuState == MAIN_MENU) {
    drawCenteredMenu(mainMenuItems, mainMenuCount, "Main Menu");
  } 
  else if (menuState == SUB_MENU) {
    drawCenteredMenu(searchSubMenu, subMenuCount, "Search Mode");
  }
  else if (menuState == ABOUT_SCREEN) {
    showAbout();
  }
  
  display.display();
}

void drawCenteredMenu(const char* items[], int count, const char* title) {
  int16_t x, y;
  uint16_t w, h;
  
  // Draw title with underline
  display.getTextBounds(title, 0, 0, &x, &y, &w, &h);
  display.setCursor((SCREEN_WIDTH - w)/2, 2);
  display.print(title);
  display.drawFastHLine(0, 12, SCREEN_WIDTH, SSD1306_WHITE);

  // Menu items positioning
  int startY = (SCREEN_HEIGHT - (count * 10)) / 2;
  if (startY < 16) startY = 16;

  for (int i = 0; i < count; i++) {
    display.getTextBounds(items[i], 0, 0, &x, &y, &w, &h);
    int16_t xPos = (SCREEN_WIDTH - w) / 2;
    int16_t yPos = startY + (i * 10);

    if (i == selectedItem) {
      display.fillRoundRect(xPos - 4, yPos - 2, w + 8, h + 4, 2, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    
    display.setCursor(xPos, yPos);
    display.print(items[i]);
  }
  display.setTextColor(SSD1306_WHITE);
}

void showAbout() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setCursor(0, 2);
  display.println("     Team Member  ");
  display.println("Thoriq Putra Belligan");
  display.println("Fajari Ani Novita S");
  display.println("Muhammad Harits N K");
  display.println("Muhammad Adib Elfito");
  display.println();
  display.println("Press Back to Return");
}

void rightModeSearch() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Starting Right mode");
  display.display();
  delay(1000);
}

void leftModeSearch() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Starting Left mode");
  display.display();
  delay(1000);
}

void playPath() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Playing path...");
  display.display();
  delay(1000);
}

void resetPath() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Resetting path...");
  display.display();
  delay(1000);
}
