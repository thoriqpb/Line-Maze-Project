#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Button Pins
#define BUTTON_UP 12
#define BUTTON_DOWN 16
#define BUTTON_OK 17
#define BUTTON_CANCEL 9

// Buzzer Pin
#define BUZZER_PIN 8

// Menu states
enum MenuState { MAIN_MENU, MAZE_SUB_MENU, ABOUT_SCREEN };
MenuState menuState = MAIN_MENU;

// Menu items
const char* mainMenuItems[] = {"Line Follower Mode", "Line Maze Mode", "About"};
const int mainMenuCount = 3;

const char* mazeSubMenu[] = {"Search Path", "Play Path", "Reset Path"};
const int mazeSubMenuCount = 3;

int selectedItem = 0;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 300;

void setup() {
  Serial.begin(9600);

  pinMode(BUZZER_PIN, OUTPUT);
  startupBeep();

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

// Buzzer functions
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

// Button handling
void handleButtons() {
  if ((millis() - lastDebounceTime) < debounceDelay) return;

  if (menuState == ABOUT_SCREEN) {
    if (digitalRead(BUTTON_CANCEL) == LOW || digitalRead(BUTTON_OK) == LOW) {
      menuState = MAIN_MENU;
      selectedItem = 0;
      shortBeep();
      lastDebounceTime = millis();
    }
    return;
  }

  if (digitalRead(BUTTON_UP) == LOW) {
    selectedItem--;
    int maxItems = (menuState == MAIN_MENU) ? mainMenuCount - 1 :
                   (menuState == MAZE_SUB_MENU) ? mazeSubMenuCount - 1 : 0;
    if (selectedItem < 0) selectedItem = maxItems;
    shortBeep();
    lastDebounceTime = millis();
  } 
  else if (digitalRead(BUTTON_DOWN) == LOW) {
    selectedItem++;
    int maxItems = (menuState == MAIN_MENU) ? mainMenuCount - 1 :
                   (menuState == MAZE_SUB_MENU) ? mazeSubMenuCount - 1 : 0;
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
    if (menuState != MAIN_MENU) {
      menuState = MAIN_MENU;
      selectedItem = 0;
    }
    lastDebounceTime = millis();
  }
}

// OK button handling
void handleOK() {
  switch (menuState) {
    case MAIN_MENU:
      switch (selectedItem) {
        case 0: // Line Follower Mode
          lineFollowerMode();
          break;
        case 1: // Line Maze Mode
          menuState = MAZE_SUB_MENU;
          selectedItem = 0;
          break;
        case 2: // About
          menuState = ABOUT_SCREEN;
          break;
      }
      break;

    case MAZE_SUB_MENU:
      switch (selectedItem) {
        case 0: searchPath(); break;
        case 1: playPath(); break;
        case 2: resetPath(); break;
      }
      menuState = MAIN_MENU;
      selectedItem = 0;
      break;

    case ABOUT_SCREEN:
      menuState = MAIN_MENU;
      selectedItem = 0;
      break;
  }
}

// Drawing the menu
void drawMenu() {
  display.clearDisplay();

  if (menuState == MAIN_MENU) {
    drawCenteredMenu(mainMenuItems, mainMenuCount, "Main Menu");
  } 
  else if (menuState == MAZE_SUB_MENU) {
    drawCenteredMenu(mazeSubMenu, mazeSubMenuCount, "Line Maze");
  }
  else if (menuState == ABOUT_SCREEN) {
    showAbout();
  }

  display.display();
}

void drawCenteredMenu(const char* items[], int count, const char* title) {
  int16_t x, y;
  uint16_t w, h;

  display.getTextBounds(title, 0, 0, &x, &y, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 2);
  display.print(title);
  display.drawFastHLine(0, 12, SCREEN_WIDTH, SSD1306_WHITE);

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

// About screen
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

// Action functions
void lineFollowerMode() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Starting Line Follower...");
  display.display();
  delay(1000);
}

void searchPath() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Searching path...");
  display.display();
  delay(1000);
}

void playPath() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Playing path...");
  display.display();
  delay(1000);
}

void resetPath() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Resetting path...");
  display.display();
  delay(1000);
}
