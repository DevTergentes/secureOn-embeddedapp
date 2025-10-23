#include <Arduino.h>
#include "DisplayActuator.h"
#include <LiquidCrystal_I2C.h>

/**
 * DisplayActuator.cpp
 *
 * This class is responsible for controlling the display actuator.
 * @see DisplayActuator.h
 * @method init() - Initializes the display actuator.
 * @method setCursor(int row, int column) - Sets the cursor to the specified row and column.
 * @method printDistance(float distance) - Prints the distance to the display.
 * @method printText(String text) - Prints the specified text to the display.
 * @method clear() - Clears the display.
 */

// Define the LCD I2C address and dimensions
LiquidCrystal_I2C lcd(0x27, 20, 4);

/**
 * Constructor
 * Initializes the display actuator.
 */
DisplayActuator::DisplayActuator() {}

/**
 * Initializes the display actuator.
 */
void DisplayActuator::init() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.print("Sensor-RitmoCardiaco");
  delay(3000);
  lcd.clear();
  lcd.print("Ready");
  delay(3000);
  //lcd.clear();
}

/**
 * Sets the cursor to the specified row and column.
 * @param row - The row to set the cursor to.
 * @param column - The column to set the cursor to.
 */
void DisplayActuator::setCursor(int row, int column) {
  lcd.setCursor(row, column);
}

/**
 * Prints the distance to the display.
 * @param distance - The distance to print.
 */
void DisplayActuator::printValue(int value) {
  delay(2000);
  lcd.print(value);
}

/**
 * Prints the specified text to the display.
 * @param text - The text to print.
 */
void DisplayActuator::printText(String text) {
  lcd.print(text);
}

/**
 * Clears the display.
 */
void DisplayActuator::clear() {
  lcd.clear();
}
