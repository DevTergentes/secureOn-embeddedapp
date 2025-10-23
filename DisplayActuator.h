#ifndef DisplayActuator_h
#define DisplayActuator_h

/**
 * DisplayActuator.h
 *
 * This class is responsible for controlling the display actuator.
 * @see DisplayActuator.cpp
 * @method init() - Initializes the display actuator.
 * @method setCursor(int row, int column) - Sets the cursor to the specified row and column.
 * @method printDistance(float distance) - Prints the distance to the display.
 * @method printText(String text) - Prints the specified text to the display.
 * @method clear() - Clears the display.
 */

class DisplayActuator {
  public:
    DisplayActuator();
    void init();
    void setCursor(int row, int column);
    void printValue(int value);
    void printText(String text);
    void clear();
};

#endif