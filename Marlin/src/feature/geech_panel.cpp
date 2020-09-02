/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * geech_panel.cpp - support for geech's i2c control panel
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(GEECH_PANEL)

#include "geech_panel.h"
#include "../MarlinCore.h"
#include "../core/serial.h"
#include <Adafruit_MCP23017.h>

#include "../lcd/ultralcd.h"
#include "../lcd/menu/menu.h"

// MCP23017 number 1
//
#define X_LEFT_BUTTON 0
// Pins 1,2,3 UNUSED
#define ONE_MM_BUTTON 4
#define ONE_MM_LED 5
#define ONE_TENTH_MM_BUTTON 6
#define ONE_TENTH_MM_LED 7
#define STOP_BUTTON 8
#define STOP_LED 9
#define MENU_DOWN_BUTTON 10
#define PAUSE_RESUME_BUTTON 11
#define PRINT_BUTTON 12
#define MENU_UP_BUTTON 13
#define MENU_BACK_BUTTON 14
#define MENU_ENTER_BUTTON 15

// MCP23017 number 2
//
// Pins 0,1 UNUSED
#define Z_UP_BUTTON (2 | 0x10)
#define Z_HOME_BUTTON (3 | 0x10)
#define Z_DOWN_BUTTON (4 | 0x10)
#define TEN_MM_BUTTON (5 | 0x10)
#define ONE_HUNDRED_MM_LED (6 | 0x10)
#define ONE_HUNDRED_MM_BUTTON (7 | 0x10)
#define TEN_MM_LED (8 | 0x10)
#define Y_DOWN_BUTTON (9 | 0x10)
#define XY_HOME_BUTTON (10 | 0x10)
#define X_RIGHT_BUTTON (11 | 0x10)
#define Y_UP_BUTTON (12 | 0x10)
// Pins 14,15 UNUSED

#define EXPANDER_2_FLAG 0x10
#define PIN_MASK 0x0F
#define EXPANDER_MASK 0xF0

GeechPanel geechPanel;

Adafruit_MCP23017 expander1;
Adafruit_MCP23017 expander2;
millis_t nextKillButtonToggle = 0;
bool killButtonState = false;
bool killButtonToggleEnabled = false;
uint8_t currentDistanceSetting = 255;
uint8_t waitingForButtonRelease = 255;
uint8_t lastGPIOState = 0;

Adafruit_MCP23017 *expanderForPin(uint8_t pin)
{
  Adafruit_MCP23017 *expander = &expander1;
  if ((pin & EXPANDER_MASK) > 0)
  {
    expander = &expander2;
  }

  return expander;
}

void expanderPinWrite(uint8_t pin, uint8_t value)
{
  Adafruit_MCP23017 *expander = expanderForPin(pin);
  pin &= PIN_MASK;

  expander->digitalWrite(pin, value);
}

void updateDistanceSetting(uint8_t newSetting)
{
  if (currentDistanceSetting != 255)
  {
    expanderPinWrite(currentDistanceSetting, 0);
  }
  currentDistanceSetting = newSetting;
  if (currentDistanceSetting != 255 && !printingIsActive())
  {
    expanderPinWrite(currentDistanceSetting, 1);
  }
}

void setupExpanderPin(uint8_t pin, uint8_t direction)
{
  Adafruit_MCP23017 *expander = expanderForPin(pin);
  pin = pin & PIN_MASK;

  if (direction == INPUT)
  {
    expander->pinMode(pin, INPUT);
    expander->pullUp(pin, 1);
    expander->setupInterruptPin(pin, CHANGE);
  }
  else
  {
    expander->pinMode(pin, OUTPUT);
  }
}

void blinkAllLEDs(millis_t duration)
{
  // Turn all LEDs on for a moment, then off
  expander1.digitalWrite(ONE_MM_LED, 1);
  expanderPinWrite(ONE_TENTH_MM_LED, 1);
  expanderPinWrite(STOP_LED, 1);
  expanderPinWrite(ONE_HUNDRED_MM_LED, 1);
  expanderPinWrite(TEN_MM_LED, 1);

  safe_delay(duration);

  expanderPinWrite(ONE_MM_LED, 0);
  expanderPinWrite(ONE_TENTH_MM_LED, 0);
  expanderPinWrite(STOP_LED, 0);
  expanderPinWrite(ONE_HUNDRED_MM_LED, 0);
  expanderPinWrite(TEN_MM_LED, 0);
}

void GeechPanel::init()
{
  // SET_INPUT(32);
  // lastGPIOState = READ(32);
  //pinMode(32, INPUT);
  //lastGPIOState = digitalRead(32);

  //
  expander1.begin(GP_MCP23017_1_ADDRESS);

  setupExpanderPin(X_LEFT_BUTTON, INPUT);
  setupExpanderPin(ONE_MM_BUTTON, INPUT);
  setupExpanderPin(ONE_TENTH_MM_BUTTON, INPUT);
  setupExpanderPin(STOP_BUTTON, INPUT);
  setupExpanderPin(MENU_DOWN_BUTTON, INPUT);
  setupExpanderPin(PAUSE_RESUME_BUTTON, INPUT);
  setupExpanderPin(PRINT_BUTTON, INPUT);
  setupExpanderPin(MENU_UP_BUTTON, INPUT);
  setupExpanderPin(MENU_BACK_BUTTON, INPUT);
  setupExpanderPin(MENU_ENTER_BUTTON, INPUT);

  setupExpanderPin(ONE_MM_LED, OUTPUT);
  setupExpanderPin(ONE_TENTH_MM_LED, OUTPUT);
  setupExpanderPin(STOP_LED, OUTPUT);
  setupExpanderPin(1, OUTPUT);
  setupExpanderPin(2, OUTPUT);
  setupExpanderPin(3, OUTPUT);

  expander1.setupInterrupts(1, 1, 0);

  expander2.begin(GP_MCP23017_2_ADDRESS);

  setupExpanderPin(Z_UP_BUTTON, INPUT);
  setupExpanderPin(Z_HOME_BUTTON, INPUT);
  setupExpanderPin(Z_DOWN_BUTTON, INPUT);
  setupExpanderPin(TEN_MM_BUTTON, INPUT);
  setupExpanderPin(ONE_HUNDRED_MM_BUTTON, INPUT);
  setupExpanderPin(Y_DOWN_BUTTON, INPUT);
  setupExpanderPin(XY_HOME_BUTTON, INPUT);
  setupExpanderPin(X_RIGHT_BUTTON, INPUT);
  setupExpanderPin(Y_UP_BUTTON, INPUT);

  setupExpanderPin(ONE_HUNDRED_MM_LED, OUTPUT);
  setupExpanderPin(TEN_MM_LED, OUTPUT);
  setupExpanderPin(0 | 0x10, OUTPUT);
  setupExpanderPin(1 | 0x10, OUTPUT);
  setupExpanderPin(14 | 0x10, OUTPUT);
  setupExpanderPin(15 | 0x10, OUTPUT);

  expander2.setupInterrupts(1, 1, 0);

  blinkAllLEDs(250);
}

uint8_t getLastInterruptPin()
{
  uint8_t changedPin = expander1.getLastInterruptPin();
  if (changedPin != MCP23017_INT_ERR)
  {
    return changedPin;
  }

  changedPin = expander2.getLastInterruptPin();
  if (changedPin != MCP23017_INT_ERR)
  {
    return changedPin | EXPANDER_2_FLAG;
  }

  return MCP23017_INT_ERR;
}

uint8_t readExpander1()
{
  static uint16_t previousBothPorts = 0;

  uint16_t bothPorts = expander1.readGPIOAB();
  // buttons are active low, so invert what we get from the expander
  bothPorts = ~bothPorts;
  // mask out the output pins, we don't care about those.
  bothPorts = bothPorts & 0xFD51; // 0b1111110101010001;

  if (bothPorts == previousBothPorts)
  {
    return 255;
  }

  previousBothPorts = bothPorts;

  if (bothPorts == 0)
  {
    return 255;
  }

  uint8_t result = 0;
  //SERIAL_ECHOPGM("expander 1: ");
  for (int i = 0; i < 16; i++)
  {
    if ((bothPorts >> i) & 0x0001)
    {
      result = i;
      //SERIAL_CHAR('1');
    }
    else
    {
      //SERIAL_CHAR('0');
    }
  }

  //SERIAL_EOL();

  return result;
}

uint8_t readExpander2()
{
  static uint16_t previousBothPorts = 0;

  uint16_t bothPorts = expander2.readGPIOAB();
  bothPorts = ~bothPorts;
  // mask out the output pins, we don't care about those.
  bothPorts = bothPorts & 0b001111010111100;

  if (bothPorts == previousBothPorts)
  {
    return 255;
  }

  previousBothPorts = bothPorts;

  if (bothPorts == 0)
  {
    return 255;
  }

  uint8_t result = 0;
  for (int i = 0; i < 16; i++)
  {
    if ((bothPorts >> i) & 0x0001)
    {
      result = i | EXPANDER_2_FLAG;
      //SERIAL_CHAR('1');
    }
    else
    {
      //SERIAL_CHAR('0');
    }
  }

  //SERIAL_EOL();

  return result;
}

uint8_t checkForButtonPress()
{
  uint8_t buttonPress = readExpander1();
  if (buttonPress != 255)
  {
    return buttonPress;
  }
  return readExpander2();
}

void handleButtonPress(uint8_t buttonPressed)
{
  switch (buttonPressed)
  {
  case MENU_ENTER_BUTTON:
    //SERIAL_ECHOLNPGM("menu enter");
    ui.lcd_clicked = !wait_for_user;
    wait_for_user = false;
    ui.quick_feedback();
    break;
  case MENU_BACK_BUTTON:
    //SERIAL_ECHOLNPGM("menu back");
    MenuItem_back::action();
    ui.quick_feedback();
    break;
  case MENU_DOWN_BUTTON:
    //SERIAL_ECHOLNPGM("menu down");
    encoderDiff = (ENCODER_STEPS_PER_MENU_ITEM) * (ENCODER_PULSES_PER_STEP);
    break;
  case MENU_UP_BUTTON:
    //SERIAL_ECHOLNPGM("menu up");
    encoderDiff = (ENCODER_STEPS_PER_MENU_ITEM) * (ENCODER_PULSES_PER_STEP) * -1;
    break;
  case PRINT_BUTTON:
//SERIAL_ECHOLNPGM("print");
#if ENABLED(SDSUPPORT)
    if (card.isMounted())
    {
      ui.goto_screen(menu_media);
    }
    else
    {
      ui.set_status("No media");
      ui.chirp();
    }
#else
#error No SDSUPPORT == No print from SD card
#endif
    break;
  case PAUSE_RESUME_BUTTON:
#if ENABLED(SDSUPPORT)
    if (printingIsActive())
    {
      SERIAL_ECHOLNPGM("pause");
      ui.pause_print();
      ui.quick_feedback();
    }
    else if (printingIsPaused())
    {
      SERIAL_ECHOLNPGM("resume");
      ui.resume_print();
      ui.quick_feedback();
    }
    else
    {
      SERIAL_ECHOLNPGM("nothing doing");
      ui.set_status("Not printing");
      ui.chirp();
    }
#else
#error No pause
#endif
    break;
  case X_LEFT_BUTTON:
    SERIAL_ECHOLNPGM("X Left");
    break;
  case X_RIGHT_BUTTON:
    SERIAL_ECHOLNPGM("X Right");
    break;
  case Y_UP_BUTTON:
    SERIAL_ECHOLNPGM("Y Up");
    break;
  case Y_DOWN_BUTTON:
    SERIAL_ECHOLNPGM("Y Down");
    break;
  case XY_HOME_BUTTON:
    SERIAL_ECHOLNPGM("XY Home");
    break;
  case Z_UP_BUTTON:
    SERIAL_ECHOLNPGM("Z Up");
    break;
  case Z_DOWN_BUTTON:
    SERIAL_ECHOLNPGM("Z Down");
    break;
  case Z_HOME_BUTTON:
    SERIAL_ECHOLNPGM("Z Home");
    break;
  case STOP_BUTTON:
    kill();
    break;
  case ONE_TENTH_MM_BUTTON:
    SERIAL_ECHOLNPGM(".1 MM");
    updateDistanceSetting(ONE_TENTH_MM_LED);
    break;
  case ONE_MM_BUTTON:
    SERIAL_ECHOLNPGM("1 MM");
    updateDistanceSetting(ONE_MM_LED);
    break;
  case TEN_MM_BUTTON:
    SERIAL_ECHOLNPGM("10 MM");
    updateDistanceSetting(TEN_MM_LED);
    break;
  case ONE_HUNDRED_MM_BUTTON:
    SERIAL_ECHOLNPGM("100 MM");
    updateDistanceSetting(ONE_HUNDRED_MM_LED);
    break;
  default:
    break;
  }
}

void updateButtonLEDs()
{
  static millis_t nextButtonLEDUpdate = 0;
  static bool lastPrintingState = false;

  millis_t ms = millis();

  if (!ELAPSED(ms, nextButtonLEDUpdate))
  {
    return;
  }
  nextButtonLEDUpdate = ms + 100;

  bool printingActive = printingIsActive();
  if (printingActive != lastPrintingState)
  {
    lastPrintingState = printingActive;
    if (printingActive)
    {
      updateDistanceSetting(255);
      killButtonToggleEnabled = true;
      nextKillButtonToggle = 0;
      killButtonState = false;
    }
    else
    {
      killButtonToggleEnabled = false;
    }
  }

  if (ELAPSED(ms, nextKillButtonToggle))
  {
    if (killButtonState && !killButtonToggleEnabled)
    {
      killButtonState = false;
      expanderPinWrite(STOP_LED, false);
    }
    else if (killButtonToggleEnabled)
    {
      nextKillButtonToggle = ms + 500;
      killButtonState = !killButtonState;
      expanderPinWrite(STOP_LED, killButtonState);
    }
  }
}

void GeechPanel::update()
{
  updateButtonLEDs();

  uint8_t buttonPressed = checkForButtonPress();
  if (buttonPressed != 255)
  {
    handleButtonPress(buttonPressed);
  }
}

#endif // GEECH_PANEL