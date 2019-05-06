/**
 * LCD Auton Script Selector
 */
#ifndef _LCD_SELECTOR_H_
#define _LCD_SELECTOR_H_

#include "main.h"
#include "scripts.hpp"

namespace lcdSelector {
extern std::vector<std::function<void(auton::color, uint32_t, uint32_t)>>
    scripts;
extern std::vector<std::function<void(auton::color)>> inits;
extern std::vector<std::string> titles;

/**
 * Initializes the LCD.
 */
void init();

/**
 * Runs the LCD Script Selection process.
 *
 * Logs a message with the "L," prefix.
 */
void select();

/**
 * Runs the selected autonomous script. Nothing will run if a script was not
 * selected.
 */
void executeScript();

/**
 * Runs the initialize routine for the selected autonomous script.
 */
void executeInit();

/**
 * Gets the initialization state of the chosen script.
 *
 * \return True if the script's initialization has been run.
 */
bool isScriptInitialized();

/**
 * Gets the auton::color that was selected.
 *
 * \return The auton::color that was selected.
 */
auton::color getSide();

std::string getScriptName();

}  // namespace lcdSelector

#endif
