#include "main.h"

#define SCREEN_LOOP_DELAY 250

static int scriptNumber;
static auton::color side;
static uint32_t allianceFireTime, centerFireTime;

static bool runInit = false;

using namespace pros;

namespace lcdSelector {
std::vector<std::function<void(auton::color, uint32_t, uint32_t)>> scripts;
std::vector<std::function<void(auton::color)>> inits;
std::vector<std::string> titles;

void init() { lcd::initialize(); }

void select() {
  lcd::print(0, "Please configure an autonomous routine");
  lcd::print(1, "Script: %s", titles[0]);
  lcd::print(2, "Color: red");
  lcd::print(3, "Alliance Time (0 = N/A): 0");
  lcd::print(4, "Center Time (0 = N/A): 0");
  lcd::print(5, "Configure Camera? No");
  lcd::print(7, "    -                 Select                 +");
  // Select a script type
  scriptNumber = -1;
  unsigned int selection = 0;
  while (scriptNumber == -1) {
    unsigned int button = lcd::read_buttons();
    if (button == 1) {
      if (selection != 0)
        selection--;
    }

    else if (button == 4) {
      if (selection < (scripts.size() - 1))
        selection++;
    }

    if (button == 2) {
      scriptNumber = selection;
      lcd::print(1, "**Selected Script** %s", titles.at(selection));
      break;
    }

    lcd::print(1, "Script: %s", titles.at(selection));
    delay(SCREEN_LOOP_DELAY);
  }
  delay(300);
  // Choose a side
  side = auton::color::red;
  auton::color sideSelection = auton::color::red;
  while (true) {
    unsigned int button = lcd::read_buttons();
    if (button == 1) {
      if (sideSelection == auton::color::red)
        sideSelection = auton::color::blue;
      else
        sideSelection = auton::color::red;
    }

    else if (button == 4) {
      if (sideSelection == auton::color::red)
        sideSelection = auton::color::blue;
      else
        sideSelection = auton::color::red;
    }

    if (button == 2) {
      side = sideSelection;
      if (side == auton::color::red)
        lcd::print(2, "**Selected Color:** red");
      else
        lcd::print(2, "**Selected Color:** blue");
      break;
    }

    if (sideSelection == auton::color::red)
      lcd::print(2, "Color: red");
    else
      lcd::print(2, "Color: blue");
    delay(SCREEN_LOOP_DELAY);
  }
  delay(300);

  // Select shot times
  int allianceMin =
      (!scriptNumber) ? MIN_FIRST_SHOT_DELAY : MIN_SECOND_SHOT_DELAY;
  int centerMin =
      (!scriptNumber) ? MIN_SECOND_SHOT_DELAY : MIN_FIRST_SHOT_DELAY;

  allianceFireTime = -1;
  unsigned int delaySelection = 0;
  while (allianceFireTime == -1) {
    unsigned int button = lcd::read_buttons();
    if (button == 4) {
      if (delaySelection == allianceMin) {
        delaySelection = 0;
      } else if (delaySelection != 0) {
        delaySelection--;
      }
    }

    else if (button == 1) {
      if (!delaySelection) {
        delaySelection = allianceMin;
      } else if (delaySelection < 45) {
        delaySelection++;
      }
    }

    if (button == 2) {
      allianceFireTime = delaySelection;
      lcd::print(3, "**Selected Alliance Time** %d", delaySelection);
      break;
    }

    lcd::print(3, "Alliance Time (0 = N/A): %d", delaySelection);
    delay(SCREEN_LOOP_DELAY);
  }
  delay(300);
  centerFireTime = -1;
  delaySelection = 0;
  while (centerFireTime == -1) {
    unsigned int button = lcd::read_buttons();
    if (button == 4) {
      if (delaySelection == centerMin) {
        delaySelection = 0;
      } else if (delaySelection != 0) {
        delaySelection--;
      }
    }

    else if (button == 1) {
      if (!delaySelection) {
        delaySelection = centerMin;
      } else if (delaySelection < 45) {
        delaySelection++;
      }
    }

    if (button == 2) {
      centerFireTime = delaySelection;
      lcd::print(4, "**Selected Center Time** %d", delaySelection);
      break;
    }

    lcd::print(4, "Center Time (0 = N/A): %d", delaySelection);
    delay(SCREEN_LOOP_DELAY);
  }
  delay(500);

  // configure camera
  int configureCamera = -1;
  int configureSelection = 0;
  while (configureCamera == -1) {
    unsigned int button = lcd::read_buttons();

    if (button == 4 || button == 1) {
      configureSelection ^= 1;
    }

    if (button == 2) {
      configureCamera = configureSelection;
      break;
    }

    if (configureSelection) {
      lcd::print(5, "Configure Camera? Yes");
    } else {
      lcd::print(5, "Configure Camera? No");
    }
    delay(SCREEN_LOOP_DELAY);
  }
  delay(500);
  if (configureCamera) {
    if (capCam)
      capCam->editSensorParamsLCD(side == auton::color::red ? 1 : 2);
  }

  for (int i = 0; i < 8; i++)
    lcd::clear_line(i);
  lcd::print(1, "    Autonomous Selection Complete.");
  if (logger) {
    if (side == auton::color::red) {
      logger->log(
          "L, Selected %s script on the RED side with %d alliance time and %d "
          "center time.\n",
          titles.at(scriptNumber).c_str(), allianceFireTime, centerFireTime);
    } else {
      logger->log(
          "L, Selected %s script on the BLUE side with %d alliance time and %d "
          "center time.\n",
          titles.at(scriptNumber).c_str(), allianceFireTime, centerFireTime);
    }
  }
}

void executeScript() {
  if (scriptNumber < 0)
    return; // Don't run a script if one has not been selected.
  scripts.at(scriptNumber)(side, allianceFireTime, centerFireTime);
}

void executeInit() {
  if (scriptNumber < 0)
    return;
  if (side == auton::color::red)
    capCam->setColor(true);
  else
    capCam->setColor(false);
  inits.at(scriptNumber)(side);
  runInit = true;
}

bool isScriptInitialized() { return runInit; }

auton::color getSide() { return side; }

std::string getScriptName() {
  if (scriptNumber < 0)
    return "";
  return titles.at(scriptNumber);
}
} // namespace lcdSelector
