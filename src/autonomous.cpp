#include "main.h"

void autonomous() {
  // Initialize the script paths if they didn't get completed in comp_init for
  // some reason
  if (!lcdSelector::isScriptInitialized()) {
    printf("didn't initialize\n");
    lcdSelector::executeInit();
  }
  lcdSelector::executeScript();
}
