#include "main.h"
#include <random>

DataLogger *logger;

unsigned int hash3(unsigned int h1, unsigned int h2, unsigned int h3) {
  return ((h1 * 2654435789) + h2) * 2654435789 + h3;
}

// From https://stackoverflow.com/a/12468109
static std::string random_string(size_t length) {
  uint32_t now = pros::millis();
  double sumY = 0, sumX = 0, sumT = 0;
  odom::start(true);
  pros::delay(100);
  pros::lcd::initialize();
  pros::lcd::print(0, "Generating RNG Seed");
  while (pros::millis() - now < 2000) {
    sumY += odom::getState().y.convert(inch);
    sumX += odom::getState().x.convert(inch);
    sumT += odom::getState().theta.convert(degree);
    pros::delay(5);
  }
  odom::stop();
  pros::lcd::clear_line(0);
  std::mt19937 gen(hash3((int)sumY, (int)sumX, (int)sumT)); // Mersenne twister
  std::uniform_real_distribution<double> dist(0, RAND_MAX);
  auto randchar = [&dist, &gen]() -> char {
    const char charset[] = "0123456789"
                           "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                           "abcdefghijklmnopqrstuvwxyz";
    const size_t max_index = (sizeof(charset) - 1);
    return charset[(int)dist(gen) % max_index];
  };
  std::string str(length, 0);
  std::generate_n(str.begin(), length, randchar);
  return str;
}

DataLogger::DataLogger(output iout) {
  if (iout == output::SD_CARD) {
    const int nameSize = 10;
    // char* filename = (char*)(malloc(nameSize));  // guessing on the size here
    // pros::date_s_t d = pros::get_date();
    // pros::time_s_t t = pros::get_time();
    // snprintf(filename, nameSize, "/usd/%04d%02d%02d%02d%02d%02d.txt", d.year,
    //          d.month, d.day, t.hour, t.min, t.sec);
    std::string filename = random_string(nameSize);
    filename.append(".csv");
    printf("%s\n", filename.c_str());
    const char *fname = strcat("/usd/", filename.c_str());
    file = fopen(fname, "w");
    if (file)
      fprintf(file, "Starting Log:\n");
    else
      printf("ERROR OPENING FILE: %s\n", strerror(errno));
    // free(filename);
  } else if (iout == output::STD_OUT) {
    file = fopen("/ser/sout", "w");
  }
}
