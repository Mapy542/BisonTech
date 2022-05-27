#include "vex.h"

const char* printToConsole_numberFormat() {
  // look at the current precision setting to find the format string
  //switch(Console_precision){
    //case 0:  return "%.0f"; // 0 decimal places (1)
    //case 1:  return "%.1f"; // 1 decimal place  (0.1)
    //case 2:  return "%.2f"; // 2 decimal places (0.01)
   // case 3:  return "%.3f"; // 3 decimal places (0.001)
    //default: 
    return "%.6f"; // use the print system default for everthing else
  //}
}

void Print_XYR() {
  extern double CurrentXAxis;
  extern double CurrentYAxis;
  
  printf("X");
  printf(printToConsole_numberFormat(), static_cast<float>(CurrentXAxis));
  printf(",Y");
  printf(printToConsole_numberFormat(), static_cast<float>(CurrentYAxis));
  printf(",R");
  printf(printToConsole_numberFormat(), static_cast<float>(Gyroscope.heading(degrees)));
  printf("\n");
}