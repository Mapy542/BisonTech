#include "BaseCode/VectorEngine/Robot_Telemetry_Structure.cpp" //include the definition of ricky so the code has context

// Precondition: an input is given as an interger
// Postcondition: the input is returned as an interger and incremented by 1
// multiple times
void testFunction(
    int inputvalue) { // void defines a function that returns nothing, but int
                      // float or double or more could be used  //testFunction
                      // is the name of the function //inputvalue is the name of
                      // the variable that is passed into the function, and
                      // others comma semparated may be used
  extern Robot_Telemetry
      ricky; // includes the robots main telemetry structure for passing data
             // between functions //this is not the best way, but is most
             // logical for the current situation and may not be needed on all
             // functions

  int something =
      inputvalue; // create a new variable as an integer and assign it a value
  something += 1; // add 1 to the variable

  while (something < 10) { // while the variable is less than 10
    something += 1;        // add 1 to the variable
  }

  for (int i = 0; i < 10; i++) { // for loop that starts i at 0 and ends at 10
    something += 1;              // add 1 to the variable
  }

  // return something; //return the variable to what called the function //not
  // aplicable to void functions

}; // end of function with a semicolon