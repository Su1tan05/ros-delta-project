#include "mbed.h"

class Filters{

    private:
    
    public:
    //Filters(); 
    float kalmanFilter(float k, float potData);
    float moovingAverageFilter(float k, float potData);
};