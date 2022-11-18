#include "mbed.h"
#include "filters.h"

float Filters::kalmanFilter(float k, float potData){
return k;
}

float Filters::moovingAverageFilter(float k, float potData){

return  k;

}
    
   /* float kalman_read(float k)
    {
        static float real_angle = read();
        real_angle = real_angle * k + read() * (1 - k);
        return real_angle;
    }

    //Moving average
    float moving_average()
    {
        const int k = 12; // Размер окна
        static int i = 0;
        static float array[k];
        static float average = 0;
        average -= array[i];
        array[i] = read();
        average += array[i];
        if (++i >= k) i = 0;
        return float(average/k);
    }*/