#ifndef uwEstimator_H
#define uwEstimator_H

// Import necessary packages
#include <cmath>                // For goniometric calculations
#include <filters.h>            // For low-pass filters
#include <BasicLinearAlgebra.h> // For matrix operations

using namespace BLA;

class uwEstimator {
    public:
        // Constructors
        uwEstimator();
        void init();
        /**
        * Updates the u and w velocity estimates using Forward Euler integration.
        * @param ax    Specific force in body X-axis from accelerometer (m/s^2)
        * @param az    Specific force in body Z-axis from accelerometer (m/s^2)
        * @param pitch Pitch angle from Extended Kalman Filter (rad)
        * @param roll   Roll angle from Extended Kalman Filter (rad)
        * @param pitch_rate     Pitch rate from gyro (rad/s)
        */        // Eventually predict step? Using model?
        //void predict(float gx, float gy, float gz, float dt, bool calibrating);
        // Update step using the accelerometer data
        void update(float ax,float az, float pitch, float roll, float pitch_rate);
        
        float getU() const { return u; }
        float getW() const { return w; }
        
        void setU(float u_val) { u = u_val; }
        void setW(float w_val) { w = w_val; }
    
    private:
        // Private variables
        float *_ax_filt, *_az_filt;          // (Reference to) filtered accelerometer data

    
        Filter ax_filter;                               // Instance of low-pass filter for accelerometer (in x-direction)
        Filter az_filter;                               // Instance of low-pass filter for accelerometer (in z-direction)

        float *_u = 0;                               // (Reference to) u estimation
        float *_w = 0;                              // (Reference to) w estimation

        float d2r = asin(1)/90;                         // Conversion from degrees to radians
        float r2d = 90/asin(1);                         // Conversion from radians to degrees
};

#endif