#ifndef uwEstimator_H
#define uwEstimator_H

#include <cmath>                
#include <filters.h>            
#include <BasicLinearAlgebra.h> 

using namespace BLA;

class uwEstimator {
    public:
        uwEstimator();
        uwEstimator(float* u_ptr, float* w_ptr, float f_acc, float* pitch_ptr, float* roll_ptr);

        // Public functions
        void update(float raw_ax, float raw_az, float pitch_rate, float dt);
        void set_calibration(Matrix<3,2> calibration);

        // Public variables (for consistency with ExtendedKalman, though ExtendedKalman uses a bool parameter)
        bool calibrating = false; // Indicates if the drone is flying or being calibrated
        
    private:
        Filter ax_filter;                               
        Filter az_filter;                               

        // Internal pointers to the shared state
        float* _u = nullptr;                               
        float* _w = nullptr;                              
        float* _pitch_ptr = nullptr;
        float* _roll_ptr = nullptr;

        const float g = 9.81f; 

        Matrix<3,2> _calibration = {1, 0, 0, 0, 1, 0}; // Calibration settings {slope ax, bias ax, slope ay, bias ay, slope az, bias az}
};

#endif