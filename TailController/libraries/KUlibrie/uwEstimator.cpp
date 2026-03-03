#include "uwEstimator.h"

// Constructor sets pointers safely to nullptr
uwEstimator::uwEstimator() {
    _u = nullptr;
    _w = nullptr;
}

// Initialization: Link the local pointers to the main memory addresses
void uwEstimator::init(float* u_ptr, float* w_ptr) {
    _u = u_ptr;
    _w = w_ptr;
    
    // Set the initial physical values to 0 m/s
    if (_u != nullptr) *_u = 0.0f;
    if (_w != nullptr) *_w = 0.0f;
}

// Main update loop
void uwEstimator::update(float ax, float az, float* pitch, float* roll, float pitch_rate, float dt) {
    
    // Safety check: Don't do math if the pointers aren't linked!
    if (_u == nullptr || _w == nullptr || pitch == nullptr || roll == nullptr) {
        return; 
    }

    // 1. FILTERING
    ax_filter.update(ax); // Pass through ax_filter if needed
    az_filter.update(az); // Pass through az_filter if needed


    // If the drone is calibrating, the raw filtered data should be used, otherwise, 
    // the measurements should be transformed using the calibration settings
    if (calibrating) {
        *_ax_filt = ax_filter.get_filtered();
        *_az_filt = az_filter.get_filtered();
    } else {
        *_ax_filt = ax_filter.get_filtered()*_calibration(0, 0) + _calibration(0, 1);
        *_az_filt = az_filter.get_filtered()*_calibration(2, 0) + _calibration(2, 1);
    }

    // 2. GRAVITY COMPENSATION
    // We use '*pitch' and '*roll' to read the actual angles calculated by the EKF
    float a_lin_x = ax_f - (g * sin(*pitch));
    float a_lin_z = az_f - (g * cos(*pitch) * cos(*roll));

    // 3. CORIOLIS EFFECT COMPENSATION
    // We use '*_u' and '*_w' to read the current velocity states
    float du = a_lin_x - (pitch_rate * (*_w));
    float dw = a_lin_z + (pitch_rate * (*_u));

    // 4. FORWARD EULER INTEGRATION
    // We ADD the new delta velocity DIRECTLY into the shared memory!
    *_u += (du * dt);
    *_w += (dw * dt);

    // 5. LEAKY INTEGRATION (DRIFT PROTECTION & DRAG)
    const float leak_factor = 0.99f; 
    *_u *= leak_factor;
    *_w *= leak_factor;
}