#include <KUlibrie.h>

KUlibrie* KUlibrie::instance = nullptr;

// -----------
// Constructor
// -----------
KUlibrie::KUlibrie(float *u, float *w, float *q, float *theta, 
                   float *VI0, float *servo_angle, 
                   float *ref_pitch)
    :   imu(I2C_MODE, 0x6A),
        telemetryService("1101"), telemetryChar("2101"),
        controlService("1102"), controlChar("2102"),
        actionService("1103"), actionChar("2103"),
        referenceService("1104"), referenceChar("2104"),
        
        filter(Q, R, f_acc, f_gyr, 
            u, w, q, theta, 
            &ax_filt, &az_filt, &gy_filt, 
            0, 0, 0),        
            file(InternalFS) {
            
            instance = this;

            _u = u; _w = w; _q = q; _theta = theta;
            _VI0 = VI0; 
            _servo_angle = servo_angle;
            _ref_pitch = ref_pitch;

            // Set legacy pointers to null to avoid crashes
            _roll = nullptr; _pitch_legacy = nullptr; _yaw_rate = nullptr;
            _ref_roll = nullptr; _ref_yaw_rate = nullptr;
        }


// -----------------------------------------
// Function that executes the hardware setup
// -----------------------------------------
void KUlibrie::setup_hardware() {
    /*
    This function executes the hardware setup. This includes setting up the bluetooth 
    communication, assigning the pins and reading the calibration settings from the file.
    */

    // Setup bluetooth communication
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);  // Set bandwidth to maximum
    Bluefruit.configUuid128Count(15);

    Bluefruit.begin();
    Bluefruit.setTxPower(4);                       // Parameter that indicates power output during packet transmissions
    
    Bluefruit.setName("KUlibrie");                 // Set Bluetooth name of the device

    // Setup services and characteristics
    telemetryService.begin();
    telemetryChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    telemetryChar.setFixedLen(NB_POINTS_PER_SEND * DATANUM_TELEMETRY);
    telemetryChar.begin();

    controlService.begin();
    controlChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    controlChar.setFixedLen(NB_POINTS_PER_SEND * DATANUM_CONTROL);
    controlChar.begin();

    actionService.begin();
    actionChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ | CHR_PROPS_WRITE);
    actionChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    actionChar.setFixedLen(DATANUM_ACTION);
    actionChar.setWriteCallback(KUlibrie::staticAction); // Runs the "staticAction" function when the data on this characteristic is changed
    actionChar.begin();

    referenceService.begin();
    referenceChar.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ | CHR_PROPS_WRITE);
    referenceChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    referenceChar.setFixedLen(DATANUM_REFERENCE);
    referenceChar.setWriteCallback(KUlibrie::staticReference); // Runs the "staticAction" function when the data on this characteristic is changed
    referenceChar.begin();

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(telemetryService);
    Bluefruit.Advertising.addService(controlService);
    Bluefruit.Advertising.addService(actionService);
    Bluefruit.ScanResponse.addName();

    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setIntervalMS(20, 153);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);

    // Setup pins for driveline
    pinMode(SLP_PIN1, OUTPUT);
    pinMode(SLP_PIN2, OUTPUT);
    pinMode(DIREC_A, OUTPUT);
    pinMode(PWM_A, OUTPUT);
    pinMode(DIREC_B, OUTPUT);
    pinMode(PWM_B, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(TRIGGERPIN, INPUT);

    digitalWrite(SLP_PIN1, HIGH);
    digitalWrite(SLP_PIN2, HIGH);
    digitalWrite(DIREC_A, LOW);
    analogWrite(PWM_A, 0);
    digitalWrite(DIREC_B, LOW);
    analogWrite(PWM_B, 0);

    // Start the IMU sensor
    imu.begin();

    // Access the calibration file
    InternalFS.begin();

    file.open(FILENAME, FILE_O_READ);

    // Check if the correct file is located in the memory of the KUlibrie
    if (file) {
        // Read the contents of the file
        Serial.println("Reading calibration data from file");
        uint8_t buffer[24] = { 0 };
        uint32_t readlen = file.read(buffer, sizeof(buffer));

        buffer[readlen] = 0;
        
        // Close the connection to the file
        file.close();

        // Copy data from the file to the cataconversion variable
        for (int i = 0; i < sizeof(buffer); ++i) {
            calibrationData.bytes[i] = buffer[i];
        }

        calibration(0, 0) = calibrationData.value[0];
        calibration(0, 1) = calibrationData.value[1];
        calibration(1, 0) = calibrationData.value[2];
        calibration(1, 1) = calibrationData.value[3];
        calibration(2, 0) = calibrationData.value[4];
        calibration(2, 1) = calibrationData.value[5];
        
        // Set the calibration settings
        filter.set_calibration(calibration, gyro_biasses);

        Serial.println("Calibration data:");
        Serial.print(calibration(0, 0), 4);
        Serial.print("    ");
        Serial.println(calibration(0, 1), 4);
        Serial.print(calibration(1, 0), 4);
        Serial.print("    ");
        Serial.println(calibration(1, 1), 4);
        Serial.print(calibration(2, 0), 4);
        Serial.print("    ");
        Serial.println(calibration(2, 1), 4);
    }
    
}


// ---------------------------------------
// Function that interprets given commands
// ---------------------------------------
void KUlibrie::action(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    /*
    This function is called when there is a change on the "action" characteristics (by the "staticAction" function (see later))

    Inputs:
        - conn_handle       uint16_t                Connection handler
        - chr               BLECharacteristic*      The id of the bluetooth characteristic that changed
        - data              uint8_t*                The data currently on the characteristic
        - len               uint16_t                Length of the data vector
    */

    // The "action" characterisctic should only contain 1 byte
    int receivedData = data[0];
    
    // Check if the characteristic that changed is the "action" characteristic (not actually necessary, this function can only be called when the "action" characteristic changed)
    if (chr == &actionChar) {
        // Execute the desired command
        if (receivedData == 0) {
            // Start sending telemetry data
            send_telemetry_data = true;
        } else if (receivedData == 1) {
            // Stop sending telemetry data
            send_telemetry_data = false;
        } else if (receivedData == 2) {
            // Start sending controller data
            send_control_data = true;
        } else if (receivedData == 3) {
            // Stop sending controller data
            send_control_data = false;
        } else if (receivedData == 4) {
            // Start the controller (= start flying)
            control = true;

            // Turn the LED green
            digitalWrite(LED_RED, HIGH);
            digitalWrite(LED_GREEN, LOW);
            digitalWrite(LED_BLUE, HIGH);

            // Store the start time
            start_control = millis();
            
            // Initiate the sent telemetry and controller data to 0
            memset(telemetryData.bytes, 0, NB_POINTS_PER_SEND * DATANUM_TELEMETRY);
            memset(controlData.bytes, 0, NB_POINTS_PER_SEND * DATANUM_CONTROL);
            
            // Notify the central computer when the controller was started
            send_telemetry(start_control);
            send_control(start_control);
            
        } else if (receivedData == 5) {
            // Stop the controller (= stop flying)
            control = false;
        
            // Turn of the LED
            digitalWrite(LED_RED, HIGH);
            digitalWrite(LED_GREEN, HIGH);
            digitalWrite(LED_BLUE, HIGH);
        } else if (receivedData == 6) {
            // Start the calibration process
            calibrate = true;
        }
    }
  
}


// -----------------------------------------------------------------------
// Function that is called when a command is given by the central computer
// -----------------------------------------------------------------------
void KUlibrie::staticAction(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    /*
    This function is called when a bluetooth characteristic is changed. This function will call the local 
    "action" function to interpret the command.

    Inputs:
        - conn_handle       uint16_t                Connection handler
        - chr               BLECharacteristic*      The id of the bluetooth characteristic that changed
        - data              uint8_t*                The data currently on the characteristic
        - len               uint16_t                Length of the data vector
    */
    instance->action(conn_handle, chr, data, len);
}


// --------------------------------------------------------
// Function that interprets HEX( 0000-0099)
// --------------------------------------------------------
void KUlibrie::reference(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    
    // 1. Read just the first byte (0-255)
    uint8_t value = data[0];

    // 2. Clamp input to 100 max (just in case you type 101+)
    if (value > 100) value = 100;

    // 3. Map 0-100 to 0.0-3.7 Volts
    // Formula: (Value / 100.0) * 3.7
    float desired_voltage = ((float)value / 100.0) * 3.7;

    // 4. Set the voltage
    *_ref_pitch = desired_voltage;
}

// -------------------------------------------------------------------------
// Function that is called when a reference is given by the central computer
// -------------------------------------------------------------------------
void KUlibrie::staticReference(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    /*
    This function is called when a bluetooth characteristic is changed. This function will call the local 
    "reference" function to interpret the command.

    Inputs:
        - conn_handle       uint16_t                Connection handler
        - chr               BLECharacteristic*      The id of the bluetooth characteristic that changed
        - data              uint8_t*                The data currently on the characteristic
        - len               uint16_t                Length of the data vector
    */
    instance->reference(conn_handle, chr, data, len);
}

// ---------------------------------------------
// Function that updates the attitude estimation
// ---------------------------------------------
void KUlibrie::update_filter(float dt) {
    // 1. Read Raw Sensors
    ax = imu.readFloatAccelX();
    ay = imu.readFloatAccelY();
    az = imu.readFloatAccelZ();

    gx = imu.readFloatGyroX();
    gy = imu.readFloatGyroY(); // Pitch Rate
    gz = imu.readFloatGyroZ();

    // 2. Predict Step (Model)
    // Uses Voltage (*_VI0) and Servo Angle (*_servo_angle converted to radians)
    float servo_rad = (*_servo_angle) * d2r;
    filter.predict(*_VI0, servo_rad, dt);

    // 3. Update Step (Correction)
    // Uses Ax, Az, and Gy (Pitch Rate)
    // 'calibrate' flag tells the filter if it should just collect data or filter it
    filter.update(ax, az, gy, calibrate);
}
// ---------------------------------------------------------
// Function that fits a linear regression through given data
// ---------------------------------------------------------
void KUlibrie::curve_fit(Matrix<3> x, Matrix<3> y, float axis) {
    /*
    This function calculates a linear regression through given (calibration) data.

    Inputs:
        - x         3d vector       x-data
        - y         3d vector       y-data
        - axis      float           Indicates the direction (to store the correct calibration settings)
    */
    Matrix<1> a;
    a = Inverse(~x*x)*~x*y;

    float b = Mean(y - x*a);

    Matrix<2> fit = {a(0), b};
    
    calibration(axis, 0) = fit(0);
    calibration(axis, 1) = fit(1);

    Serial.println("The linear fit is in the form: actual = a*measurement + b, with:");
    Serial.print("    a = ");
    Serial.print(fit(0), 4);
    Serial.print("    b = ");
    Serial.println(fit(1), 4);
}

// -------------------------------------------------
// Function that stores the biasses of the gyroscope
// -------------------------------------------------
void KUlibrie::set_gyro_biasses(float bias_gx, float bias_gy, float bias_gz) {
    /*
    This function stores the biasses that are measured on the gyroscope

    Inputs:
        - bias_gx       float       Bias in the x-direction
        - bias_gy       float       Bias in the y-direction
        - bias_gz       float       Bias in the z-direction
    */
    gyro_biasses = {bias_gx, bias_gy, bias_gz};
}

// -------------------------------------------
// Function that sets the calibration settings
// -------------------------------------------
void KUlibrie::set_calibration() {
    /*
    This function sets the calibration settings for the accelerometer data in the 
    Kalman filter instance and writes it to the calibration file on the memory
    */
    
    filter.set_calibration(calibration, gyro_biasses);

    write_calibration();
}

// -------------------------------------------------------
// Function that writes the calibration settings to a file
// -------------------------------------------------------
void KUlibrie::write_calibration() {
    /*
    This function writes the calibration settings to a file on the EMMC
    */
    
    // Transform the calibration data to bytes
    calibrationData.value[0] = calibration(0, 0);
    calibrationData.value[1] = calibration(0, 1);
    calibrationData.value[2] = calibration(1, 0);
    calibrationData.value[3] = calibration(1, 1);
    calibrationData.value[4] = calibration(2, 0);
    calibrationData.value[5] = calibration(2, 1);
    
    // Open the calibration file on the EMMC
    file.open(FILENAME, FILE_O_WRITE);

    // Create a vector containing the calibration data
    uint8_t content[24];
    // Copy data from source to destination
    for (int i = 0; i < sizeof(content); ++i) {
        content[i] = calibrationData.bytes[i];
    }
    
    // Start writing at the beginning of the file
    file.seek(0);

    // Write the content
    int r = file.write(content, sizeof(content));

    // Close the file
    file.close();
}

// -------------------------------------------------------
// Function to send telemetry data to the central computer
// -------------------------------------------------------
void KUlibrie::send_telemetry(unsigned long t) {
    /*
    This function stores and sends the telemetry data

    Inputs:
        - t         unsigned long       Number of milliseconds sinds last boot
    */

    // Increase the number of passed timesteps
    count_timesteps_telemetry += 1;
    
    // A number of telemetry timesteps is stored to be send via bluetooth. 
    // This data is send with a frequency (about) equal to the refreshrate of the Matlab app
    if (count_timesteps_telemetry > PERIOD_APP / PERIOD_CONTROLLER / NB_POINTS_PER_SEND) {
        // Convert the telemetry data to bytes
        telemetryData.value[16*count_period_telemetry] = ax;
        telemetryData.value[16*count_period_telemetry + 1] = ay;
        telemetryData.value[16*count_period_telemetry + 2] = az;
        telemetryData.value[16*count_period_telemetry + 3] = ax_filt;
        telemetryData.value[16*count_period_telemetry + 4] = ay_filt;
        telemetryData.value[16*count_period_telemetry + 5] = az_filt;
        
        telemetryData.value[16*count_period_telemetry + 6] = gx;
        telemetryData.value[16*count_period_telemetry + 7] = gy;
        telemetryData.value[16*count_period_telemetry + 8] = gz;
        telemetryData.value[16*count_period_telemetry + 9] = gx_filt*r2d;
        telemetryData.value[16*count_period_telemetry + 10] = gy_filt*r2d;
        telemetryData.value[16*count_period_telemetry + 11] = gz_filt*r2d;
        
        telemetryData.value[16*count_period_telemetry + 12] = *_u;
        telemetryData.value[16*count_period_telemetry + 13] = *_theta * r2d; // Convert to degrees
        telemetryData.value[16*count_period_telemetry + 14] = *_w;
        
        telemetryData.value[16*count_period_telemetry + 15] = (float)(t - start_control)/1000;
        
        count_period_telemetry += 1;
        
        // If the vector is full, the vector containing a number of timesteps is send via bluetooth
        if (count_period_telemetry == NB_POINTS_PER_SEND) {
            telemetryChar.notify(telemetryData.bytes, NB_POINTS_PER_SEND * DATANUM_TELEMETRY);

            count_period_telemetry = 0;
        }

        // Reset the timestep counter
        count_timesteps_telemetry = 0;
    }
}

// --------------------------------------------------------
// Function to send controller data to the central computer
// --------------------------------------------------------
void KUlibrie::send_control(unsigned long t) {
    /*
    This function stores and sends the controller data

    Inputs:
        - t         unsigned long       Number of milliseconds sinds last boot
    */
    
    // Increase the number of passed timesteps
    count_timesteps_control += 1;
    
    // A number of telemetry timesteps is stored to be send via bluetooth. 
    // This data is send with a frequency (about) equal to the refreshrate of the Matlab app
    if (count_timesteps_control > PERIOD_APP / PERIOD_CONTROLLER / NB_POINTS_PER_SEND) {
        // Convert the controller data to bytes
        controlData.value[5*count_period_control]     = *_VI0;
        controlData.value[5*count_period_control + 1] = 0; // V0 (Unused)
        controlData.value[5*count_period_control + 2] = 0; // dV (Unused)
        controlData.value[5*count_period_control + 3] = *_servo_angle; // Using ds slot for servo
        controlData.value[5*count_period_control + 4] = (float)(t - start_control)/1000;

        count_period_control += 1;

    
        // If the vector is full, the vector containing a number of timesteps is send via bluetooth
        if (count_period_control == NB_POINTS_PER_SEND) {
            controlChar.notify(controlData.bytes, NB_POINTS_PER_SEND * DATANUM_CONTROL);

            count_period_control = 0;
        }

        // Reset the timestep counter
        count_timesteps_control = 0;
    }
}

// --------------------------------------------
// Getters:
// These functions return the filtered IMU data
// --------------------------------------------
float KUlibrie::readFloatAccelX() {
    return ax_filt;
}

float KUlibrie::readFloatAccelY() {
    return ay_filt;
}

float KUlibrie::readFloatAccelZ() {
    return az_filt;
}

float KUlibrie::readFloatGyroX() {
    return gx_filt;
}

float KUlibrie::readFloatGyroY() {
    return gy_filt; // Note: This now contains the filtered pitch rate from LongitudinalEKF
}

float KUlibrie::readFloatGyroZ() {
    return gz_filt;
}

// --------------------------------------------------------------------------------
// Legacy Functions (Kept for compatibility, but emptied to prevent crashes)
// The LongitudinalEKF does not support manual resetting of individual axes
// and does not use the old _roll/_pitch pointers.
// --------------------------------------------------------------------------------

void KUlibrie::set_roll(float roll) {
    // Function disabled for Longitudinal Model to prevent NullPointer crash
}

void KUlibrie::set_pitch(float pitch) {
    // Function disabled for Longitudinal Model to prevent NullPointer crash
    // If you need to reset pitch, you would ideally add a method to LongitudinalEKF
    // e.g., filter.set_state(u, w, q, pitch);
}

void KUlibrie::set_ax_filter(float ax) {
    // Disabled: Filter handles this internally
}

void KUlibrie::set_ay_filter(float ay) {
    // Disabled
}

void KUlibrie::set_az_filter(float az) {
    // Disabled
}