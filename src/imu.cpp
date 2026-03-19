// #include <Arduino.h>
// #include "ICM_20948.h"

// #define SERIAL_PORT Serial
// #define WIRE_PORT Wire
// #define AD0_VAL 1

// ICM_20948_I2C myICM;

// // Fusion filter variables
// float roll = 0, pitch = 0, yaw = 0;
// float magYaw = 0;  // Yaw from magnetometer
// bool hasMagReference = false;
// unsigned long lastMagUpdate = 0;

// void setup() {
//     SERIAL_PORT.begin(115200);
//     delay(1000);
    
//     WIRE_PORT.begin();
//     WIRE_PORT.setClock(400000);
    
//     bool initialized = false;
//     while (!initialized) {
//         myICM.begin(WIRE_PORT, AD0_VAL);
//         if (myICM.status != ICM_20948_Stat_Ok) {
//             delay(500);
//         } else {
//             initialized = true;
//         }
//     }
    
//     bool success = true;
//     success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
    
//     // Enable BOTH Quat6 (fast) and Quat9 (magnetometer correction)
//     success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
//     success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    
//     // Set update rates
//     success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);  // Fast (~225Hz)
//     success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 9) == ICM_20948_Stat_Ok);  // Slow (~25Hz)
    
//     success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
//     success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
//     success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
//     success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
    
//     if (!success) {
//         while (1);
//     }
    
//     delay(2000);
// }

// // Helper function to normalize angle to -180 to +180
// float normalizeAngle(float angle) {
//     while (angle > 180.0) angle -= 360.0;
//     while (angle < -180.0) angle += 360.0;
//     return angle;
// }

// // Helper function to calculate shortest angle difference
// float angleDifference(float target, float current) {
//     float diff = target - current;
//     return normalizeAngle(diff);
// }

// void loop() {
//     icm_20948_DMP_data_t data;
//     myICM.readDMPdataFromFIFO(&data);
    
//     if ((myICM.status == ICM_20948_Stat_Ok) || 
//         (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
        
//         // Priority 1: Always use fast Quat6 for immediate response
//         if ((data.header & DMP_header_bitmap_Quat6) > 0) {
            
//             double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
//             double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
//             double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;
//             double qsum = (q1 * q1) + (q2 * q2) + (q3 * q3);
            
//             if (qsum < 1.0 && qsum > 0.0) {
//                 double q0 = sqrt(1.0 - qsum);
//                 double q2sqr = q2 * q2;
                
//                 // Roll
//                 double t0 = +2.0 * (q0 * q1 + q2 * q3);
//                 double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
//                 roll = atan2(t0, t1) * 180.0 / PI;
                
//                 // Pitch
//                 double t2 = +2.0 * (q0 * q2 - q3 * q1);
//                 t2 = constrain(t2, -1.0, 1.0);
//                 pitch = asin(t2) * 180.0 / PI;
                
//                 // Yaw (will drift without magnetometer)
//                 double t3 = +2.0 * (q0 * q3 + q1 * q2);
//                 double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
//                 float rawYaw = atan2(t3, t4) * 180.0 / PI;
                
//                 // Apply magnetometer correction if available
//                 if (hasMagReference) {
//                     // Slowly correct yaw drift using magnetometer
//                     // Calculate how much to correct
//                     float yawError = angleDifference(magYaw, rawYaw);
                    
//                     // Apply small correction each frame (0.5% per update)
//                     // This gives smooth correction without jumps
//                     yaw = rawYaw + (yawError * 0.005);
//                 } else {
//                     yaw = rawYaw;
//                 }
                
//                 // Output fast, smooth data
//                 if (!isnan(roll) && !isnan(pitch) && !isnan(yaw)) {
//                     SERIAL_PORT.print(roll);
//                     SERIAL_PORT.print(",");
//                     SERIAL_PORT.print(pitch);
//                     SERIAL_PORT.print(",");
//                     SERIAL_PORT.println(yaw);
//                 }
//             }
//         }
        
//         // Priority 2: Update magnetometer reference when available
//         if ((data.header & DMP_header_bitmap_Quat9) > 0) {
            
//             double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
//             double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
//             double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
//             double qsum = (q1 * q1) + (q2 * q2) + (q3 * q3);
            
//             if (qsum < 1.0 && qsum > 0.0) {
//                 double q0 = sqrt(1.0 - qsum);
//                 double q2sqr = q2 * q2;
                
//                 // Calculate magnetometer-corrected yaw
//                 double t3 = +2.0 * (q0 * q3 + q1 * q2);
//                 double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
//                 float newMagYaw = atan2(t3, t4) * 180.0 / PI;
                
//                 if (!isnan(newMagYaw)) {
//                     // Low-pass filter on magnetometer yaw
//                     if (!hasMagReference) {
//                         magYaw = newMagYaw;
//                         hasMagReference = true;
//                     } else {
//                         // Smooth magnetometer updates
//                         float yawDiff = angleDifference(newMagYaw, magYaw);
//                         magYaw = normalizeAngle(magYaw + yawDiff * 0.1);
//                     }
//                     lastMagUpdate = millis();
//                 }
//             }
//         }
//     }
    
//     // If no magnetometer data for 5 seconds, disable correction
//     if (hasMagReference && (millis() - lastMagUpdate > 5000)) {
//         hasMagReference = false;
//     }
    
//     delay(10);  // ~100Hz output
// }