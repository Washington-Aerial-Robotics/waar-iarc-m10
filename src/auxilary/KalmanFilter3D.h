#pragma once
#include <cmath>
#include <chrono>
#include <stdexcept>

// Example usage:
//    KalmanFilter3D kf(1);
//    kf.update(...);
//    kf.getPositionX();
//    kf.update(...);

class KalmanFilter3D {
public:


    /*
        Mode cheat sheet
        Enter a number between 0 and 3 for the mode

        0        Default, // gps should be in meters NED, acc should be rotated and without gravity, mag must be calibrated
        1        CalibrateMag, // calibrates mag data inside the filter
        2        ConvertGPS2Meters, // converts from gps to meters inside the filter, input lat long for gps x and y pos
        3        Full // calibrates mag and convert gps in the filter
    */

    KalmanFilter3D(
        int mode,
        double initial_lat,
        double initial_long
    );


    // Input position and acceleration in NED frame
    // Input variance of each measurement as well
    void update(double px, double ax, double py, double ay, double pz, double az,
                double varpx, double varax, double varpy, double varay,
                double varpz, double varaz, double magx, double magy, double magz);

    double getPositionX(); // returns in NED in meters
    double getPositionY();
    double getPositionZ();

    double getVelocityX();
    double getVelocityY();
    double getVelocityZ();

    double getAccelerationX();
    double getAccelerationY();
    double getAccelerationZ();

    double getMagx();
    double getMagy();
    double getMagz();
};