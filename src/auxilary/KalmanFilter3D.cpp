#include "KalmanFilter3D.h"
#include <cmath>
#include <chrono>
#include <stdexcept>



struct Vector3 {
    double data[3] = {0};

    double dotProduct(Vector3 otherVector) {
        double result = 0;
        for (int i = 0; i < 3; ++i) {
            result = result + otherVector.data[i]*data[i];
        }
        return result;
    }

    Vector3 add(Vector3 otherVector) {
        Vector3 result;
        for (int i = 0; i < 3; ++i) {
            result.data[i] = data[i] + otherVector.data[i];
        }
        return result;
    }
    Vector3 subtract(Vector3 otherVector) {
        Vector3 result;
        for (int i = 0; i < 3; ++i) {
            result.data[i] = data[i] - otherVector.data[i];
        }
        return result;
    }
};

struct Matrix3x3 {
    double data[3][3] = {0};

    static Matrix3x3 Identity() {
        Matrix3x3 m;
        for (int i = 0; i < 3; ++i) m.data[i][i] = 1.0;
        return m;
    }


    static Matrix3x3 OuterProduct(Vector3 v1, Vector3 v2) {
        Matrix3x3 m;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                m.data[i][j] = v1.data[i]*v2.data[j];
            }
        }
        return m;
    }

    Matrix3x3 transpose() const {
        Matrix3x3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.data[i][j] = data[j][i];
            }
        }
        return result;
    }

    // matrix multiplication
    Matrix3x3 multiply(const Matrix3x3& other) const {
        Matrix3x3 result;

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.data[i][j] = 0;
                for (int k = 0; k < 3; ++k) {
                    result.data[i][j] += data[i][k] * other.data[k][j];
                }
            }
        }

        return result;
    }

    Matrix3x3 scalarMult(double mult) {
        Matrix3x3 result;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.data[i][j] = data[i][j]*mult;
            }
        }
        return result;
    }

    // vector multiplication
    Vector3 multiply(const Vector3& v) const {
        Vector3 result;

        for (int i = 0; i < 3; ++i) {
            result.data[i] = 0;
            for (int k = 0; k < 3; ++k) {
                result.data[i] += data[i][k] * v.data[k];
            }
        }

        return result;
    }

     // matrix addition
    Matrix3x3 add(const Matrix3x3& other) const {
        Matrix3x3 result;

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                    result.data[i][j] = data[i][j] + other.data[i][j];
            }
        }
        return result;
    }

    // from chatGPT, may need double checking or replacement
    Matrix3x3 inverse() const {
        Matrix3x3 inv;

        const double a = data[0][0], b = data[0][1], c = data[0][2];
        const double d = data[1][0], e = data[1][1], f = data[1][2];
        const double g = data[2][0], h = data[2][1], i = data[2][2];

        double det =
            a * (e * i - f * h) -
            b * (d * i - f * g) +
            c * (d * h - e * g);

        if (det == 0.0) {
            throw std::runtime_error("Matrix is not invertible");
        }

        double invDet = 1.0 / det;

        inv.data[0][0] =  (e * i - f * h) * invDet;
        inv.data[0][1] = -(b * i - c * h) * invDet;
        inv.data[0][2] =  (b * f - c * e) * invDet;

        inv.data[1][0] = -(d * i - f * g) * invDet;
        inv.data[1][1] =  (a * i - c * g) * invDet;
        inv.data[1][2] = -(a * f - c * d) * invDet;

        inv.data[2][0] =  (d * h - e * g) * invDet;
        inv.data[2][1] = -(a * h - b * g) * invDet;
        inv.data[2][2] =  (a * e - b * d) * invDet;

        return inv;
    }




};


// in the future, could add accelerometer bias state


class KalmanFilter3D {
public:

    double ACCVARIANCE = 0.1; // estimate for constant imu accelerometer variance jerk
    int mode;
    double initial_lat; // initial latitude measurement, from where 0,0 in NED frame is based
    double initial_long; // initial longitude measurement, from where 0,0 in NED frame is based

    double IMUDEADBAND = 0.1; // deadband for imu data
                              // measurement of less than 0.1 for ax, ay, or az are treated like 0.0

    
    Vector3 mag = {0, 0, 0}; // magx, magy, magz



    // might need to initialize like {{0, 0, 0}}
    Vector3 x = {0, 0, 0}; // px, vx, ax
    Vector3 y = {0, 0, 0}; // py, vy, ay
    Vector3 z = {0, 0, 0}; // pz; vz, az

    Matrix3x3 H = {{ // state measurement matrix, z = Hx
        {1, 0, 0},   // not measuring velocity
        {0, 0, 0},
        {0, 0, 1}
    }};

    // old states, xn_n-1
    Vector3 xold = {0, 0, 0}; // px, vx, ax
    Vector3 yold = {0, 0, 0}; // py, vy, ay
    Vector3 zold = {0, 0, 0}; // pz; vz, az

    Matrix3x3 P_x_nn;
    Matrix3x3 P_y_nn;
    Matrix3x3 P_z_nn;

    // I intitialize, updates with old P predictions later on
    Matrix3x3 P_x_nm1_n = Matrix3x3::Identity(); // P n-1, n
    Matrix3x3 P_y_nm1_n = Matrix3x3::Identity(); // P n-1, n
    Matrix3x3 P_z_nm1_n = Matrix3x3::Identity(); // P n-1, n

    Matrix3x3 Q; // process covariance noise, uses accel jerk model
    Matrix3x3 F; // same for xzz

    // Need to establish these, and seperate x and y
    Matrix3x3 Rx = Matrix3x3::Identity();
    Matrix3x3 Ry = Matrix3x3::Identity();
    Matrix3x3 Rz = Matrix3x3::Identity();


    double dt = 0;

    // Create matrices needed

    // input mode, and initial lat and longitude to base gps off
    KalmanFilter3D(int mode, double initial_lat, double initial_long)
    {

        this->initial_lat = initial_lat*DEG2RAD;
        this->initial_long = initial_long*DEG2RAD;

        
        // variance matrix for x
        P_x_nn = Matrix3x3::Identity();

        P_y_nn = Matrix3x3::Identity();

        // variance matrix for z
        P_z_nn = Matrix3x3::Identity();


        last_update = std::chrono::steady_clock::now();
    }

    // main call for everything:
    // INPUTS:
    // px   Position, North positive
    // ax   Acceleration, North positive
    // py   Position, East positive
    // ay   Acceleration, East positive
    // pz   Position, Down positive
    // az   Acceleration, Down positive
    // varpx    Variance in position North measurements
    // varax    Variance in Acceleration North measurements
    // varpy    Variance in Position East measurements
    // varpa    Variance in Acceleration East measurements
    // varpz    Variance in Position Down measurements
    // varpa    Variance in Acceleration Down measurements
    void update(double px, double ax, double py, double ay, double pz, double az, double varpx, double varax, double varpy, double varay, double varpz, double varaz, double magx, double magy, double magz) {
        
        deadband(ax, ay, az);

        tick();

    /*
        Mode cheat sheet
        Enter a number between 0 and 3 for the mode

        0        Default, // gps should be in meters NED, acc should be rotated and without gravity, mag must be calibrated
        1        CalibrateMag, // calibrates mag data inside the filter
        2        ConvertGPS2Meters, // converts from gps to meters inside the filter, input lat long for gps x and y pos
        3        Full // calibrates mag and convert gps in the filter
    */
        if (mode == 1 || mode == 3) { // calibrate the mag
            double uncalib[3];
            double calib[3];
            uncalib[0] = magx;
            uncalib[1] = magy;
            uncalib[2] = magz;
            calibrateMag(uncalib, calib);
            magx = calib[0];
            magy = calib[1];
            magz = calib[2];
        }

        if (mode == 2 || mode == 3) {
            // method from https://www.edwilliams.org/avform147.htm#flat
            // convert to rad
            px = px*DEG2RAD;
            py = py*DEG2RAD;
            double dlong = px - initial_lat; // initial lat and long are already in rad
            double dlat = py - initial_long;
            double a = 6378137.000; // meters
            double f = 1/298.257223563;
            double e2 = f*(2-f);
            double R1 = std::pow( a*(1-e2)/(1-e2*std::sin(initial_lat)*std::sin(initial_lat)), 1.5 );
            double R2 = a/std::pow(1-e2*(std::sin(initial_lat)*std::sin(initial_lat)), 0.5);

            double distance_north_meters = R1*dlat;
            double distance_east_meters = R2*std::cos(initial_lat)*dlong;

            px = distance_north_meters;
            py = distance_east_meters;
        }

        mag.data[0] = magx;
        mag.data[1] = magy;
        mag.data[2] = magz;

        // Measurement vectors Z
        Vector3 zx = {px, 0, ax};
        Vector3 zy = {py, 0, ay};
        Vector3 zz = {pz, 0, az};

        // Measurement uncertainty R
        Rx.data[0][0] = varpx;
        Rx.data[1][1] = 100; // don't measure vx, vy, or vz, make variance big
        Rx.data[2][2] = varax;

        Ry.data[0][0] = varpy;
        Ry.data[1][1] = 100;
        Ry.data[2][2] = varay;

        Rz.data[0][0] = varpz;
        Rz.data[1][1] = 100;
        Rz.data[2][2] = varaz;

       // Extrapolate state
       Vector3 x_n1_n = F.multiply(x);
       Vector3 y_n1_n = F.multiply(y);
       Vector3 z_n1_n = F.multiply(z);

       Matrix3x3 P_x_n1_n = F.multiply(P_x_nn).multiply(F.transpose()).add(Q);
       Matrix3x3 P_y_n1_n = F.multiply(P_y_nn).multiply(F.transpose()).add(Q);
       Matrix3x3 P_z_n1_n = F.multiply(P_z_nn).multiply(F.transpose()).add(Q);

       predict(zx, zy, zz, // measurements
            x_n1_n, y_n1_n, z_n1_n, // state predictions for n+1 from n
            P_x_n1_n, P_y_n1_n, P_z_n1_n); // Process
    }

    void predict(Vector3 zx, Vector3 zy, Vector3 zz, 
        Vector3 x_n1_n, Vector3 y_n1_n, Vector3 z_n1_n, 
        Matrix3x3 P_x_n1_n, Matrix3x3 P_y_n1_n, Matrix3x3 P_z_n1_n) {

        
        // Compute Kalman Gain
        Matrix3x3 Knx = P_x_nm1_n.multiply(H.transpose()).multiply( H.multiply(P_x_nm1_n).multiply(H.transpose()).add(Rx).inverse());
        Matrix3x3 Kny = P_y_nm1_n.multiply(H.transpose()).multiply( H.multiply(P_y_nm1_n).multiply(H.transpose()).add(Ry).inverse());
        Matrix3x3 Knz = P_z_nm1_n.multiply(H.transpose()).multiply( H.multiply(P_z_nm1_n).multiply(H.transpose()).add(Rz).inverse());
        
        // Update estimate
        x = xold.add(Knx.multiply(zx.subtract(H.multiply(xold))));
        y = yold.add(Kny.multiply(zy.subtract(H.multiply(yold))));
        z = zold.add(Knz.multiply(zz.subtract(H.multiply(zold))));



        P_x_nn = (Matrix3x3::Identity().add(Knx.multiply(H.scalarMult(-1)))).multiply(P_x_nm1_n).multiply(Matrix3x3::Identity().add(Knx.multiply(H.scalarMult(-1))).transpose()) ;
        P_x_nn = P_x_nn.add(Knx.multiply(Rx.add(Ry).scalarMult(0.5)).multiply(Knx.transpose()));

        P_y_nn = (Matrix3x3::Identity().add(Kny.multiply(H.scalarMult(-1)))).multiply(P_y_nm1_n).multiply(Matrix3x3::Identity().add(Kny.multiply(H.scalarMult(-1))).transpose()) ;
        P_y_nn = P_y_nn.add(Kny.multiply(Ry).multiply(Kny.transpose()));

        P_z_nn = (Matrix3x3::Identity().add(Knz.multiply(H.scalarMult(-1)))).multiply(P_z_nm1_n).multiply(Matrix3x3::Identity().add(Knz.multiply(H.scalarMult(-1))).transpose()) ;
        P_z_nn = P_z_nn.add(Knz.multiply(Rz).multiply(Knz.transpose()));

        // Update old variables
        P_x_nm1_n = P_x_n1_n;
        P_y_nm1_n = P_y_n1_n;
        P_z_nm1_n = P_z_n1_n;

        xold = x_n1_n;
        yold = y_n1_n;
        zold = z_n1_n;
    };

    double getPositionX() {
        return x.data[0];
    }
    double getPositionY() {
        return y.data[0];
    }
    double getPositionZ() {
        return z.data[0];
    }

    double getVelocityX() {
        return x.data[1];
    }
    double getVelocityY() {
        return y.data[1];
    }
    double getVelocityZ() {
        return z.data[1];
    }

    double getAccelerationX() {
        return x.data[2];
    }
    double getAccelerationY() {
        return y.data[2];
    }
    double getAccelerationZ() {
        return z.data[2];
    }

    double getMagx() {
        return mag.data[0];
    }
    double getMagy() {
        return mag.data[1];
    }
    double getMagz() {
        return mag.data[2];
    }
    

private:
    std::chrono::steady_clock::time_point last_update;

    void tick() {
        auto now = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = now - last_update;
        dt = elapsed.count();   // seconds
        last_update = now;

        // update F
        // state transition matrix, same for xz
        // x_(k+1^-) = F x_(k^+)
        F = {{
            {1, dt, 0.5 * dt * dt},
            {0, 1, dt},
            {0, 0, 1}
        }};

        double dt2 = dt*dt;
        double dt3 = dt2*dt;
        double dt4 = dt2*dt2;
        double dt5 = dt3*dt2;

        // update Q
        Q = {{
            {dt5/20, dt4/8, dt3/6},
            {dt4/8, dt3/3, dt2/2},
            {dt3/6, dt2/2, dt}
        }};
        
        Q = Q.scalarMult( ACCVARIANCE );
        
    }


    // Darrell custom calibration stuff for magnetometer. Will need to be redone for every drone, every time the setup is changed
    // Use magneto software to build A and b
    const double A[3][3] = {
        { 1,  0.018, 0.025637 },
        { 0.02919,  1, -0.01586 },
        { -0.0256, -0.158, 1 }
    };
    const double b[3] = {
        -250.7,
        -270,
        -1409
    };
    
    void calibrateMag(double uncalib[3], double calib[3]) {
        double h[3] = {
            uncalib[0] - b[0],
            uncalib[1] - b[1],
            uncalib[2] - b[2]
        };

        for (int i = 0; i < 3; i++) {
            calib[i] = 0.0;
            for (int j = 0; j < 3; j++) {
                calib[i] += A[i][j] * h[j];
            }
        }
    }

    const double DEG2RAD = 3.14159265358979323846 / 180.0;

    void deadband(double ax, double ay, double az) {
        if (ax<IMUDEADBAND) {
            ax = 0;
        }
        if (ay<IMUDEADBAND) {
            ay = 0;
        }
        if (az<IMUDEADBAND) {
            az = 0;
        }
    }
};
