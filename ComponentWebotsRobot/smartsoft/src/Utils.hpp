#include <cmath>

#ifndef _CAMERARECOGNITIONUTILS_HH
#define _CAMERARECOGNITIONUTILS_HH

struct Quaternion {
    Quaternion();
    Quaternion(double w, double x, double y, double z) :
        w(w), x(x), y(y), z(z){};
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q); 

#endif