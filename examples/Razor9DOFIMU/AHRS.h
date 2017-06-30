//Declination in degrees in Shalimar, FL
#define DECLINATION 3.183333

double radToDeg(double rad);
double degToRad(double deg);
double invSqrt(double x);
void eulerToQuaternion();
void quaternionToEuler();
void normalizeQuaternion();
void initializeQuaternion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

extern double q0, q1, q2, q3;
extern double radRoll, radPitch, radYaw;
extern double degRoll, degPitch, degYaw;