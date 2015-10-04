#include "motion/MotionOdometry.hpp"

#include "utils/body.hpp"
#include "motion/touch/SensorOutput.hpp"

#define ALPHA 0.3f
#define SLIP_AMOUNT 0.005f          // roughly 30 degrees per second
#define PESSIMISTIC_SCALE 0.95f     // never over report to localisation
#define SENSOR_LAG 10

using namespace std;

MotionOdometry::MotionOdometry() : isEnabled(false) {
    reset();
}

Odometry MotionOdometry::updateOdometry(const SensorValues& sensors, Odometry walkChange) {
    // older v4s don't have gyroZ
    float gyroZ = sensors.sensors[Sensors::InertialSensor_GyroscopeZ];
    if (abs(gyroZ) > 0) {
        isEnabled = true;
    }
    // try to get the readings to match phase
    Odometry laggedWalkChange = walkChangeBuffer.back();
    walkChangeBuffer.insert(walkChangeBuffer.begin(), walkChange);
    walkChangeBuffer.pop_back();
    if (isEnabled) {
        // convert to radians per frame, same direction as walkChange
        gyroZ *= -0.01f * PESSIMISTIC_SCALE;

        float slipError = gyroZ - laggedWalkChange.turn;
        slipAverage = ALPHA*slipError + (1.f - ALPHA)*slipAverage;

        // use gyroZ if there is sufficient difference, don't always use it due to gyro drift skewing the result
        if (fabs(slipAverage) > SLIP_AMOUNT){
            laggedWalkChange.turn = gyroZ;
        }
    }
    return laggedWalkChange;
}

void MotionOdometry::reset() {
    slipAverage = 0;
    walkChangeBuffer.clear();
    while (walkChangeBuffer.size() < SENSOR_LAG) {
        walkChangeBuffer.push_back(Odometry());
    }
}
