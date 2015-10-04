#include "motion/touch/SensorOutput.hpp"

using namespace std;

void SensorOutput::enable() {
   csv.open("sensors.csv");
   csv << "accRaw;accOurs;accTheirs;gyrRaw;gyrOurs;gyrTheirs;angleOurs;angleTheirs" << endl;
//   csv << "raw;smoothed" << endl;
   enabled = true;
}

void SensorOutput::log(SensorValues raw, SensorValues filtered) {
   if (enabled) {
      if (counter < 20000) {
            csv << raw.sensors[Sensors::InertialSensor_AccY] << ";" << filtered.sensors[Sensors::InertialSensor_AccY] << ";" << raw.sensors[Sensors::InertialSensor_AccelerometerY] << ";"
                << raw.sensors[Sensors::InertialSensor_GyrX] << ";" << filtered.sensors[Sensors::InertialSensor_GyrX] << ";" << raw.sensors[Sensors::InertialSensor_GyroscopeX] << ";"
                << filtered.sensors[Sensors::InertialSensor_AngleX] << ";" << raw.sensors[Sensors::InertialSensor_AngleX] << endl;
//         csv << raw.sensors[Sensors::InertialSensor_GyroscopeZ] << ";" << filtered.sensors[Sensors::InertialSensor_GyroscopeZ] << endl;
         counter++;
      } else {
         csv.close();
      }
   }
}
