#pragma once

#include <fstream>
#include "types/SensorValues.hpp"

// outs csv for analysis
class SensorOutput {
   public:
      SensorOutput() : counter(0), enabled(false) {}
      void enable();
      void log(SensorValues raw, SensorValues filtered);
   private:
      std::ofstream csv;
      int counter;
      bool enabled;
};
