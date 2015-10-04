#pragma once

#include "types/Point.hpp"
#include "types/BBox.hpp"
#include <iostream>

struct FootInfo
{
   FootInfo (){}
   FootInfo (BBox& _a, BBox& _b, int _age):robotBounds(_a), imageBounds(_b), age(_age){}
   bool operator==(const FootInfo &other) const{
	   return 	imageBounds == other.imageBounds
	   	   && 	robotBounds == other.robotBounds
	   	   &&   age 		== other.age;
   }
   BBox robotBounds, imageBounds;
   int age;

   template <class Archive>
   void serialize(Archive &ar, const unsigned int file_version){
	   ar & robotBounds;
	   ar & imageBounds;
	   ar & age;
   }
};

inline std::ostream& operator<<(std::ostream& os, const FootInfo& f) {
   os << "Age: " << f.age << " RobotBB [ " << f.robotBounds << " ] ImageBB [ " << f.imageBounds << " ]";
   return os;
}
