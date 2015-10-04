#pragma once
#include <vector>
#include <deque>

#include "VisionConstants.hpp"
#include "Fovea.hpp"

#include "VisionDefs.hpp"
#include "types/Point.hpp"
#include "types/BBox.hpp"
#include "CameraToRR.hpp"
#include "types/FootInfo.hpp"
#include "types/FieldFeatureInfo.hpp"

#define TOP_CUTOFF 20
#define BOT_CUTOFF 1
#define BOTTOM_CAMERA_HEIGHT 700
#define BOTTOM_CAMERA_WIDTH 900

struct Bucket{
	std::vector<Point> points;
	Point midPoint;
	void setMidPoint(){
		int x = 0;
		int y = 0;
		int total = 0;
		for (std::vector<Point>::iterator it = points.begin(); it != points.end(); ++it){
			x += it->x();
			y += it->y();
			++total;
		}
		x /= total;
		y /= total;
		midPoint = Point(x, y);
	}
};

class FootDetection {
   public:
	  FootDetection():visited(NULL), width(0), height(0){
	  }
	  ~FootDetection(){
		  for (int i = 0; i < width; ++i){
		    	delete[] visited[i];
		  }
		  delete[] visited;
	  }
      void findFoveaPoints(VisionFrame &frame,
              const Fovea &fovea, std::vector<Point> &pts);
      //passes a vector containing locations of feet to the visionframe
      void findFeet(VisionFrame &frame,
              const Fovea &fovea,
			  unsigned int *seed);
      BBox floodFill(const Fovea & fovea, Point& start, int & err);

      void createBoxes(VisionFrame& frame, const Fovea& fovea,std::vector<BBox> & sets);

      void keepNearHigh(VisionFrame &frame, const Fovea& fovea, std::vector<Point>& pts);

      //removes any point that isn't at the bottom
      void reducePoints(std::vector<Point> & pts);

      void findLines(VisionFrame &frame, unsigned int* seed, std::vector<Point> & pts);

      std::vector<Bucket> bucketPoints(VisionFrame &frame, std::vector<Point> & pts);
      bool **visited;
      std::vector<Point> debugPoints;
   private:
      int width;
      int height;
};
