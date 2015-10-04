#include "FootDetection.hpp"
#include "Ransac.hpp"
#include <vector>
#include <iostream>
#include <map>
#include <cstdlib>
#include <cmath>
#include <climits>
#include <algorithm>
#include <deque>
#include <utility>
#include <queue>
#include <set>
#include "utils/Timer.hpp"
#define ANGLE_THRESHOLD             0.610865238
#define EDGE_THRESHOLD  350000
//	#define EDGE_THRESHOLD 1000

using std::vector;
using std::cout;
using std::cerr;
using std::endl;
using std::pair;
using std::min;
using std::max;
using std::deque;
class Vision;

bool bucketCompare(const Bucket &a, const Bucket& b){
	return a.midPoint.y() > b.midPoint.y();
}

bool ptsCompare(const Point &a, const Point& b){
	return a.x() < b.x();
}

void FootDetection::createBoxes(VisionFrame& frame, const Fovea& fovea,vector<BBox> & sets){
	for (vector<BBox>::iterator it = sets.begin(); it != sets.end(); ++it){

		Point left = frame.cameraToRR.pose.imageToRobotXY(fovea.mapFoveaToImage(it->a));
		//minus 3 here to add an error margin of 3cm
		Point right = frame.cameraToRR.pose.imageToRobotXY(fovea.mapFoveaToImage(it->b));
		right.x() -= 20;

		BBox robotBounds(left, right);
		BBox imageBounds(fovea.mapFoveaToImage(it->a), fovea.mapFoveaToImage(it->b));
		frame.feetBoxes.push_back(FootInfo(robotBounds, imageBounds, 0));
		//Uncomment for debugging
		/*for (int i = it->p.x(); i < it->rrp.x(); ++i){
			footPoints.push_back((fovea.mapFoveaToImage(Point(i, it->p.y())), Point(0,0)));
			footPoints.push_back((fovea.mapFoveaToImage(Point(i, it->rrp.y())), Point(0,0)));
		}
		for (int i = it->p.y(); i < it->rrp.y(); ++i){
			footPoints.push_back((fovea.mapFoveaToImage(Point(it->rrp.x(), i)), Point(0,0)));
			footPoints.push_back((fovea.mapFoveaToImage(Point(it->p.x(), i)), Point(0,0)));
		}*/
	}
}

struct locInfo{
	int x, y, h;
};

const int heightReq = 30;
const int downHeightBounds = 2;
const int upHeightBounds = 4;
const int widthBounds = 6;
void FootDetection::keepNearHigh(VisionFrame &frame, const Fovea& fovea, vector<Point>& pts){
	vector<locInfo> locInfoVec;
	locInfoVec.reserve(pts.size());
	for (unsigned i = 0; i < pts.size(); ++i){
		Point foveaLoc = pts[i];
		locInfoVec[i].x = foveaLoc.x();
		locInfoVec[i].y = foveaLoc.y();
		int y = foveaLoc.y();
		int height = 3;
		y -= 3;
		while (y > 0){
			int c = fovea.colour(Point(foveaLoc.x(),y));
			if (c == cFIELD_GREEN){
				break;
			}
			--y;
			++height;
		}
		locInfoVec[i].h = height;
	}
	std::vector<Point> keepPoints;
	for (unsigned i = 0; i < pts.size(); ++i){
		for (unsigned j = 0; j < pts.size(); ++j){
			if (locInfoVec[j].h > heightReq){
				if (locInfoVec[i].x >= locInfoVec[j].x - widthBounds
						&& locInfoVec[i].x <= locInfoVec[j].x + widthBounds){
					if (locInfoVec[i].y >= locInfoVec[j].y - upHeightBounds &&
											locInfoVec[i].y <= locInfoVec[j].y + downHeightBounds){
						keepPoints.push_back(Point(locInfoVec[i].x, locInfoVec[i].y));
						break;
					}
				}
			}
		}
	}
	pts.clear();
	pts = keepPoints;
}


void FootDetection::findFeet(VisionFrame& frame, const Fovea& botFovea, unsigned int * seed) {
	if (visited == NULL){
		//first time setup;
		width = botFovea.bb.width();
		height = botFovea.bb.height();
		visited = new bool*[width]();
		for (int i = 0; i < width; ++i){
			visited[i] = new bool[height]();
		}
	}
    //clear any existing information
	frame.feetBoxes.clear();
	debugPoints.clear();
    vector<Point> pts;

    findFoveaPoints(frame, botFovea, pts);
    for(unsigned i = 0; i < pts.size(); ++i){
    	pts[i] = botFovea.mapFoveaToImage(pts[i]);
    }
    findLines(frame, seed, pts);
    for(unsigned i = 0; i < pts.size(); ++i){
    	pts[i] = botFovea.mapImageToFovea(pts[i]);
    }

    keepNearHigh(frame, botFovea, pts);

    reducePoints(pts);

    vector<BBox> bounds;
    vector<Bucket> groups;
    std::sort(pts.begin(), pts.end(), ptsCompare);

    groups = bucketPoints(frame, pts);

    pts.clear();
    for (unsigned int i = 0; i < groups.size(); ++i){
      	groups[i].setMidPoint();
    }
    std::sort(groups.begin(), groups.end(), bucketCompare);
    for(unsigned int i = 0; i < groups.size(); ++i){
    	if (groups[i].points.size() >= 2){
			int error = 0;
			BBox bound = floodFill(botFovea, groups[i].midPoint, error);
			if (error != 1){
				bounds.push_back(bound);
			}
    	}
    }

    for (int i = 0; i < width; ++i){
    	for (int j = 0; j < height; ++j){
    		visited[i][j] = false;
    	}
    }
   createBoxes(frame, botFovea, bounds);
   /*int size = frame.feetBoxes.size();
   for (std::deque<std::pair<, int> >::iterator it = oldFeetBoxes.begin(); it != oldFeetBoxes.end(); ++it){
   	it->second += 1;
   }
   if (oldFeetBoxes.size()  > 0){
	   int age = oldFeetBoxes.front().second;
	   while (age == 10){
		   oldFeetBoxes.pop_front();
		   if (oldFeetBoxes.size() > 0){
			   age = oldFeetBoxes.front().second;
		   } else {
			   age = 0;
		   }
	   }
   }
   for (deque<std::pair<s, int> >::iterator it = oldFeetBoxes.begin(); it != oldFeetBoxes.end(); ++it){
 	   frame.feetBoxes.push_back(it->first);
   }
   for (int i = 0; i < size; ++i){
	   oldFeetBoxes.push_back(std::make_pair(frame.feetBoxes[i], 1));
   }*/
}


void FootDetection::findLines(VisionFrame &frame, unsigned int* seed, vector<Point> & pts){
    RANSACLine resultLine(Point(0,0), Point(0,0));
    // RANSAC variables
    //k max number of iterations
    uint16_t k = 20;
    //maximum distance a point can be from the concensus set
    float e = 4;
    //minimum number of points in the concensus set
    uint16_t n = 20;
    vector<bool> *con, consBuf[2];
    consBuf[0].resize(pts.size());
    consBuf[1].resize(pts.size());
    con = &consBuf[0];

    while (n > 12){
    	//find lines in the image
        if (RANSAC::findLine(pts, &con, resultLine, k, e, n, consBuf, seed)){
            //remove points from the set of points

            int count = pts.size() - 1;
            for (vector<bool>::reverse_iterator it = con->rbegin(); it != con->rend(); ++it){
                if (*it == 1){
                    vector<Point>::iterator pointIt = pts.begin()+count;
                    pts.erase(pointIt);
                }
                if (count == 0){
                    break;
                }
                --count;
            }
            consBuf[0].clear();
            consBuf[1].clear();
            consBuf[0].resize(pts.size());
            consBuf[1].resize(pts.size());
        } else {
            --n;
        }
        if (n < 16){
        	e = 0;
        }
    }
}
const int fillCutoff = 5;
const int fillHeightMax = 15;
const int fillDepth = 10;
const int fillWidth = 13;

BBox FootDetection::floodFill(const Fovea& fovea,  Point& start, int &err){
	Point foveaLoc =start;
	if (fovea.colour(foveaLoc) == cFIELD_GREEN){
		int p = foveaLoc.x();
		int q = foveaLoc.y();
		bool found = false;
		if (p + 2 < fovea.bb.width()){
			if (fovea.colour(p + 2, q) != cFIELD_GREEN){
				foveaLoc.x() = p + 2;
				foveaLoc.y() = q;
				found = true;
			}
		}
		if (!found && p - 2 >= 0){
			if (fovea.colour(p - 2, q) != cFIELD_GREEN){
				foveaLoc.x() = p - 2;
				foveaLoc.y() = q;
				found = true;
			}
		}
		if (!found){
			if (fovea.colour(p, q - 2) != cFIELD_GREEN){
			foveaLoc.x() = p;
			foveaLoc.y() = q - 2;
			}
		}
	}
	int y = foveaLoc.y();
	int x = foveaLoc.x();
	std::queue<pair<int, int> > q;
	pair<int, int> cur(x, y);
	pair<int, int> org = cur;
	q.push(cur);
	visited[cur.first][cur.second] = true;
	int minX = INT_MAX, minY = INT_MAX, maxX = 0, maxY = 0;
	while(!q.empty()){
		cur = q.front();
		q.pop();
		minX = min(minX, cur.first);
		minY = min(minY, cur.second);
		maxX = max(maxX, cur.first);
		maxY = max(maxY, cur.second);
		//get neighbors
		if (cur.second > 0 && cur.second > y - fillHeightMax){
			pair<int, int> next = cur;
			--next.second;
			int c = fovea.colour(next.first, next.second);
			if (c != cFIELD_GREEN && c != cBALL){
				if (!visited[next.first][next.second]){
					q.push(next);
					visited[next.first][next.second] = true;
				}
			}
		}
		if (cur.second < fovea.bb.height() - 1 && cur.second < y + fillDepth){
			pair<int, int> next = cur;
			++next.second;
			int c = fovea.colour(next.first, next.second);
			if (c != cFIELD_GREEN && c != cBALL){
				if (!visited[next.first][next.second]){
					q.push(next);
					visited[next.first][next.second] = true;
				}
			}
		}
		if (cur.second > y - fillCutoff){
			if (cur.first > 0 && cur.first > x - fillWidth){
				pair<int, int> next = cur;
				--next.first;
				int c = fovea.colour(next.first, next.second);
				if (c != cFIELD_GREEN && c != cBALL){
					if (!visited[next.first][next.second]){
						q.push(next);
						visited[next.first][next.second] = true;
					}
				}
			}

			if (cur.first < fovea.bb.width() - 1 && cur.first < x + fillWidth){
				pair<int, int> next = cur;
				++next.first;
				int c = fovea.colour(next.first, next.second);
				if (c != cFIELD_GREEN && c != cBALL){
					if (!visited[next.first][next.second]){
						q.push(next);
						visited[next.first][next.second] = true;
					}
				}
			}
		}
	}
	if (org == cur){
		err = 1;
	}
	return BBox(Point(minX, minY), Point(maxX, maxY));
}


void FootDetection::reducePoints(vector<Point> & pts){
    std::map<int, Point> points;

    //Remove everything but the bottomest y
    for (vector<Point>::iterator it = pts.begin(); it != pts.end(); ++it){
        if (points.find(it->x()) != points.end()){
            //compare
            Point old = points[it->x()];
            if (old.y() < it->y()){
                points[it->x()] = *it;
            }

        } else {
            //insert new element
            points[it->x()] = *it;
        }
    }
    pts.clear();
    for (std::map<int, Point>::iterator it = points.begin(); it != points.end(); ++it){
        pts.push_back(it->second);
    }
}
const int yVar = 5;
const int xVar = 2;
vector<Bucket> FootDetection::bucketPoints(VisionFrame &frame, vector<Point> & pts){
	vector<Bucket> result;
	if (pts.size() <= 2){
		return result;
	}
	Point* previous;
	Point* current;

	previous = &pts[0];
	Bucket curBucket;
	curBucket.points.push_back(*previous);
	for(unsigned int i = 1; i < pts.size(); ++i){
		current = &pts[i];
		if (abs(current->x() - previous->x()) < xVar && abs(current->y() - previous->y()) < yVar){
			curBucket.points.push_back(*current);
		} else{
			result.push_back(curBucket);
			curBucket = Bucket();
			curBucket.points.push_back(*current);
		}
		previous = current;
	}

	result.push_back(curBucket);
	return result;
}



void FootDetection::findFoveaPoints(VisionFrame &frame, const Fovea& fovea, vector<Point> &pts) {
    // Vertical Scan
    Point start;
    const int cols = fovea.bb.width();
    const int rows = fovea.bb.height();
    bool next = false;
    for (int i = 0; i < cols; i++) {
        start = fovea.mapFoveaToImage(Point(i, 0));
        start.y() = std::max(start.y(), frame.botStartScanCoords[start.x()]);
        start = fovea.mapImageToFovea(start);
        next = false;
        for (int j = start.y() + TOP_CUTOFF; j < rows - BOT_CUTOFF; j++) {

            int dx = fovea.edge(i, j).x();
            int dy = fovea.edge(i, j).y();
            float magnitude = (dx * dx) + (dy * dy);
            // Only consider points with a strong edge
            if (magnitude > EDGE_THRESHOLD) {
                next = true;
            } else if (next) {
            	next = false;
            	pts.push_back(Point(i,j));
            }
        }
    }
}
