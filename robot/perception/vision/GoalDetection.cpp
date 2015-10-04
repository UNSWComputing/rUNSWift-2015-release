#include "GoalDetection.hpp"
#include "Vision.hpp"

#include <vector>
#include <algorithm>
#include <utility>
#include "utils/Logger.hpp"
#include "utils/basic_maths.hpp"
#include "utils/SPLDefs.hpp"
#include "utils/Timer.hpp"

#define MIN_COLOUR_THRESHOLD 30
#define MIN_H_COLOUR_THRESHOLD 30
#define MIN_EDGE_THRESHOLD 35000
#define COLOUR_RATIO_THRESHOLD 0.75

using namespace std;

void GoalDetection::findGoals(
VisionFrame    &frame,
std::vector<Point> fEP,
const Fovea    &fovea,
unsigned int   *seed)
{

    goalFoveas.clear();
    std::vector<PostInfo> posts;
    goalPostLines.clear();
    // Find candidate regions to look for goals
    std::vector<BBox> regions;
    postSearch(frame,fEP,fovea,regions);
    // Sanity check goals to eliminate crappy ones


    // Convert into goal posts
    std::vector<BBox>::const_iterator it;
    for (it = regions.begin(); it != regions.end(); ++it) {

        PostInfo p;
        BBox bb (fovea.mapFoveaToImage(it->a), fovea.mapFoveaToImage(it->b));

        // If we are in the top camera, ensure we don't round the BBox into the bottom camera
        if (fovea.top && bb.b.y() == TOP_IMAGE_ROWS) {
            bb.b.y() = (TOP_IMAGE_ROWS-1);
        }
        float wdistance = adjustSize(frame,fovea,bb,regions.size(),p);
        std::vector<BBox> bb2;
        bb2.push_back(BBox(fovea.mapImageToFovea(bb.a),fovea.mapImageToFovea(bb.b)));
        //performSanityChecks(fovea, frame, bb2,true);
        // Calculate best distance to goal - also fine tunes the BBox
        // Fine tunes and sets the BBox
        // Sets rr, kDistance, wDistance, trustDistance on variable p
        if(bb2.size() != 0){
            RRCoord rr = findDistanceToPost(frame, fovea, bb, regions.size(), p,wdistance);
            //if (rr.distance() > 2000) p.trustDistance = false;

            // Work out left and right goals
            // Sets type and dir
            determineLeftRightPost(fovea, regions, *it, p);

            // Save goal post
            posts.push_back(p);
        }
    }

    // Sanity check the LHS / RHS of goals
    if (posts.size() == 2) {
        PostInfo pLeft, pRight;
        if (posts[0].type == PostInfo::pLeft) {
            pLeft  = posts[0];
            pRight = posts[1];
        } else {
            pLeft  = posts[1];
            pRight = posts[0];
        }

        if (pLeft.dir == PostInfo::pToLeftOf &&
        pLeft.rr.distance() >= pRight.rr.distance()) {
            posts[0].dir = PostInfo::pUnknown;
            posts[1].dir = PostInfo::pUnknown;
        } else if (pLeft.dir == PostInfo::pToRightOf &&
        pRight.rr.distance() >= pLeft.rr.distance()) {
            posts[0].dir = PostInfo::pUnknown;
            posts[1].dir = PostInfo::pUnknown;
        }
    }

    // Merge top and bottom cameras
    const float headingThreshold = DEG2RAD(7);
    if (frame.posts.empty()) {
        frame.posts = posts;
    } else if (fovea.top && posts.size() > 0 && frame.posts.size() == 1) {
        for (unsigned int i = 0; i < posts.size(); i++) {
            if (abs(posts[i].rr.heading() - frame.posts[0].rr.heading())
            > headingThreshold) {
                frame.posts.push_back(posts[i]);
            }
        }

        // Ensure if we have 2 posts, they are left and right
        if (frame.posts.size() == 2) {
            if (frame.posts[0].rr.heading() < frame.posts[1].rr.heading()) {
                frame.posts[0].type = PostInfo::pRight;
                frame.posts[1].type = PostInfo::pLeft;
            } else {
                frame.posts[0].type = PostInfo::pLeft;
                frame.posts[1].type = PostInfo::pRight;
            }
        }
    }
}
bool sortBBox (BBox a,BBox b){return a.b.y() - a.a.y() > b.b.y() - b.a.y();};


void GoalDetection::postSearch(const VisionFrame &frame,std::vector<Point> fEP,const Fovea &fovea, std::vector<BBox> &regions){
    regions.clear();
    std::vector<Point> postLines;
    std::vector<BBox> candidates(regions);

    std::vector<Point>::iterator i;
    for (i  = fEP.begin(); i != fEP.end(); i++){
        Point ffe = fovea.mapImageToFovea(*i);
        Point ffe2 = ffe;
        Point ffe3 = ffe;
        if(ffe2.y() < fovea.bb.height() - 1){
        	ffe2.y() = ffe.y() - 1;
        }
        if(ffe3.y() > 0){
        	ffe3.y() = ffe.y() + 1;
        }
        Colour c = fovea.colour(ffe);
        Colour c2 = fovea.colour(ffe2);
        Colour c3 = fovea.colour(ffe3);
        while((c == cWHITE || c2 == cWHITE || c3 == cWHITE) && ffe.x() < fovea.bb.width()-1){
            ffe.x()++;
            c = fovea.colour(ffe);
            c2 = c;
            c3 = c;
        }
        Point init = fovea.mapImageToFovea(*i);
        if((ffe.x() - init.x()) >= 3){
            int top = ffe.y() - (ffe.x() - init.x())*9;
            if(top < 0) top = 0;
            candidates.push_back(BBox(Point(init.x(),top),Point(ffe.x(),ffe.y())));
        }
    }

    //Cull out the wider boxes
    std::vector<BBox>::iterator it;
    for(it = candidates.begin(); it != candidates.end(); it++){
        int thisx, nextx,thisy,nexty = 0;
        thisx = it->b.x();
        thisy = it->b.y();
        it++;
        if(it != candidates.end()){
            nextx = it->b.x();
            nexty = it->b.y();
            while(thisx >= nextx && thisy >= nexty && it != candidates.end()){
                it++;
                if(it != candidates.end()) {
                	nexty = it->b.y();
                	nextx = it->b.x();
                }
            }



        }
        it--;
        regions.push_back(*it);
    }
    goalPostLines.push_back(postLines);
    performSanityChecks(fovea, frame, regions,false);

}



void GoalDetection::performSanityChecks(const Fovea &fovea,
const VisionFrame &frame,
std::vector<BBox> &regions,bool secondCheck) {
    std::vector<BBox> candidates (regions);
    regions.clear();
    std::vector<BBox>::iterator it;
    for (it = candidates.begin(); it != candidates.end(); ++it) {
        int centre = (it->a.x() + it->b.x()) / 2;
        // Check that the middle and the bottom 75% for edges
        // Only need 1 strong edge in each to be good
        // Check bottom of goal post is below field edge

        Point fieldEdge = fovea.mapFoveaToImage(Point(centre, 0));
        int fieldEdgeY = 0;
        if (fovea.top) {
            fieldEdgeY = frame.topStartScanCoords[fieldEdge.x()];
        } else {
            fieldEdgeY = frame.botStartScanCoords[fieldEdge.x()];
        }
        fieldEdge.y() = std::max(fieldEdge.y(), fieldEdgeY);
        fieldEdge = fovea.mapImageToFovea(fieldEdge);
        float edgeAbove = 0.0;
        float percentAbove = 0.0;
        float length = (it->b.y()-it->a.y());

        if (fieldEdge.y() > it->b.y()) {
            //std::cout << "throwing away since above field edge" << std::endl;
            continue;
        }
        if (fieldEdge.y() < it->a.y()){
            //throwing away because top
            continue;
        }
        edgeAbove = fieldEdge.y()-it->a.y();
        percentAbove = edgeAbove/(length);

        if (percentAbove < 0.70){
            continue;
        }
        //int bbLeftEdge, bbRightEdge = 0;
        //if (fovea.top) {
        //   bbLeftEdge = frame.topStartScanCoords[it->a.x()];
        //   bbRightEdge = frame.topStartScanCoords[it->b.x()];
        //} else {
        //	 bbLeftEdge = frame.botStartScanCoords[it->a.x()];
        //	 bbRightEdge = frame.botStartScanCoords[it->b.x()];
        //}

        //int highEdge = std::min(bbLeftEdge,bbRightEdge);
        //int middle = (highEdge + it->b.y()) / 2;
        //int bottom = highEdge + (it->b.y() - highEdge) * 0.75;
        int middle = it->b.y() - (it->b.y() - it->a.y())*0.5;
        int bottom = it->b.y() - (it->b.y() - it->a.y())*0.75;
        bool keepM = false;
        bool keepB = false;
        bool keepBase = false;

        for (int col = it->a.x(); col < it->b.x(); ++col) {

            // Check middle
            Point edge = fovea.edge(col, middle);
            float magnitudex = edge.x();
            float magnitudey = edge.y();
            int baseAbove = it->b.y();
            int baseBelow = it->b.y();
            if (magnitudex > 100) keepM = true;

            // Check bottom
            edge = fovea.edge(col, bottom);
            magnitudex = edge.x();

            if (magnitudex > 100) keepB = true;


            edge = fovea.edge(col, it->b.y());
            magnitudey = edge.y();
            if(magnitudey > 100) keepBase = true;
            if(baseAbove >= 0){
            	edge = fovea.edge(col, baseAbove);
            }
            magnitudey = edge.y();
            if(magnitudey > 100) keepBase = true;
            if(baseBelow <= fovea.bb.height() - 1){
            	edge = fovea.edge(col, baseBelow);
            }
            magnitudey = edge.y();
            if(magnitudey > 100) keepBase = true;
        }
        if (!keepM) {
            // std::cout << "throwing away since no keepM" << std::endl;
            continue;
        }
        if (!keepB) {
            //std::cout << "throwing away since no keepB" << std::endl;
            continue;
        }
        if(!keepBase){
            //cout << "Throwin on base" <<endl;
            continue;
        }
        // Check % of colour in goal post - ie compare length and colour
        //float length = (it->b.y()-it->a.y());
        float width = it->b.x() - it->a.x();
        float numColourPixels = 0;
        for (int row = it->b.y(); row > it->b.y()-(int)(length/4); --row) {
            if (fovea.colour(centre, row) == cWHITE) {
                ++numColourPixels;
            }
            if (fovea.colour(centre,row) == cTEAM_AWAY || fovea.colour(centre,row) == cTEAM_HOME){
                continue;
            }
            if (fovea.colour(it->b.x(),row) == cTEAM_AWAY || fovea.colour(it->b.x(),row) == cTEAM_HOME){
                continue;
            }
            if (fovea.colour(it->a.x(),row) == cTEAM_AWAY || fovea.colour(it->a.x(),row) == cTEAM_HOME){
                continue;
            }
        }

        //Bottom quarter of the post must be white and contain no jerseys
        if ((numColourPixels / (int)(length/4)) < 0.8) {
            //std::cout << "throwing away since not enough colour " << numColourPixels << "/" << length << std::endl;
            continue;
        }


        for (int row = it->b.y()-(length/4); row != it->b.y()-length; --row) {
            if (fovea.colour(centre, row) == cWHITE) {
                ++numColourPixels;
            }
            if (fovea.colour(centre,row) == cTEAM_AWAY || fovea.colour(centre,row) == cTEAM_HOME){
                continue;
            }
            if (fovea.colour(it->b.x(),row) == cTEAM_AWAY || fovea.colour(it->b.x(),row) == cTEAM_HOME){
                continue;
            }
            if (fovea.colour(it->a.x(),row) == cTEAM_AWAY || fovea.colour(it->a.x(),row) == cTEAM_HOME){
                continue;
            }
        }
        //Remainder of the post should be 75% white and contian no jerseys
        float colourRatio = numColourPixels / length;
        if (colourRatio < COLOUR_RATIO_THRESHOLD) {
            //std::cout << "throwing away since not enough colour " << numColourPixels << "/" << length << std::endl;
            continue;
        }
        numColourPixels = 0;
        for (int row = it->b.y(), height = it->b.y() + width; row < height && row <fovea.bb.height() ;row++ ) {
            if (fovea.colour(centre, row) == cFIELD_GREEN) {
                ++numColourPixels;
            }

        }
        colourRatio = numColourPixels/width;
        if (colourRatio < 0.25) {
            //std::cout << "throwing away since not enough green below " << numColourPixels << "/" << length << std::endl;
            continue;
        }
        regions.push_back(*it);
    }
    if(!secondCheck){
        if(regions.size() == 1){
            findOther(fovea,frame,regions);
        }
    }
    // If more than 2 goals, things have gone wrong, so panic
    if (regions.size() > 2) {
        if(secondCheck){
            regions.resize(1);
        } else {
            //std::sort (regions.begin(),regions.end(),sortBBox);
            regions.clear();
        }
    }
    //check they're not overlapping, and if they are take the left one(generally more accurate than the right one)
    if (regions.size() == 2) {
    	BBox left = regions.at(0);
    	BBox right = regions.at(1);
    	if (left.b.x() > right.a.x()){
    		regions.resize(1);
    	} else if(right.a.x() - left.b.x()) {
    		int lengthA = left.b.y() - left.a.y();
    		int lengthB = right.b.y() - right.a.y();
    		BBox big = regions.at(0);
    		BBox small = regions.at(1);
    		bool leftBig = true;
    		if(lengthA <  lengthB){
    			big = regions.at(1);
    			small = regions.at(0);
    			leftBig = false;
    		}
    		int bigWidth = big.b.x() - big.a.x();
    		int dist = right.a.x() - left.b.x();
    		if (bigWidth * 5 > dist){
    			if(leftBig){
    				regions.resize(1);
    			}else{
    				regions.erase(regions.begin());
    			}
    		}


    	}

    }
}
void GoalDetection::findOther(const Fovea &fovea,
const VisionFrame &frame,
std::vector<BBox> &regions){
    BBox goalPost = regions.at(0);
    const XHistogram &xhistogram = fovea.xhistogram;
    const YHistogram &yhistogram = fovea.yhistogram;
    int width = goalPost.b.x() - goalPost.a.x();
    int midPost = (goalPost.b.y() + goalPost.a.y()) * 0.5;
    int wLowThreshold = width * 0.5;
    int wHighThreshold = width * 1.5;
    int hLowThreshold = (goalPost.b.y() - midPost) * 0.5;
    int hHighThreshold = (goalPost.b.y() - midPost) * 1.5;
    for(int i = 0; i < xhistogram.size ; i ++){
        if(i >= goalPost.a.x() - width && i <= goalPost.b.x() + width){
            continue;
        }

        Colour c = fovea.colour(i,midPost);
        if (c == cWHITE){
            int start = i;
            while(c == cWHITE &&  i < xhistogram.size && (!(i > goalPost.a.x() - width && i < goalPost.b.x()+width))){
                i++;
                c = fovea.colour(i,midPost);
            }
            if(i < xhistogram.size) i--;
            if((i-start) > wLowThreshold && (i-start) < wHighThreshold ){
                int base = midPost;
                int newcenter = (i + start) * 0.5;
                c = fovea.colour(newcenter,base);
                while(c == cWHITE && base < yhistogram.size){
                    base++;
                    c = fovea.colour(newcenter,base);
                }
                if((base - midPost) > hLowThreshold && (base - midPost) < hHighThreshold){
                    if(i-1 > start && base-1 > midPost){
                        int top = (base - 1)-(i-start)*9;
                        regions.push_back(BBox(Point(start,top),Point(i,base-1)));
                    }
                }
            }

        }
    }
    performSanityChecks(fovea, frame, regions,true);
}
float GoalDetection::adjustSize(VisionFrame &frame,
const Fovea& fovea,
BBox& goal,
int numPosts,
PostInfo& p){



    findBaseOfPost(frame, goal);

    // **** Try using the width the find the distance ****
    // Calculate width distance at 3 points and take median
    std::map<float, float> distances;
    for (float h = 0.90; h < 1.01; h += 0.05) {
        float d = widthDistanceToPost(frame, goal, h);
        distances.insert(std::make_pair(d,h));
    }
    std::map<float, float>::iterator i = distances.begin();
    if (distances.size() > 2) i= distances.end();
    float wdistance = widthDistanceToPost(frame, goal, i->second, false);
    return wdistance;

}
// Finds the distance to post
// Tunes the BBox using higher resolution foveas
RRCoord GoalDetection::findDistanceToPost(VisionFrame &frame,
const Fovea& fovea,
BBox& goal,
int numPosts,
PostInfo& p,float wdistance) {

    bool trustDistance = true;
    float differenceThreshold = 1.7;


    //widthDistanceToPost(frame, goal, i->second, false);
    // float wdistance = (goal.b.x() - goal.a.x());

    const CameraToRR *convRR = &frame.cameraToRR;
    Point base = Point((goal.a.x()+goal.b.x())/2, goal.b.y());
    RRCoord rr = convRR->convertToRR(base, false);
    float kdistance = rr.distance();
    // **** Decide which distance to use ****
    // Kinematics is usually more accurate, so use it unless we know it's wrong

    // If post ends at bottom of image, probably not the bottom, so use width
    bool width = false;
    if (fovea.top && goal.b.y() > (TOP_IMAGE_ROWS-10)) {
        width = true;
    }

    // If still yellow below the base, probably missed the bottom, so use width
    // Only for 1 post though
    if (numPosts == 1) {
        Point fTop = fovea.mapImageToFovea(goal.a);
        Point fBot = fovea.mapImageToFovea(goal.b);
        const YHistogram &yhistogram = fovea.yhistogram;
        int height = (fBot.y() - fTop.y()) / 4; // set max scan size
        int endPoint = std::min(fovea.bb.height(), fBot.y() + height);
        int noYellow = fBot.y();

        for (int i = fBot.y(); i < endPoint; ++i) {
            int current = yhistogram._counts[i][cWHITE];
            if (current < 10) noYellow = i;
        }
        if (noYellow == fBot.y()) trustDistance = false;
    }

    // Decided to use width distance
    if (width) {
        if (wdistance < 1500) rr.distance() = wdistance;
        else trustDistance = false;
            //      differenceThreshold = 1.7;
    }

    // Check that kinematics and width distances are similar
    else if (kdistance < 2500) {
    }

    else if (((kdistance / wdistance) > differenceThreshold) ||
    ((wdistance / kdistance) > differenceThreshold)) {
        trustDistance = false;
    }

    // Check distance is reasonable
    if (rr.distance() > 12000) {
        trustDistance = false;
        rr.distance() = 12000;
    }

    // Set variables in PostInfo
    p.rr = rr;
    p.kDistance = kdistance;
    p.wDistance = wdistance;
    p.trustDistance = trustDistance;
    p.imageCoords = goal;

    return rr;
}

//Extend base and sides
//Check again for validity?

void GoalDetection::findBaseOfPost(VisionFrame &frame,
BBox& goal) {

    // Calculate where fovea should go
    // Add +-20 pixels around the current base
    int padding = 50;
    int x = (goal.a.x() + goal.b.x())/2;
    Point tl = Point (x-2, goal.b.y() - padding);
    Point br = Point (x+1, goal.b.y() + padding);
    int density = 1;

    // Create a high res fovea
    boost::shared_ptr<FoveaT<hGoals, eGrey> > goalFovea(
    new FoveaT<hGoals, eGrey>(BBox(tl, br), density, 0, true));
    goalFovea->actuate(frame);
    goalFoveas.push_back(goalFovea);
    int centre = goalFovea->bb.width()/2;

    // Find base point
    int lastYellow = -1;
    int maxNotYellow = 5;

    for (int y = 0; y < goalFovea->bb.height(); ++y) {
        Colour c = goalFovea->colour(centre,y);
        if (c == cWHITE) {
            lastYellow = y;
        } else if (abs(lastYellow - y) > maxNotYellow) {
            break;
        }
    }
    if (lastYellow != -1) {
        goal.b.y() = goalFovea->mapFoveaToImage(Point(0, lastYellow)).y();
    }
}

float GoalDetection::widthDistanceToPost(VisionFrame &frame,
BBox& goal,
float h,
bool update) {

    const CameraToRR *convRR = &frame.cameraToRR;
    float distance = -1;

    // Calculate where fovea should go
    int y = goal.a.y() + (goal.b.y() - goal.a.y()) * h;
    //int y = (goal.b.y() - (goal.b.y() - goal.a.y())/20);
    Point tl = Point (goal.a.x()-2, y-1);
    Point br = Point (goal.b.x()+2, y+2);
    int width  = br.x() - tl.x();
    int density = 1;
    // Try and extend fovea a bit
    // Note add extra to left since edge data seems slightly skewed left
    tl.x() = std::max(tl.x() - width/2, 0);
    br.x() = std::min(br.x() + width/2, TOP_IMAGE_COLS);

    // Create a high res fovea
    boost::shared_ptr<FoveaT<hGoals, eGrey> > goalFovea(
    new FoveaT<hGoals, eGrey>(BBox(tl, br), density, 0, true));
    goalFovea->actuate(frame);
    goalFoveas.push_back(goalFovea);

    width  = br.x() - tl.x();


    // Trace the yellow from the centre outwards
    // Note: left and right are in fovea coords
    // int left = -1;
    //int right = -1;
    //int max = 0;
    //int lastYellow = width/2;
    //int maxNotWhite = std::max(4, width/30);
    //find 1/4 from base, use to confirm width

    int centre = goalFovea->bb.width()/2;
    int lastWhiteL = -1;
    int lastWhiteR = -1;
    int maxNotWhite = 3;
    for(int x = centre;(x > 0) && (maxNotWhite > 0);--x){
        Colour c = goalFovea->colour(x,0);
        if(c == cWHITE){
            lastWhiteL = x;
            maxNotWhite = 3;
        } else{
            maxNotWhite--;
            if(maxNotWhite == 0 || c == cFIELD_GREEN){
                break;
            }
        }
    }
    maxNotWhite = 3;
    for(int x = centre;(x < goalFovea->bb.width()) && (maxNotWhite > 0);++x){
        Colour c = goalFovea->colour(x,0);
        if(c == cWHITE){
            lastWhiteR = x;
            maxNotWhite = 3;
        } else{
            maxNotWhite--;
            if(maxNotWhite == 0 || c == cFIELD_GREEN){
                break;
            }
        }
    }


    if(update){
        if(lastWhiteL != -1)goal.a.x() = goalFovea->mapFoveaToImage(Point(lastWhiteL,0)).x();
        if(lastWhiteR != -1)goal.b.x() = goalFovea->mapFoveaToImage(Point(lastWhiteR,0)).x();
        goal.a.y() = goal.b.y()-(goal.b.x()-goal.a.x())*9;
    }

    distance = convRR->pixelSeparationToDistance
    (lastWhiteR-lastWhiteL, GOAL_POST_DIAMETER);
    return distance;
}

PostInfo::Type GoalDetection::determineLeftRightPost(const Fovea& fovea,
const std::vector<BBox>& regions,
const BBox& goal,
PostInfo& p) {
    // Default Case
    PostInfo::Type type = PostInfo::pNone;
    PostInfo::Direction dir = PostInfo::pUnknown;

    // Two Posts Case - just look which is left and which is right
    if (regions.size() == 2) {

        // Make 'it' the other post
        std::vector<BBox>::const_iterator it;
        for (it = regions.begin(); it != regions.end(); ++it) {
            if (goal != *it) break;
        }

        if (goal.a.x() < it->a.x()) type = PostInfo::pLeft;
        else type = PostInfo::pRight;

        // For two posts, also work out if we are on RHS or LHS of posts
        // Do this by checking where the base pixel of each is
        int leftY  = 0;
        int rightY = 0;
        if (type == PostInfo::pLeft) {
            leftY  = goal.b.y();
            rightY = it->b.y();
        } else {
            leftY  = it->b.y();
            rightY = goal.b.y();
        }

        // If the base is higher for the left post, it is further away, so RHS
        // If the base is higher for the right post, it is further away, so LHS
        if (leftY < rightY) {
            dir = PostInfo::pToRightOf;
        } else {
            dir = PostInfo::pToLeftOf;
        }
    }

    // One Post Case - try and use the crossbar to determine left or right

    // This method is deprecated due to the occurence of white
    //scan left and right from midpoint of single post
    //When white is encountered and the x coord is within postlines assume it's a goalpost?

    if (regions.size() == 1) {
        const XHistogram &xhistogram = fovea.xhistogram;

        // Start at post and search left for yellow
        int leftLength = 0;
        for (int i = goal.a.x(); i > 0; --i) {
            int current = xhistogram._counts[i][cWHITE];
            if (current > 0) {
                ++leftLength;
            } else {
                break;
            }
        }

        // Start at post and search right for yellow
        int rightLength = 0;
        for (int i = goal.b.x(); i < xhistogram.size; ++i) {
            int current = xhistogram._counts[i][cWHITE];
            if (current > 0) {
                ++rightLength;
            } else {
                break;
            }
        }

        // Check that the lengths we have are significant
        int postLength = goal.b.x() - goal.a.x();
        postLength *= 2;
        //std::cout << "l = " << leftLength << " r = " << rightLength << std::endl;

        if (leftLength > postLength || rightLength > postLength) {

            // If we can definitely pick a side, do so :)
            if (leftLength > (4*rightLength)) type = PostInfo::pRight;
            else if (rightLength > (4*leftLength)) type = PostInfo::pLeft;
        }

    }

    p.type = type;
    p.dir = dir;
    return type;

}
