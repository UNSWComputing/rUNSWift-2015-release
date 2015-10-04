/*
Copyright 2010 The University of New South Wales (UNSW).

This file is part of the 2010 team rUNSWift RoboCup entry. You may
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version as
modified below. As the original licensors, we add the following
conditions to that license:

In paragraph 2.b), the phrase "distribute or publish" should be
interpreted to include entry into a competition, and hence the source
of any derived work entered into a competition must be made available
to all parties involved in that competition under the terms of this
license.

In addition, if the authors of a derived work publish any conference
proceedings, journal articles or other academic papers describing that
derived work, then appropriate academic citations to the original work
must be included in that publication.

This rUNSWift source is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with this source code; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#pragma once

#include <iostream>
#include <sstream>
#include <vector>
#include <QObject>

/*
 * YUVHistogram class
 */

class YUVHistogram {
    std::vector<unsigned int> frequency;
    unsigned int from;
    unsigned int to;
    unsigned int numberOfBuckets;
    unsigned int numberOfItems;

    unsigned int minValue;
    unsigned int maxValue;
    unsigned int minCount;
    unsigned int maxCount;
    unsigned int getModeValue(void);
    unsigned int getMedianValue(void);
    double meanValue;
    double stdevValue;
    double entropyValue;
    double maxEntropyValue;

    bool valuesValid;
    bool minCountValid;
    bool meanValid;
    bool stdevValid;
    bool entropyValid;
    bool maxEntropyValid;

public:
    enum ImageType {eBW, eRGB, eYUV, eHSV, eRED, eGREEN, eBLUE};

    YUVHistogram(unsigned int minimum, unsigned int maximum);
    YUVHistogram(unsigned int minimum, unsigned int maximum, unsigned int buckets);

    void addDatapoint(unsigned int item);
    void deleteDatapoint(unsigned int item);

    unsigned int getMinimum(void);
    unsigned int getMaximum(void);
    unsigned int getNumberOfItems(void);
    unsigned int getMinCount(void);
    unsigned int getMaxCount(void);
    unsigned int getMinValue(void);
    unsigned int getMaxValue(void);
    double getMeanValue(void);
    double getStdevValue(void);
    double getEntropyValue(void);
    double getMaxEntropyValue(void);
    void drawHistogram(QPixmap &imagePixmap, unsigned int width, unsigned int height, enum ImageType colour =eBW);
    void printStatistics(std::ostringstream &ost);
};

