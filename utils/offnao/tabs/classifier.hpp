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

#include <stdio.h>
#include <stdint.h>
#include <QRgb>
#include <fstream>
#include <deque>
#include <iostream>
#include <vector>
#include <string>
#include "perception/vision/VisionDefs.hpp"

static const int YMAX = 128;
static const int UMAX = 128;
static const int VMAX = 128;
static const int CMAX = cNUM_COLOURS;

static const int Y_KERNEL_RADIUS = 10;
static const int Y_KERNEL_RADIUS_SQUARED = Y_KERNEL_RADIUS * Y_KERNEL_RADIUS;
static const int Y_KERNEL_RADIUS_POW_4 = Y_KERNEL_RADIUS_SQUARED*
                                         Y_KERNEL_RADIUS_SQUARED;
static const double INV_Y_KERNEL_RADIUS_SQUARED = 1.0/
                              (Y_KERNEL_RADIUS_SQUARED);
static const double INV_Y_KERNEL_RADIUS_POW_4 = 1.0 / (Y_KERNEL_RADIUS_POW_4);


static const int U_KERNEL_RADIUS = 10;
static const int U_KERNEL_RADIUS_SQUARED = U_KERNEL_RADIUS * U_KERNEL_RADIUS;
static const double INV_U_KERNEL_RADIUS_SQUARED = 1.0 /
                              (U_KERNEL_RADIUS_SQUARED);

static const int V_KERNEL_RADIUS = 10;
static const int V_KERNEL_RADIUS_SQUARED = V_KERNEL_RADIUS * V_KERNEL_RADIUS;
static const double INV_V_KERNEL_RADIUS_SQUARED = 1.0/
                              (V_KERNEL_RADIUS_SQUARED);

static const float INITIAL_NONE_VALUE = 0.001f;

struct YuvTriple {
   unsigned char y, u, v;
};

struct Weights {
   float w[CMAX];
};

struct YuvGaussian {
   YuvGaussian() {
      yuv.y = 255;
      yuv.u = 255;
      yuv.v = 255;
      weight = 0;
      hacks = false;
      classified = cBACKGROUND;
   }

   YuvGaussian(YuvTriple yuv, double weight, Colour classified, int yRadius,
               int uRadius, int vRadius, bool hacks) {
      this->yuv = yuv;

      this->weight = weight;
      this->yRadius = yRadius;
      this->uRadius = uRadius;
      this->vRadius = vRadius;
      this->classified = classified;
      this->hacks = hacks;
   }

   YuvTriple yuv;
   double weight;
   int yRadius, uRadius, vRadius;
   bool hacks;
   Colour classified;
};

// An action is a list of gaussians
struct Action {
   std::vector<YuvGaussian> gaussians;
   void clear(void) {
      gaussians.clear();
   }
};

// This class lets you make a classification file
class Classifier {
   public:

      /* Converts a YUV pixel spec to RGB */
      static QRgb yuv2rgb(int y, int u, int v);

      Classifier(void);
      ~Classifier(void);

      void newClassificationFile(void);
      void saveClassificationFile(std::string filename);
      bool loadClassificationFile(std::string filename);

      void beginAction(void);  // Wrap your Classify calls inside these
      void endAction(void);    // so undo will work.

      bool isMostlyClassified(int y, int u, int v, Colour c);
      void classify(int y, int u, int v, double weight, Colour c, int yRadius,
                    int uRadius, int vRadius, bool hacks);

      bool canUndo(void);
      void getUndoStatus(int& levels, Colour& lastColour);
      void undo(void);

      void saveNnmc(std::string filename);

      bool classificationFileOpened(void);

      Colour getClassifiedColour(int y, int u, int v);
      float getColourMargin(int y, int u, int v);
      void colourInfo(int y, int u, int v, float weights[CMAX]);
      float rawKernelVal(int dY, int dU, int dV, int yRadius, int uRadius,
            int vRadius, bool hacks);

      // The next few guys are for doing hacks with visualisation and
      // stuff like that
      unsigned char* getNnmcPointer(void);
      void setNnmcPointer(unsigned char* nnmc);
      void setUpdateLiveNnmc(bool);

   private:
      void applyAction(Action &a, bool undo);
      void applyGaussian(YuvGaussian&, bool undo);
      void addWeight(YuvTriple yuv, float amount, Colour c, bool undo);

      bool fileOpened;

      void clearFile(void);
      Colour maxWeightIndex(Weights&);
      void makeNnmc(void);
      void getWeights(YuvTriple, Weights&);
      void setWeights(YuvTriple, Weights&);
      int64_t yuvToFilePos(YuvTriple);
      std::deque<Action> undoBuffer;
      Action currentAction;

      unsigned char nnmc[VMAX][UMAX][YMAX];
      unsigned char* liveNnmc;
      bool useLiveNnmc;
      Weights weight[YMAX][UMAX][VMAX];
};

