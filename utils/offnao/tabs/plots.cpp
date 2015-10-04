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

#include "tabs/plots.hpp"
#include <qwt_painter.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_widget.h>
#include <qwt_legend.h>
#include <qwt_scale_draw.h>
#include "utils/angles.hpp"

#if QWT_VERSION >= 0x060000
#define setRawData setRawSamples
#endif

void DataPlot::alignScales() {
   // The code below shows how to align the scales to
   // the canvas frame, but is also a good example demonstrating
   // why the spreaded API needs polishing.

   canvas()->setFrameStyle(QFrame::Box | QFrame::Plain);
   canvas()->setLineWidth(1);

   for (int i = 0; i < QwtPlot::axisCnt; i++) {
      QwtScaleWidget *scaleWidget = (QwtScaleWidget *)axisWidget(i);
      if (scaleWidget)
         scaleWidget->setMargin(0);

      QwtScaleDraw *scaleDraw = (QwtScaleDraw *)axisScaleDraw(i);
      if (scaleDraw)
         scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
   }
}

void XYZPlot::alignScales() {
   // The code below shows how to align the scales to
   // the canvas frame, but is also a good example demonstrating
   // why the spreaded API needs polishing.

   canvas()->setFrameStyle(QFrame::Box | QFrame::Plain);
   canvas()->setLineWidth(1);

   for (int i = 0; i < QwtPlot::axisCnt; i++) {
      QwtScaleWidget *scaleWidget = (QwtScaleWidget *)axisWidget(i);
      if (scaleWidget)
         scaleWidget->setMargin(0);

      QwtScaleDraw *scaleDraw = (QwtScaleDraw *)axisScaleDraw(i);
      if (scaleDraw)
         scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
   }
}

void DataPlot::push(std::vector<SensorValues> sensors, bool left) {
   if (left)
      for (std::vector<SensorValues>::reverse_iterator it = sensors.rbegin();
           it != sensors.rend(); it++)
         push(*it, left);
   else
      for (std::vector<SensorValues>::iterator it = sensors.begin();
           it != sensors.end(); it++)
         push(*it, left);
}

void XYZPlot::push(std::vector<XYZ_Coord> coms, bool left) {
   if (left)
      for (std::vector<XYZ_Coord>::reverse_iterator it = coms.rbegin();
           it != coms.rend(); it++)
         push(*it, left);
   else
      for (std::vector<XYZ_Coord>::iterator it = coms.begin();
           it != coms.end(); it++)
         push(*it, left);
}

DataPlot::DataPlot(QWidget *parent):
   QwtPlot(parent) {

#if QWT_VERSION >= 0x060000
   // We don't need the cache here
   canvas()->setPaintAttribute(QwtPlotCanvas::BackingStore, false);
#else
   // Disable polygon clipping
   QwtPainter::setDeviceClipping(false);

   // We don't need the cache here
   canvas()->setPaintAttribute(QwtPlotCanvas::PaintCached, false);
   canvas()->setPaintAttribute(QwtPlotCanvas::PaintPacked, false);
#endif

#if QT_VERSION >= 0x040000
#ifdef Q_WS_X11
   /*
      Qt::WA_PaintOnScreen is only supported for X11, but leads
      to substantial bugs with Qt 4.2.x/Windows
   */
   canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif
#endif

   alignScales();

   for (int i = 0; i < PLOT_SIZE; ++i) {
      t[i] = i;
   }
}

XYZPlot::XYZPlot(QWidget *parent):
   QwtPlot(parent) {

#if QWT_VERSION >= 0x060000
   // We don't need the cache here
   canvas()->setPaintAttribute(QwtPlotCanvas::BackingStore, false);
#else
   // Disable polygon clipping
   QwtPainter::setDeviceClipping(false);

   // We don't need the cache here
   canvas()->setPaintAttribute(QwtPlotCanvas::PaintCached, false);
   canvas()->setPaintAttribute(QwtPlotCanvas::PaintPacked, false);
#endif

#if QT_VERSION >= 0x040000
#ifdef Q_WS_X11
   /*
      Qt::WA_PaintOnScreen is only supported for X11, but leads
      to substantial bugs with Qt 4.2.x/Windows
   */
   canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif
#endif

   alignScales();

   for (int i = 0; i < PLOT_SIZE; ++i) {
      t[i] = i;
   }
}

MultiPlot::MultiPlot(QWidget *parent, const std::string &title, u_int size, int min, int max)
   : DataPlot(parent) {
   datum = new double*[size];
   for (u_int i = 0; i < size; ++i) {
      datum[i] = new double[PLOT_SIZE];
      for (u_int j = 0; j < PLOT_SIZE; j++) {
         datum[i][j] = 0.0;
      }
      QwtPlotCurve *compareCurve = new QwtPlotCurve();
      compareCurve->attach(this);
      compareCurve->setRawData(t, datum[i], PLOT_SIZE);
      Qt::GlobalColor colour = static_cast<Qt::GlobalColor>(Qt::red + i);
      compareCurve->setPen(QPen(colour));
   }
   setTitle(QString::fromUtf8(title.c_str()));

   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, min, max);
}

void MultiPlot::push(std::vector<float> values, bool left) {
   if (left)
      for (u_int i = 0; i < values.size(); ++i) {
         for (u_int j = PLOT_SIZE-1; j > 0; --j) {
            datum[i][j] = datum[i][j - 1];
         }
      }
   else
      for (u_int i = 0; i < values.size(); ++i) {
         for (u_int j = 0; j < PLOT_SIZE - 1; ++j) {
            datum[i][j] = datum[i][j + 1];
         }
      }
   for (u_int i = 0; i < values.size(); ++i) {
      datum[i][left ? 0 : PLOT_SIZE - 1] = values[i];
   }
   replot();
}

void MultiPlot::push(SensorValues sensors, bool left) {}

//changing to angle x comparison plot
FsrPlot::FsrPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      data_r2[i] = data_l[i] = data_r[i] = data_t[i] = 0.0;
   setTitle("angleX");

   QwtPlotCurve *curveL = new QwtPlotCurve("filtered angleX");
   curveL->attach(this);
   curveL->setRawData(t, data_l, PLOT_SIZE);
   curveL->setPen(QPen(Qt::red));

   QwtPlotCurve *curveR = new QwtPlotCurve("raw angleX");
   curveR->attach(this);
   curveR->setRawData(t, data_r, PLOT_SIZE);
   curveR->setPen(QPen(Qt::blue));

//   QwtPlotCurve *curveR2 = new QwtPlotCurve("Right2");
//   curveR2->attach(this);
//   curveR2->setRawData(t, data_r2, PLOT_SIZE);
//   curveR2->setPen(QPen(Qt::black));

//   QwtPlotCurve *curveT = new QwtPlotCurve("Total");
//   curveT->attach(this);
//   curveT->setRawData(t, data_t, PLOT_SIZE);

   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -12, 12);
}

void FsrPlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE-1; i > 0; --i) {
         data_l[i] = data_l[i-1];
         data_r[i] = data_r[i-1];
         data_t[i] = data_t[i-1];
         data_r2[i] = data_r2[i-1];
      }
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i) {
         data_l[i] = data_l[i+1];
         data_r[i] = data_r[i+1];
         data_t[i] = data_t[i+1];
         data_r2[i] = data_r2[i+1];
      }
   data_l[left ? 0 : PLOT_SIZE - 1] =
         RAD2DEG(sensors.sensors[Sensors::InertialSensor_AngleX]);
//      sensors.sensors[Sensors::LFoot_FSR_FrontLeft] +
//      sensors.sensors[Sensors::LFoot_FSR_FrontRight] +
//      sensors.sensors[Sensors::LFoot_FSR_RearLeft] +
//      sensors.sensors[Sensors::LFoot_FSR_RearRight];
//   data_r[left ? 0 : PLOT_SIZE - 1] =
//         RAD2DEG(sensors.sensors[Sensors::LFoot_FSR_CenterOfPressure_X]);
//         RAD2DEG(sensors.sensors[Sensors::InertialSensor_GyrY]);
//      sensors.sensors[Sensors::RFoot_FSR_FrontLeft] +
//      sensors.sensors[Sensors::RFoot_FSR_FrontRight] +
//      sensors.sensors[Sensors::RFoot_FSR_RearLeft] +
//      sensors.sensors[Sensors::RFoot_FSR_RearRight];
//   data_t[left ? 0 : PLOT_SIZE - 1] = 0;
//      data_l[left ? 0 : PLOT_SIZE - 1] + data_r[left ? 0 : PLOT_SIZE - 1];
//   data_r2[left ? 0 : PLOT_SIZE - 1] =
//         RAD2DEG(sensors.sensors[Sensors::InertialSensor_AccY]) * 100.0;
         //         RAD2DEG(sensors.sensors[Sensors::LFoot_FSR_CenterOfPressure_Y]);
   replot();
}

SonarPlot::SonarPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      for (int j = 0; j < 10; ++j)
         data[j][i] = 0.0;
   setTitle("Sonar");

   QwtPlotCurve *curve = new QwtPlotCurve("Sonar");
   curve->attach(this);
   curve->setRawData(t, data[0], PLOT_SIZE);
   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, 0, 2.55);
}

void SonarPlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE-1; i > 0; --i)
         for (int j = 0; j < 10; ++j)
            data[j][i] = data[j][i-1];
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i)
         for (int j = 0; j < 10; ++j)
            data[j][i] = data[j][i+1];
   for (int j = 0; j < 10; ++j)
      data[j][left ? 0 : PLOT_SIZE - 1] = sensors.sonar[j];
   replot();
}


OdomPlot::OdomPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i){
      motionData[i] = 0.0; visionData[i] = 0.0; combinedData[i] = 0.0;
   }
   for (int i = 0; i < PLOT_SIZE-1; ++i){
      motionDiff[i] = 0.0; visionDiff[i] = 0.0; combinedDiff[i] = 0.0;
   }
   setTitle("Odometry turn");

   QwtPlotCurve *fcurve = new QwtPlotCurve("Vision");
   fcurve->attach(this);
   fcurve->setRawData(t, visionDiff, PLOT_SIZE-1);
   fcurve->setPen(QPen(Qt::red));

   QwtPlotCurve *rcurve = new QwtPlotCurve("Motion");
   rcurve->attach(this);
   rcurve->setRawData(t, motionDiff, PLOT_SIZE-1);
   rcurve->setPen(QPen(Qt::blue));

   QwtPlotCurve *vcurve = new QwtPlotCurve("Combined");
   vcurve->attach(this);
   vcurve->setRawData(t, combinedDiff, PLOT_SIZE-1);
   vcurve->setPen(QPen(Qt::green));
   
   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 2);
   setAxisScale(QwtPlot::yLeft, -10, 10);
}

void OdomPlot::push(SensorValues sensors, bool left) {
}

void OdomPlot::push(std::vector<Odometry> vision, std::vector<Odometry> motion, std::vector<Odometry> combined, bool left) {
   if (left){
      for (int i=vision.size()-1; i>=0; i--){
         push(vision[i], motion[i], combined[i], left);
      }
   } else {
      for (int i=0; i< (int) vision.size(); i++){
         push(vision[i], motion[i], combined[i], left);
      }
   }
}

void OdomPlot::push(Odometry vision, Odometry motion, Odometry combined, bool left) {
   if (left)
      for (int i = PLOT_SIZE-1; i > 0; --i){
         motionData[i] = motionData[i-1];
         visionData[i] = visionData[i-1];
         combinedData[i] = combinedData[i-1];
      }
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i){
         motionData[i] = motionData[i+1];
         visionData[i] = visionData[i+1];
         combinedData[i] = combinedData[i+1];
      }
   motionData[left ? 0 : PLOT_SIZE - 1] = RAD2DEG(motion.turn);
   visionData[left ? 0 : PLOT_SIZE - 1] = RAD2DEG(vision.turn);
   combinedData[left ? 0 : PLOT_SIZE - 1] = RAD2DEG(combined.turn);

   //std::cout << RAD2DEG(motion.turn) << "\t" << RAD2DEG(vision.turn) << "\t" << RAD2DEG(combined.turn) << "\n";

   for(int i=0; i< (PLOT_SIZE-2); ++i){
      motionDiff[i] = motionData[i+1] - motionData[i];
      visionDiff[i] = visionData[i+1] - visionData[i];
      combinedDiff[i] = combinedData[i+1] - combinedData[i];
   }

   replot();
}



//using it for gyroX
ChargePlot::ChargePlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i) {
      data[i] = 0.0; data2[i] = 0.0;
   }
   setTitle("gyroX");

   QwtPlotCurve *fcurve = new QwtPlotCurve("filtered gyroX");
   fcurve->attach(this);
   fcurve->setRawData(t, data, PLOT_SIZE);
   fcurve->setPen(QPen(Qt::red));

   QwtPlotCurve *rcurve = new QwtPlotCurve("raw gyroX");
   rcurve->attach(this);
   rcurve->setRawData(t, data2, PLOT_SIZE);
   rcurve->setPen(QPen(Qt::blue));

   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -70, 70);
}

void ChargePlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE-1; i > 0; --i){
         data[i] = data[i-1];
         data2[i] = data2[i-1];
      }
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i){
         data[i] = data[i+1];
         data2[i] = data2[i+1];
      }
   data[left ? 0 : PLOT_SIZE - 1] = RAD2DEG(sensors.sensors[Sensors::InertialSensor_GyrX]);
//   data2[left ? 0 : PLOT_SIZE - 1] = RAD2DEG(sensors.sensors[Sensors::Battery_Charge]);
   replot();
}

//using it for gyrY
CurrentPlot::CurrentPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i) {
      data[i] = 0.0; data2[i] = 0.0;
   }
   setTitle("gyroY");

   QwtPlotCurve *fcurve = new QwtPlotCurve("filtered gyroY");
   fcurve->attach(this);
   fcurve->setRawData(t, data, PLOT_SIZE);
   fcurve->setPen(QPen(Qt::red));

   QwtPlotCurve *rcurve = new QwtPlotCurve("raw gyroY");
   rcurve->attach(this);
   rcurve->setRawData(t, data2, PLOT_SIZE);
   rcurve->setPen(QPen(Qt::blue));

   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -70, 70);
}

void CurrentPlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE-1; i > 0; --i){
         data[i] = data[i-1];
         data2[i] = data2[i-1];
      }
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i){
         data[i] = data[i+1];
         data2[i] = data2[i+1];
      }
   data[left ? 0 : PLOT_SIZE - 1] = RAD2DEG(sensors.sensors[Sensors::InertialSensor_GyrY]);
//   data2[left ? 0 : PLOT_SIZE - 1] = RAD2DEG(sensors.sensors[Sensors::Battery_Current]);
   replot();
}

CoronalZMPPlot::CoronalZMPPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i) {
      data_l[i] = 0.0;
      data_r[i] = 0.0;
      data_t[i] = 0.0;
   }
   setTitle("Coronal Plane");

   QwtPlotCurve *curveL = new QwtPlotCurve("Left");
   curveL->attach(this);
   curveL->setRawData(t, data_l, PLOT_SIZE);
   curveL->setPen(QPen(Qt::red));

   QwtPlotCurve *curveR = new QwtPlotCurve("Right");
   curveR->attach(this);
   curveR->setRawData(t, data_r, PLOT_SIZE);
   curveR->setPen(QPen(Qt::blue));

   QwtPlotCurve *curveT = new QwtPlotCurve("Total");
   curveT->attach(this);
   curveT->setRawData(t, data_t, PLOT_SIZE);

   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -100, 100);
}

void CoronalZMPPlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE-1; i > 0; --i) {
         data_l[i] = data_l[i-1];
         data_r[i] = data_r[i-1];
         data_t[i] = data_t[i-1];
      }
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i) {
         data_l[i] = data_l[i+1];
         data_r[i] = data_r[i+1];
         data_t[i] = data_t[i+1];
      }
   float total_weight = (sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
                       + sensors.sensors[Sensors::LFoot_FSR_FrontRight]
                       + sensors.sensors[Sensors::LFoot_FSR_RearLeft]
                       + sensors.sensors[Sensors::LFoot_FSR_RearRight]
                       + sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
                       + sensors.sensors[Sensors::RFoot_FSR_FrontRight]
                       + sensors.sensors[Sensors::RFoot_FSR_RearLeft]
                       + sensors.sensors[Sensors::RFoot_FSR_RearRight]);
   if (total_weight != 0)
      data_t[left ? 0 : PLOT_SIZE - 1] = 
         (80.0f*sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
        + 30.0f*sensors.sensors[Sensors::LFoot_FSR_FrontRight]
        + 80.0f*sensors.sensors[Sensors::LFoot_FSR_RearLeft]
        + 30.0f*sensors.sensors[Sensors::LFoot_FSR_RearRight]
        - 30.0f*sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
        - 80.0f*sensors.sensors[Sensors::RFoot_FSR_FrontRight]
        - 30.0f*sensors.sensors[Sensors::RFoot_FSR_RearLeft]
        - 80.0f*sensors.sensors[Sensors::RFoot_FSR_RearRight]) / total_weight;
   else
      data_t[left ? 0 : PLOT_SIZE - 1] = 0.0f;
   data_l[left ? 0 : PLOT_SIZE - 1] = 1000 *
      sensors.sensors[Sensors::LFoot_FSR_CenterOfPressure_Y];
   data_r[left ? 0 : PLOT_SIZE - 1] = 1000 *
      sensors.sensors[Sensors::RFoot_FSR_CenterOfPressure_Y];
   replot();
}

SagittalZMPPlot::SagittalZMPPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      data[i] = 0.0;
   setTitle("Sagittal Plane");

   QwtPlotCurve *curve = new QwtPlotCurve("SagittalZMP");
   curve->attach(this);
   curve->setRawData(t, data, PLOT_SIZE);
   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -100, 100);
}

void SagittalZMPPlot::push(SensorValues sensors, bool left) {
   if (left)
      for (int i = PLOT_SIZE-1; i > 0; --i)
         data[i] = data[i-1];
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i)
         data[i] = data[i+1];
   float total_weight = (sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
                       + sensors.sensors[Sensors::LFoot_FSR_FrontRight]
                       + sensors.sensors[Sensors::LFoot_FSR_RearLeft]
                       + sensors.sensors[Sensors::LFoot_FSR_RearRight]
                       + sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
                       + sensors.sensors[Sensors::RFoot_FSR_FrontRight]
                       + sensors.sensors[Sensors::RFoot_FSR_RearLeft]
                       + sensors.sensors[Sensors::RFoot_FSR_RearRight]);
   if (total_weight != 0)
      data[left ? 0 : PLOT_SIZE - 1] = 
         (50.0f*sensors.sensors[Sensors::LFoot_FSR_FrontLeft]
        + 50.0f*sensors.sensors[Sensors::LFoot_FSR_FrontRight]
        - 50.0f*sensors.sensors[Sensors::LFoot_FSR_RearLeft]
        - 50.0f*sensors.sensors[Sensors::LFoot_FSR_RearRight]
        + 50.0f*sensors.sensors[Sensors::RFoot_FSR_FrontLeft]
        + 50.0f*sensors.sensors[Sensors::RFoot_FSR_FrontRight]
        - 50.0f*sensors.sensors[Sensors::RFoot_FSR_RearLeft]
        - 50.0f*sensors.sensors[Sensors::RFoot_FSR_RearRight]) / total_weight;
   else
      data[left ? 0 : PLOT_SIZE - 1] = 0.0f;
   replot();
}

COMxPlot::COMxPlot(QWidget *parent)
   : XYZPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      data[i] = 0.0;
   setTitle("COM relative to foot touching ground x");

   QwtPlotCurve *curve = new QwtPlotCurve("COMx");
   curve->attach(this);
   curve->setRawData(t, data, PLOT_SIZE);
   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -100, 100);
}

void COMxPlot::push(XYZ_Coord coord, bool left) {
   if (left)
      for (int i = PLOT_SIZE-1; i > 0; --i)
         data[i] = data[i-1];
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i)
         data[i] = data[i+1];

   data[left ? 0 : PLOT_SIZE - 1] = coord.x;

   replot();
}

COMyPlot::COMyPlot(QWidget *parent)
   : XYZPlot(parent) {
   for (int i = 0; i < PLOT_SIZE; ++i)
      data[i] = 0.0;
   setTitle("COM relative to foot touching ground y");

   QwtPlotCurve *curve = new QwtPlotCurve("COMy");
   curve->attach(this);
   curve->setRawData(t, data, PLOT_SIZE);
   setAxisScale(QwtPlot::xBottom, 0, PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, -100, 100);
}

void COMyPlot::push(XYZ_Coord coord, bool left) {
   if (left)
      for (int i = PLOT_SIZE-1; i > 0; --i)
         data[i] = data[i-1];
   else
      for (int i = 0; i < PLOT_SIZE - 1; ++i)
         data[i] = data[i+1];

   data[left ? 0 : PLOT_SIZE - 1] = coord.y;

   replot();
}

SurfPlot::SurfPlot(QWidget *parent)
   : DataPlot(parent) {
   for (int i = 0; i < SURF_PLOT_SIZE; ++i)
      data[i] = 0.0;
   setTitle("Surf landmark matching score");

   QwtPlotCurve *curve = new QwtPlotCurve("Surf");
   curve->attach(this);
   curve->setRawData(t, data, SURF_PLOT_SIZE);
   setAxisScale(QwtPlot::xBottom, 0, SURF_PLOT_SIZE - 1);
   setAxisScale(QwtPlot::yLeft, 0, 2000);
}

void SurfPlot::push(std::vector<float> scores) {
   data[0] = 0.f;
   for (int i = 1; i < SURF_PLOT_SIZE - 1; ++i){
      if(i < (int)scores.size()+1) data[i] = scores[i-1];
      else data[i] = 0.f;
   }
   replot();
}

void SurfPlot::push(SensorValues sensors, bool left) {
  return;
}
