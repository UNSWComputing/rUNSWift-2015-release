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

#include <QtCore/qglobal.h>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <QtGui/QApplication>
#include <iostream>
#include "visualiser.hpp"
#include "utils/options.hpp"

namespace po = boost::program_options;
using namespace std;
using namespace boost;

extern string git_version;

po::variables_map config;

int main(int argc, char *argv[]) {
   /** 
    * Read command line options. QT removes its recognized options
    * through the QApplication call. Currently only used for preloading
    * Vision files. If this gets too large, we may want to move this to
    * an Options.cpp helper file
    */
   po::options_description cmdline_options;
   po::options_description generic_options("Generic options");
   generic_options.add_options()
      ("help,h", "produce help message")
      ("version,v", "print version string")
      ("dump,d", po::value<string>(), "open dump.[ofn|yuv]");


   po::options_description vision_options("Vision options");
   vision_options.add_options()
      ("nnmc_bot,c", po::value<string>(), "load nnmc.cal and focus on vision tab")
      ("nnmc_top,C", po::value<string>(), "load separate nnmc.cal for top camera");

   cmdline_options.add(generic_options);
   cmdline_options.add(vision_options);

   po::variables_map vm;
   po::store(po::parse_command_line(argc, argv, cmdline_options), vm);
   po::notify(vm);   

   if (vm.count("help")) {
      std::cout << cmdline_options << endl;
      return 1;
   }

   if (vm.count("version")) {
      std::cout << "OffNao " << git_version << endl;
      return 1;
   }

   /** Load options from config file into global 'config' variable */
   po::options_description config_file_options;
   populate_options(config_file_options);
   ifstream ifs;
   ifs.open("../runswift.cfg");
   store(parse_config_file(ifs, config_file_options), config);
   ifs.close();
   ifs.open("/etc/runswift/runswift.cfg");
   store(parse_config_file(ifs, config_file_options), config);
   ifs.close();
   po::notify(config);

   /** Start the QT application */
   QApplication a(argc, argv);
   Visualiser w;

   if (vm.count("nnmc_bot")) {
      w.visionTab->loadNnmcFile(BOTTOM_CAMERA, vm["nnmc_bot"].as<string>().c_str());
      if (! vm.count("nnmc_top")) {
        w.visionTab->loadNnmcFile(TOP_CAMERA, vm["nnmc_bot"].as<string>().c_str());
      }
   } else {
      w.visionTab->loadNnmcFile(BOTTOM_CAMERA, (std::string(getenv("RUNSWIFT_CHECKOUT_DIR")) + "/image/home/nao/data/bot.nnmc.bz2").c_str());
   }

   if (vm.count("nnmc_top")) {
      w.visionTab->loadNnmcFile(TOP_CAMERA, vm["nnmc_top"].as<string>().c_str());
      if (! vm.count("nnmc_bot")) {
        w.visionTab->loadNnmcFile(BOTTOM_CAMERA, vm["nnmc_top"].as<string>().c_str());
      }
   } else {
      w.visionTab->loadNnmcFile(TOP_CAMERA, (std::string(getenv("RUNSWIFT_CHECKOUT_DIR")) + "/image/home/nao/data/top.nnmc.bz2").c_str());
   }

   if (vm.count("dump")) {
      w.openFile(vm["dump"].as<string>().c_str());
   }

   w.show();

   /* For consistancy it seems as though this must be done after the show */
   if (vm.count("nnmc_top") || vm.count("nnmc_bot")) {
      w.tabs->setCurrentWidget(w.visionTab);
   }


   cerr << "logpath is: " << config["debug.logpath"].as<string>() << endl;

   return a.exec();
}
