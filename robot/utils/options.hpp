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
#include <boost/program_options.hpp>
#include <string>
#include <vector>

void populate_options(
   boost::program_options::options_description &config_file_options);

/**
 * populates the variable map with the options from argv and the config files.
 *
 * first invocation should be from main, and subsequent invocations will recall
 * argv used in first invocation and overwrite with any new argv.
 *
 * unfortunately, once a variable is set in a variable map, there is no way to
 * change it, so we have to create a whole new map and reparse everything.
 *
 * @param argv    a vector of command-line options
 * @param vm      the variable map to fill
 * @param generic any additional options (like help/version) to be understood
 * @returns a complete options description of all supported options
 */
boost::program_options::options_description
store_and_notify(std::vector<std::string> argv,
                 boost::program_options::variables_map &vm,
                 boost::program_options::options_description* generic =
                    NULL);

/**
 * populates the variable map with the options from argv and the config files.
 *
 * first invocation should be from main, and subsequent invocations will recall
 * argv used in first invocation and overwrite with any new argv.
 *
 * unfortunately, once a variable is set in a variable map, there is no way to
 * change it, so we have to create a whole new map and reparse everything.
 *
 * @param argc    the length of argv
 * @param argv    an array of command-line options
 * @param vm      the variable map to fill
 * @param generic any additional options (like help/version) to be understood
 * @returns a complete options description of all supported options
 */
boost::program_options::options_description
store_and_notify(int argc, char **argv,
                 boost::program_options::variables_map &vm,
                 boost::program_options::options_description* generic =
                    NULL);

void options_print(boost::program_options::variables_map &vm);
