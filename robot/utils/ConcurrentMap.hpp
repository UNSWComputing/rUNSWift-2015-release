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

#include <boost/thread/mutex.hpp>
#include <map>
#include <vector>

/* A thread-safe wrapper around the stl map class
 * Uses a boost::mutex to ensure atomicity */
template <class Key, class Data>
class ConcurrentMap {
   public:
      /* Used to index into the map
       * @param key the key to look up in the map
       * @return reference to the value corresonding to the key */
      Data & operator[](const Key& key);
      /* Find how many values correspond to a key
       * @param key the key to look up in the map
       * @return the number of corresponding values (0 or 1) since it's not a multimap */
      unsigned int count(const Key& key) const;
      /* @return a vectory with a copy of all the keys
       * Useful for thread-safe iteration of keys */
      std::vector<Key> keys() const;
   private:
      /* Private instance of the stl map, being wrapped */
      std::map<Key, Data> theMap;
      /* Mutex to ensure atomicity of operations on the ConcurrentMap */
      mutable boost::mutex theLock;
};

#include "utils/ConcurrentMap.tcc"
