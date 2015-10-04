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

// from http://old.nabble.com/boost::iostreams-zlib-compression-td15815415.html

#include "compress.hpp"
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <algorithm>

void add_compressor
   (boost::iostreams::filtering_streambuf<boost::iostreams::output>& out,
   CompressionScheme scheme) {
   switch (scheme) {
   case ZLIB_COMPRESSION:
      out.push(boost::iostreams::zlib_compressor());
      break;
   case BZIP2_COMPRESSION:
      out.push(boost::iostreams::bzip2_compressor());
      break;
   default:
      break;
   }
}

//----------------------------------------------------------------------------
void add_decompressor(
      boost::iostreams::filtering_streambuf<boost::iostreams::input>& in,
      CompressionScheme scheme)
{
   switch (scheme) {
   case ZLIB_COMPRESSION:
      in.push(boost::iostreams::zlib_decompressor());
      break;
   case BZIP2_COMPRESSION:
      in.push(boost::iostreams::bzip2_decompressor());
      break;
   default:
      break;
   }
}

void compress(const std::string& data, std::string& buffer,
              CompressionScheme scheme) {
   buffer.clear();
   boost::iostreams::filtering_streambuf<boost::iostreams::output> out;
   add_compressor(out, scheme);
   out.push(boost::iostreams::back_inserter(buffer));
   boost::iostreams::copy(boost::make_iterator_range(data), out);
}


void decompress(const std::string& buffer, std::string& data,
                CompressionScheme scheme) {
   boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
   add_decompressor(in, scheme);
   in.push(boost::make_iterator_range(buffer));
   data.clear();
   boost::iostreams::copy(in, boost::iostreams::back_inserter(data));
}
