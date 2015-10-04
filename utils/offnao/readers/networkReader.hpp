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

#include <boost/thread.hpp>
#include <deque>
#include "readers/reader.hpp"
#include "utils/Connection.hpp"
#include "transmitter/TransmitterDefs.hpp"

typedef int64_t msg_t;
typedef std::deque<msg_t> chat_message_queue;

/* A reader that connects with the nao and collects data which is
 * then stored in a naoData object.
 *
 */
class NetworkReader : public Reader {
   public:
      explicit NetworkReader(const QString &robotName, OffNaoMask_t mask);
      explicit NetworkReader(std::pair<const QString &,
                                       OffNaoMask_t> robotNameMask);
      explicit NetworkReader(std::pair<const QString &,
                                       OffNaoMask_t> robotNameMask,
                             const NaoData &naoData);
      ~NetworkReader();

      // main loop that runs when the thread starts
      virtual void run();

      /* Writes a message to the nao */
      void write(const msg_t &msg);
   private:
      OffNaoMask_t mask;
      boost::asio::io_service *ioservice;
      // WirelessClient *wirelessClient;

      void handle_connect(const boost::system::error_code& e,
            boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
      void handle_read(const boost::system::error_code& e);
      Connection *connection_;

      boost::thread *cthread;
      Frame received;

      boost::asio::ip::tcp::resolver *resolver;
      boost::asio::ip::tcp::resolver::query *query;

      bool isRecording;
      QString robotName;

      bool disconnect();
      bool connect();


      void do_write(msg_t msg);

      void handle_write(const boost::system::error_code &error);

      void do_close();

      chat_message_queue write_msgs_;
      public slots:
         virtual void stopMediaTrigger();
      virtual void recordMediaTrigger();
      /**
       * sends a command line string to the Nao
       */
      virtual void sendCommandLineString(QString item);
};
