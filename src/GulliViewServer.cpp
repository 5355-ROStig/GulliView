//
// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2008 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//
#include "OpenCVHelper.h"
#include "TagDetector.h"

#include <sys/time.h>

#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "CameraUtil.h"


using namespace std;
using helper::ImageSource;
using boost::asio::ip::udp;
using boost::posix_time::ptime;
using boost::posix_time::time_duration;
int main()
{
  // Create logfile to be used
  std::ofstream fout("GulliViewLog.txt");
  try
  {
    boost::asio::io_service io_service;

    udp::socket socket(io_service, udp::endpoint(udp::v4(), 13));
    
    for (;;)
    {
      boost::array<char, 128> recv_buf;
      udp::endpoint remote_endpoint;
      //boost::system::error_code error;
      socket.receive_from(
        boost::asio::buffer(recv_buf), remote_endpoint);
      ptime recvTime;
      recvTime = boost::posix_time::microsec_clock::local_time();
      //std::cout<< recvTime << "\n";
      std::string data(recv_buf.begin(), recv_buf.end());
      //std::cout << "Data: " << data << "\n";
      unsigned firstDel = data.find('[');
      unsigned lastDel = data.find(']');
      string strNew = data.substr (firstDel+1,(lastDel-firstDel)-1);
      //std::cout << strNew << "\n";
      ptime startProcTime;
      startProcTime = boost::lexical_cast<ptime>(strNew);
      //std::cout << startProcTime << "\n";
      time_duration fullProcessTime = recvTime - startProcTime;
      //std::cout << "Full Proc Time: " <<fullProcessTime << "\n";
      // Time Stamp --- Received
      //std::cout << recv_buf.data() << "\n";
      // Write to logfile and save
      fout << recv_buf.data() << "" << std::endl;
      
      
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }
  // Close log file
  fout << "Program closed: " << std::endl;
  fout.close();
  return 0;
}
