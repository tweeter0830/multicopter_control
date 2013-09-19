//
// client.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <chrono>
#include <thread>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include "multirotor_attitude_control_h_infi.hpp"

using boost::asio::ip::udp;

int main(int argc, char* argv[])
{
	//TODO: Parse 
	std::string str_internal_ip = "127.0.0.1";
	try
	{
		// if (argc != 2)
		// {
		// 	std::cerr << "Usage: client <host>" << std::endl;
		// 	return 1;
		// }
		std::cout<< "Started" << std::endl;
		boost::asio::ip::address internal_ip = boost::asio::ip::address::from_string(str_internal_ip);
		boost::asio::io_service io_service;
		
		udp::endpoint endpoint_recv_from(internal_ip,2040);
		udp::endpoint endpoint_send_to(internal_ip,2500);

		udp::socket socket_recv(io_service);
		udp::socket socket_send(io_service);
		socket_recv.open(udp::v4());
		socket_send.open(udp::v4());
		socket_recv.bind(endpoint_recv_from);
		socket_send.bind(endpoint_send_to);
		boost::array<int, 1> send_buf  = {{ 10 }};
		while( true ) {
			socket_send.send_to(boost::asio::buffer(send_buf), endpoint_send_to);
			std::cout<< "Data '" << send_buf.data() << "' sent" << std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(1500));
		}
		Multirotor_Attitude_Control_H_Infi quad_control();

	}
		
	// while( true ) {
	// 	boost::array<double, 1> recv_buf;
	// 	size_t len =socket_recv.receive_from(
	// 		boost::asio::buffer(recv_buf), endpoint_recv_from);
	// 	printf( "Data '%2.3f' Received\n", recv_buf[0] );// recv_buf.data());
	// }		
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return 0;
}
