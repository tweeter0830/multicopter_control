//
// client.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2013 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

int main(int argc, char* argv[])
{
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
		
		udp::endpoint endpoint_recv_from(internal_ip,1230);
		udp::endpoint endpoint_send_to(internal_ip,1230);

		udp::socket socket(io_service);
		socket.open(udp::v4());
		socket.bind(endpoint_recv_from);

		boost::array<char, 4> send_buf  = {{ 'l','o','n','g' }};
		socket.send_to(boost::asio::buffer(send_buf), endpoint_send_to);
		std::cout<< "Data '" << send_buf.data() << "' sent" << std::endl;
		boost::array<char, 4> recv_buf;
		size_t len = socket.receive_from(
			boost::asio::buffer(recv_buf), endpoint_send_to);

		std::cout.write(recv_buf.data(), len);
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return 0;
}
