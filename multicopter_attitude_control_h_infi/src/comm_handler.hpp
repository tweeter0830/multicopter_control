#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <iostream>

using namespace boost::asio;
using boost::asio::ip::udp;

class datagram_handler
{
public:
	datagram_handler(io_service& ios, boost::asio::ip::address ip, int recv_port, int timeout )
		: _io_service(ios),
		  _timer(ios),
		  _endp(ip,recv_port),
		  _timeout(timeout),
		  _socket(_endp)
		{
			_socket.async_receive_from(
				boost::asio::buffer(_data, max_length), _endp,
				boost::bind(&datagram_handler::handle_receive_from, this,
					    boost::asio::placeholders::error,
					    boost::asio::placeholders::bytes_transferred));

			timer_.expires_from_now(boost::posix_time::seconds(timeout));
			timer_.async_wait(boost::bind(&datagram_handler::close, this));
		}

	void handle_receive_from(const boost::system::error_code& err,
				 std::size_t bytes_transferred )
		{
			if (err)
			{
				std::cout << "Receive error: " << err.message() << "\n";
				this.close();
			}
			else
			{
				timer_.expires_from_now(boost::posix_time::seconds(_timeout));
				std::cout << "Successful receive\n";
				printf( "%i Bytes\n" , bytes_transferred );
			}
		}

	void close()
		{
			_socket.close();
			_is_running = false;
		}

private:
	io_service& _io_service;
	deadline_timer _timer;
	udp::socket _socket;
	udp::endpoint _endp;
	enum { max_length = 40 };
	double _data[max_length];
	size_t _bytes_recv;
	int _timeout;
	bool _is_running;
};

int main()
{
	try
	{
		io_service ios;
		datagram_handler dh(ios);
		ios.run();
	}
	catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
	}

	return 0;
}
