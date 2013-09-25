#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <chrono> 
#include <thread>
#include <iostream>
#include <string>
#include <cstring>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include "multirotor_attitude_control_h_infi.hpp"

using boost::asio::ip::udp;
using namespace boost::asio;

class Datagram_Handler
{
public:
	Datagram_Handler(boost::asio::io_service& ios, udp::endpoint endp, int timeout ) 
		: _io_service(ios), _socket(ios), _timer(ios){
		//_timer = boost::asio::deadline_timer(ios);
		_endp = endp;
		_timeout = timeout;
		_socket.open(udp::v4());
		_socket.bind(_endp);
		_is_running = false;
	}
	void start_listening(){
		//_socket.open(udp::v4());
		//_socket.bind(_endp);
		_socket.async_receive_from(
			boost::asio::buffer(_data, max_length), _endp,
			boost::bind(&Datagram_Handler::handle_receive_from, this,
				    boost::asio::placeholders::error,
				    boost::asio::placeholders::bytes_transferred));

		_timer.expires_from_now(boost::posix_time::seconds(_timeout));
		_timer.async_wait(boost::bind(&Datagram_Handler::close, this));
		_is_running = true;

	}
	void handle_receive_from(const boost::system::error_code& err,
				 std::size_t bytes_transferred ){
		if (err)
		{
			std::cout << "Receive error: " << err.message() << "\n";
			this->close();
		}
		else
		{
			this->start_listening();
			std::cout << "Successful receive\n";
			printf( "%i Bytes\n" , bytes_transferred );
		}
	}
	void close(){
		_is_running = false;
		_socket.cancel();
       	}
	bool is_running() {
		return _is_running;
	}
	void copy_data(double in_data[], int len){
	      memcpy(_data, in_data, len);
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

int main(int argc, char* argv[])
{
	// TODO: Parse inputs
	// Physical Params:
	float Ixx = 6.23e-3f;
	float Iyy = 6.23e-3f;
	float Izz = 1.121e-2f;
	float moment_arm = 0.232f;
	// Control Params:
	float weight_deriv = 0.1f;
	float weight_state = 3.0f;
	float weight_int =   9.0f;
	float weight_torque = 1.5f;
	//Simulation Params
	std::string str_internal_ip = "127.0.0.1";
	double time = 0;
	try
	{
		// if (argc != 2)
		// {
		// 	std::cerr << "Usage: client <host>" << std::endl;
		// 	return 1;
		// }
		std::cout<< "Started" << std::endl;
		// Set up UDP communication
		boost::asio::ip::address internal_ip = boost::asio::ip::address::from_string(str_internal_ip);
		boost::asio::io_service ios;
		
		udp::endpoint endpoint_recv_from(internal_ip,2040);
		udp::endpoint endpoint_send_to(internal_ip,2500);
	       
		udp::socket socket_send(ios);
		socket_send.open(udp::v4());
		
		// Initialize our controller
		Multirotor_Attitude_Control_H_Infi quad_control;
		quad_control.set_phys_params(Ixx, Iyy, Izz);
		quad_control.set_weights(weight_state,weight_int,weight_deriv,weight_torque);
		quad_control.set_mode(true, true, true);
		// Set up a bunch of parameters for measurement reading and control sending
		Multirotor_Attitude_Control_H_Infi::State meas_state, meas_rate, torque_out;

		double recv_buf[40] = {0};
		boost::array<double, 3> send_buf;

		bool sim_running = true;
		Datagram_Handler recv_handler(ios,endpoint_recv_from,5);
		recv_handler.start_listening();
		std::cout<<"Control ready for input"<<std::endl;
		while ( sim_running ) {
			// Get state information for roll, pitch and yaw and their derivs
			ios.run();
			if( recv_handler.is_running() ){
				recv_handler.copy_data(recv_buf,7);
				printf( "RECV: '%lu' %e %e %e %e %e %e\n", 
					3, 
					recv_buf[0],
					recv_buf[1],
					recv_buf[2],
					recv_buf[3],
					recv_buf[4],
					recv_buf[5] );
				meas_state.r = recv_buf[0];
				meas_state.p = -recv_buf[1];
				meas_state.y = recv_buf[2];
				meas_rate.r = recv_buf[3];
				meas_rate.p = -recv_buf[4];
				meas_rate.y = recv_buf[5];
				time = recv_buf[6];
				// Update the control command
				quad_control.control(meas_state, meas_rate, torque_out, time);
				// Send control to the simulation
				send_buf[0]=torque_out.r/moment_arm;
				send_buf[1]=torque_out.p/moment_arm;
				send_buf[2]=-torque_out.y;
				printf( "SENT: %e %e %e\n", send_buf[0], send_buf[1], send_buf[2] );
				socket_send.send_to(boost::asio::buffer(send_buf), endpoint_send_to);
			} else {
				std::cout << "Timedout, Resetting Integral." << std::endl;
				quad_control.reset_integrator();
				//recv_handler.start_listening();
			}
			//ios.run();
			//recv_handler.start_listening();
			//ios.run();
		}
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return 0;
}
