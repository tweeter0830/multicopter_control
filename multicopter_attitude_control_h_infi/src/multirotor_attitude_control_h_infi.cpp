#include <stdint.h>
#include <math.h>
#include <float.h>
#include <stdio.h>

#include <vmmlib/matrix.hpp>
#include <vmmlib/vector.hpp>

#include "multirotor_attitude_control_h_infi.hpp"

//DEBUG
#ifdef DEBUG
#include <iostream>
#endif

typedef vmml::matrix< 3, 3, float> Matrix;
typedef vmml::vector<3,float> Vector;

Multirotor_Attitude_Control_H_Infi::Multirotor_Attitude_Control_H_Infi() {
	_last_run = 0;
	_tc = 0.01f;
	_weight_error_deriv = 1;
	_weight_error_state = 1;
	_weight_error_integral = 1;
	_weight_torque = 1;
	_Ixx =1;
	_Iyy =1;
	_Izz =1;
	_old_time = 0;
	_setpoint_state = State();
	_setpoint_rate = State();
	_setpoint_accel = State();
	_integral(0)=0.0f;
	_integral(1)=0.0f;
	_integral(2)=0.0f;
	_modes_set = false;
	_state_track = false;
	_rate_track = false;
	_accel_track= false;
}

void Multirotor_Attitude_Control_H_Infi::set_mode(bool state_track, bool rate_track, bool accel_track){
	_state_track = state_track;
	_rate_track = rate_track;
	_accel_track= accel_track;
	_modes_set = true;
}
void Multirotor_Attitude_Control_H_Infi::set_setpoints(const State& state,const State& rate,const State& accel) {
	_setpoint_state = state;
	_setpoint_rate = rate;
	_setpoint_accel = accel;
}
bool Multirotor_Attitude_Control_H_Infi::control(const State& meas_state, const State& meas_rate, State& torque_out, double time) {
	if(!_modes_set) {
		return false;
	}
	// TODO: check inputs here
	float dt = time-_old_time;
	Vector k_p;
	Vector k_i;
	Vector k_d;
	make_M(meas_state,_M);
	make_C(meas_state, meas_rate, _Cor);
	calc_gains(_M,_Cor, k_p, k_i, k_d);
	Vector error_state;
	if( _state_track ){
		error_state(0) = meas_state.r-_setpoint_state.r;
		error_state(1) = meas_state.p-_setpoint_state.p;
		error_state(2) = meas_state.y-_setpoint_state.y;
	}else{
		error_state = Vector::ZERO;
	}
	Vector error_rate;
	if( _rate_track ) {
		error_rate(0) = meas_rate.r-_setpoint_rate.r;
		error_rate(1) = meas_rate.p-_setpoint_rate.p;
		error_rate(2) = meas_rate.y-_setpoint_rate.y;
	} else {
		error_rate = Vector::ZERO;
	}
	Vector setpoint_accel;
	if( _accel_track ) {
		setpoint_accel(0) = _setpoint_accel.r;
		setpoint_accel(1) = _setpoint_accel.p;
		setpoint_accel(2) = _setpoint_accel.y;
	} else {
		setpoint_accel = Vector::ZERO;
	}
	Vector meas_rate_vect;
	meas_rate_vect(0)=meas_state.r;
	meas_rate_vect(1)=meas_state.p;
	meas_rate_vect(2)=meas_state.y;
	#ifdef DEBUG
	std::cout << "error_state " << error_state << std::endl;
	std::cout << "error_rate " << error_rate << std::endl;
	std::cout << "setpoint_accel " << setpoint_accel << std::endl;
	#endif
	_integral = _integral + error_state*dt;
	// TODO: check integral limits and saturation
	Vector control_accel = k_d*error_rate + k_p*error_state - k_i*_integral;
	Vector control_torque = _M*setpoint_accel + _Cor*meas_rate_vect - _M*control_accel;
	#ifdef DEBUG
	std::cout << "Time Diff: " << dt << std::endl;
	std::cout << "_integral " << _integral << std::endl;
	std::cout << "control_accel " << control_accel << std::endl;
	std::cout << "control_torque "<< control_torque<< std::endl;
	#endif
	torque_out.r = control_torque(0);
	torque_out.p = control_torque(1);
	torque_out.y = control_torque(2);
	_old_time = time;
	return true;
}

void Multirotor_Attitude_Control_H_Infi::reset_integrator()
{
	_integral(0)=0.0f;
	_integral(1)=0.0f;
	_integral(2)=0.0f;
}

void Multirotor_Attitude_Control_H_Infi::calc_gains(const Matrix& M,const Matrix& C, Vector& k_p, Vector& k_i, Vector& k_d) {
	const float w_1 = _weight_error_deriv;
	const float w_2 = _weight_error_state;
	const float w_3 = _weight_error_integral;
	const float w_u = _weight_torque;
	
	float I_data[] = { 1,0,0, 0,1,0, 0,0,1 };
	Matrix I;
	I.set(I_data,I_data+9);
	Matrix M_inv;
	M.inverse(M_inv);
	Matrix Dynamics_weights = M_inv*( C+I*( 1.0f/(w_u*w_u) ) );
	float long_expr = std::sqrt(w_2*w_2 + 2.0f*w_1*w_3)/w_1;
	
	k_d=(I*long_expr)+Dynamics_weights;
	k_p=I*(w_3/w_1)+Dynamics_weights*long_expr;
	k_i=Dynamics_weights*(w_3/w_1);
	#ifdef DEBUG
	std::cout<< "Calculated Gains:---------"<< std::endl;
	std::cout<< "I\n" << I << std::endl;
	std::cout<< "M_Inverse\n" << M_inv << std::endl;
	std::cout<< "long_expr\n " << long_expr << std::endl;
	std::cout<< "Dynamics_weights\n" <<Dynamics_weights << std::endl;
	std::cout<< "k_d\n"<< k_d << std::endl;
	std::cout<< "k_p\n"<< k_p << std::endl;
	std::cout<< "k_i\n"<< k_i << std::endl;
	#endif
}

void Multirotor_Attitude_Control_H_Infi::make_M(const State& St, Matrix& M) {
	float M_vals [9] = {0};
	float sin_R=std::sin(St.r);
	float cos_R=std::cos(St.r);
	float sin_P=std::sin(St.p);
	float cos_P=std::cos(St.p);
	//First Row
	M_vals[0]=_Ixx;
	M_vals[1]=0;
	M_vals[2]=-_Ixx*sin_P;
	// Second Row
	M_vals[3]=0;
	M_vals[4]=_Iyy*cos_R*cos_R + _Izz*sin_R*sin_R;
	M_vals[5]=(_Iyy-_Izz)*cos_R*sin_R*cos_P;
	// Third row
	M_vals[6]=-_Ixx*sin_P;
	M_vals[7]=(_Iyy-_Izz)*cos_R*sin_R*cos_P;
	M_vals[8]=_Ixx*sin_P*sin_P + _Iyy*sin_R*sin_R*cos_P*cos_P +
		     _Izz*cos_R*cos_R*cos_P*cos_P;
	
	M.set(M_vals,M_vals+9);
	#ifdef DEBUG
	std::cout<< "M " << M << std::endl;
	#endif
}

void Multirotor_Attitude_Control_H_Infi::make_C(const State& St, const State& Rate, Matrix& C) {
	float C_vals [9] = {0};
	float s_ph=std::sin(St.r);
	float c_ph=std::cos(St.r);
	float s_th=std::sin(St.p);
	float c_th=std::cos(St.p);
	float long_factor = Rate.p*c_ph*s_ph + Rate.y*s_ph*s_ph*c_th;
	//First Row
	C_vals[0]=0;
	C_vals[1]=(_Iyy-_Izz)*(long_factor) + 
		     (_Izz-_Iyy)*Rate.y*c_ph*c_ph*c_th -
		     _Ixx*Rate.y*c_th;
	C_vals[2]=(_Izz-_Iyy)*Rate.y*c_ph*s_ph*c_th*c_th;
	//Second Row
	C_vals[3]=(_Izz-_Iyy)*(long_factor) + 
	             (_Iyy-_Izz)*Rate.y*c_ph*c_ph*c_th +
		     _Ixx*Rate.y*c_th;
	C_vals[4]=(_Izz-_Iyy)*Rate.r*c_ph*c_ph;
	C_vals[5]=-_Ixx*Rate.y*s_th*c_th +
		      _Iyy*Rate.y*s_ph*s_ph*c_th*s_th +
		      _Izz*Rate.y*c_ph*c_ph*s_th*c_th;
	//Third Row
	C_vals[6]=(_Iyy-_Izz)*Rate.y*c_th*c_th*s_ph*c_ph - 
		     _Ixx*Rate.p*c_th;
	C_vals[7]=(_Izz-_Iyy)*(Rate.p*c_ph*s_ph*s_th+Rate.r*s_ph*s_ph*c_th) +
		     (_Iyy-_Izz)*Rate.r*c_ph*c_ph*c_th + 
		     _Ixx*Rate.y*s_th*c_th - 
		     _Iyy*Rate.y*s_ph*s_ph*s_th*c_th - 
		     _Izz*Rate.y*c_ph*c_ph*s_th*c_th;
	C_vals[8]=(_Iyy-_Izz)*Rate.r*c_ph*s_ph*c_th*c_th - 
		     _Iyy*Rate.p*s_ph*s_ph*c_th*s_th - 
		     _Izz*Rate.p*c_ph*c_ph*c_th*s_th + 
		     _Ixx*Rate.p*c_th*s_th;
	C.set( C_vals, C_vals + 9 );
	#ifdef DEBUG
	std::cout<< "C " << C << std::endl;
	#endif
}
