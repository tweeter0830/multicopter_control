/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name APL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file multirotor_attitude_control_h_infi.h
 * Definition of a simple orthogonal roll PID controller.
 *
 */

#ifndef MULTIROTOR_ATTITUDE_CONTROL_H_INFI_H
#define MULTIROTOR_ATTITUDE_CONTROL_H_INFI_H

#include <stdbool.h>
#include <stdint.h>

#include <vmmlib/matrix.hpp>
#include <vmmlib/vector.hpp>

typedef vmml::matrix< 3, 3, float> Matrix;
typedef vmml::vector<3,float> Vector;

// #if !defined(CONFIG_ARCH_CORTEXM4) && !defined(CONFIG_ARCH_FPU)
// typedef long long uint64_t;
// #endif

using namespace vmml;

// TODO: scale the integral to timestep
class Multirotor_Attitude_Control_H_Infi{ //__EXPORT
public:
	Multirotor_Attitude_Control_H_Infi();
        
	typedef struct State
	{
		//Body angles in radians
		float r;
		float p;
		float y;

		State():r(0.0f),p(0.0f),y(0.0f){};
       	};
	void set_phys_params(float Ixx, float Iyy, float Izz){
		_Ixx = Ixx;
		_Iyy = Iyy;
		_Izz = Izz;
	};
	bool control(const State& meas_state, const State& meas_rate, State& torque_out, double time);

	void reset_integrator();

	void set_mode(bool state_track, bool rate_track, bool accel_track);

	void set_setpoints(const State& state,const State& rate,const State& accel);

	void set_time_constant(float time_constant) {
		if (time_constant > 0.1f && time_constant < 3.0f) {
			_tc = time_constant;
		}
	}
	void set_weights(float w_error, float w_int, float w_deriv, float w_torque){
		_weight_error_state = w_error;
		_weight_error_integral = w_int;
		_weight_error_deriv = w_deriv;
		_weight_torque = w_torque;
	}
	void set_weight_error_deriv(float weight_in) {
		_weight_error_deriv = weight_in;
	}
	void set_weight_error_state(float weight_in) {
		_weight_error_state = weight_in;
	}
	void set_weight_integral(float weight_in) {
		_weight_error_integral = weight_in;
	}
	void set_integrator_max(float max) {
		//_integrator_max = max;
	}
	void set_max_rate(float max_rate) {
		//_max_rate = max_rate;
	}

	float get_rate_error() {
		//return _rate_error;
	}

	float get_desired_rate() {
		//return _rate_setpoint;
	}

private:
	int _last_run;
	float _tc;
	float _weight_error_deriv;
	float _weight_error_state;
	float _weight_error_integral;
	float _weight_torque;
	float _Ixx;
	float _Iyy;
	float _Izz;
	State _setpoint_state;
	State _setpoint_rate;
	State _setpoint_accel;
	float _command_torque [3];
	bool _modes_set;
	bool _state_track;
	bool _rate_track;
	bool _accel_track;

	Vector _integral;
	Matrix _M;
	Matrix _M_inv;
	Matrix _Cor;
	double _old_time;

	void calc_gains(const Matrix& M,const Matrix& C, Vector& k_p, Vector& k_i, Vector& k_d);
	void make_M(const State& St, Matrix& M);
	void make_C(const State& St, const State& Rate, Matrix& C);
};

#endif // MULTIROTOR_ATTITUDE_CONTROL_H_INFI_H
