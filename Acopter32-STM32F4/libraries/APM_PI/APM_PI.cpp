// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	ACM_PI.cpp
/// @brief	Generic PI algorithm

#include <math.h>

#include "APM_PI.h"

const AP_Param::GroupInfo APM_PI::var_info[] = {
    AP_GROUPINFO("P",    0, APM_PI, _kp, 0),
    AP_GROUPINFO("I",    1, APM_PI, _ki, 0),
    AP_GROUPINFO("IMAX", 2, APM_PI, _imax, 0),
    AP_GROUPEND
};

int32_t APM_PI::get_p(int32_t error)
{
	return (int32_t)((float)error * _kp);
}

int32_t APM_PI::get_i(int32_t error, float dt)
{
	if((dt > 0.0) || (dt < 0.0)) {
        _integrator += ((float)error * _ki) * dt;

        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
    }
	return (int32_t)_integrator;
}

int32_t APM_PI::get_pi(int32_t error, float dt)
{
    return get_p(error) + get_i(error, dt);
}


float APM_PI::get_p_f(float error)
{
	return (error * _kp);
}

float APM_PI::get_i_f(float error, float dt)
{
	if((dt > 0.0) || (dt < 0.0)) {
        _integrator += (error * _ki) * dt;

        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
    }
	return _integrator;
}

float APM_PI::get_pi_f(float error, float dt)
{
    return get_p(error) + get_i(error, dt);
}

void
APM_PI::reset_I()
{
	_integrator = 0.0;
}

void
APM_PI::load_gains()
{
    _kp.load();
    _ki.load();
    _imax.load();
}

void
APM_PI::save_gains()
{
    _kp.save();
    _ki.save();
    _imax.save();
}