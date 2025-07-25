#pragma once
/**
 * project : cvpass
 * file    : PID.h
 * Copytight <c> handle 2025 all rights reserved.
**/

class PIDController{
public:
	PIDController(double kp,double ki,double kd,double min,double max) {
		Kp = kp;
		Ki = ki;
		Kd = kd;

		this->min = min;
		this->max = max;

		integral = 0;
		last_val = 0;
	}

	double compute(double in,double dt) {
		//¼ÆËãP
		double P = Kp * in;
		//¼ÆËãI
		double error = in - last_val;
		
		integral += error * dt;

		double I = Ki * integral;
		//¼ÆËãD

		double D = Kd * error / dt;

		double output = P + I + D;

		last_val = in;

		if (output > max)output = max;
		if (output < min)output = min;

		return output;
	}
private:
	double Kp, Ki, Kd;
	double integral;
	double last_val;
	double min, max;

};