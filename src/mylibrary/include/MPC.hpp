// MPC.hpp

#ifndef MPC_H
#define MPC_H

#include <casadi/casadi.hpp>
#include "Hovercraft_model.hpp"
#include "Cost.hpp"

class MPC
{
	public:
		// constructor
		MPC(Hovercraft_model dynamics, Cost tracking_cost, int N, double dt);

		// compute the control plan
		DM planning(MX x, double t);	
	private:
		int N;
		Function qd;
		Cost Tracking_cost;
		double dt;
		vector<double> u_plan;
		vector<double> u_all;
		
};
#endif
