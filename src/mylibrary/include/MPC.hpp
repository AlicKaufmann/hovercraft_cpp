// MPC.hpp

#ifndef MPC_H
#define MPC_H

#include<casadi>

class MPC
{
	public:
		// constructor
		MPC(Hovercraft_model dynaics, Cost tracking_cost, int N, int dt);
	private:
		int N;
		Function qd;
		Function cost;
		double dt;
		vector<double> u_plan;
		vector<double> x_prev;
		double t_prev;
		vector<double> u_all;
		
};
#endif
