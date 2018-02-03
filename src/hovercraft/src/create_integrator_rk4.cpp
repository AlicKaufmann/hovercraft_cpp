// creates an integrator for autonomous ODE, faster than create_integrator_rk4
#include "casadi/casadi.hpp"
#include <integrator.hpp>
#include <iostream>
//#include <casadi_limits.hpp>

using namespace std;
using namespace casadi;

Function create_rk4_integrator(Function f /*transition map*/ , int M /*number of integration steps*/)
{
	
	MX Ts = MX::sym("Ts");

	MX t = MX::sym("t");
       	MX u = MX::sym("u", 2);
       	MX px= MX::sym("px");
       	MX py = MX::sym("py");
       	MX theta = MX::sym("theta");                   
       	MX vx = MX::sym("vx");
       	MX vy = MX::sym("vy");
       	MX omega = MX::sym("omega");

	MX dt = Ts/M;

	MX x0 = MX::sym("x0",6);
	MX x = x0;

	for(int i=1; i<=4; i++)
	{
		vector<MX> k1 = f(MXVector{x, u});
		vector<MX> k2 = f(MXVector{x + k1[0] * dt/2, u});
		vector<MX> k3 = f(MXVector{x + k2[0] * dt/2, u});
		vector<MX> k4 = f(MXVector{x + k3[0] * dt, u});

		x = x + dt/6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
	}

	//cout << "k1 vaut : " << k1[0] << endl;



	return Function("fd", {x0, Ts,  u}, {x});
}

int main()
{
	double T = 10;
	int N = 6; // number of discretization points
	double dt = T/N;
	
	MX u = MX::sym("u", 2);

       	MX px= MX::sym("px");
       	MX py = MX::sym("py");
       	MX theta = MX::sym("theta");                   
       	MX vx = MX::sym("vx");
       	MX vy = MX::sym("vy");
       	MX omega = MX::sym("omega");

	MX states = MX::vertcat({px, py, theta, vx, vy, omega}); 

	MX ode = MX::vertcat({px, 0, 0, 0, 0, 0} );
	Function f = Function("f", {states, u}, {ode});


	Function fd = create_rk4_integrator(f, 4);
	
	DM x_val = DM(6,1);
	DM u_val = DM(2,1);
	DM Ts_val = DM(1,1);
	x_val(0) = 1;
	x_val(1) = 1;
	x_val(2) = 1;
	x_val(3) = 1;
	x_val(4) = 1;
       	x_val(5) = 1;

	u_val(0) = 10;
	u_val(1) = 11;

	Ts_val(0) = 0.1;
	vector<DM> args = {x_val, Ts_val, u_val};
	cout << "la signature de ma fonction  : " << fd << endl;
	cout << "the value of x at the next step is : " << fd(args) << endl;

	return 0;
}
