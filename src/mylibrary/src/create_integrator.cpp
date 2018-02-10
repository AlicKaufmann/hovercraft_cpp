// creates an integrator for non-autonomous ODE, also returning the cost
#include <casadi/casadi.hpp>
#include <create_integrator.hpp>
#include <iostream>
//#include <casadi_limits.hpp>

using namespace std;
using namespace casadi;

Function create_integrator_cost(Function f /*transition map*/ , int M /*number of integration steps*/, Function g /*guadrature*/)
{
	
	MX Ts = MX::sym("Ts");
       	MX u = MX::sym("u", 2);
	
	MX t0 = MX::sym("t0");
	MX t = t0;
	MX x0 = MX::sym("x0",6);
	MX x = x0;
	// create the "augmented" transition map
	
	Function fg = Function("fg", {x, u, t}, {f(MXVector{x,u})[0], g(MXVector{x,u,t})[0]}, {"x", "u", "t"}, {"k_x", "k_q"});
	MX dt = Ts/M;

	MX q = MX::zeros(1,1);
	for(int i=1; i<=M; i++)
	{
		MXDict k1 = fg(MXDict{{"x", x}, {"u", u}, {"t", t}});
		MXDict k2 = fg(MXDict{{"x", x + k1["k_x"] * dt/2}, {"u", u}, {"t", t+dt/2}});
		MXDict k3 = fg(MXDict{{"x", x + k2["k_x"] * dt/2}, {"u", u}, {"t", t+dt/2}});
		MXDict k4 = fg(MXDict{{"x", x + k3["k_x"] * dt}, {"u", u}, {"t", t+dt}});
		t = t + dt;
		x = x + dt/6 * (k1["k_x"] + 2 * k2["k_x"] + 2 * k3["k_x"] + k4["k_x"]);
		q = q + dt/6 * (k1["k_q"] + 2 * k2["k_q"] + 2 * k3["k_q"] + k4["k_q"]);
		
	}



	return Function("fgd", {x0, Ts,  u, t0}, {x, q}, {"x0", "Ts", "u", "t0"}, {"xf", "qf"});
}

// integrator for autonomous ODE which doesn't return cost but more efficient computation
Function create_integrator_rk4(Function f /*transition map*/ , int M /*number of integration steps*/)
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
		cout << f << endl;
		cout << f(MXVector{x,u}) << endl;
		vector<MX> k1 = f(MXVector{x, u});
		vector<MX> k2 = f(MXVector{x + k1[0] * dt/2, u});
		vector<MX> k3 = f(MXVector{x + k2[0] * dt/2, u});
		vector<MX> k4 = f(MXVector{x + k3[0] * dt, u});

		x = x + dt/6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
	}

	//cout << "k1 vaut : " << k1[0] << endl;



	return Function("fd", {x0, Ts,  u}, {x});
}

