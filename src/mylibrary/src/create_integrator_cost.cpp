// creates an integrator for non-autonomous ODE, also returning the cost
#include "casadi/casadi.hpp"
#include <iostream>
//#include <casadi_limits.hpp>

using namespace std;
using namespace casadi;

Function create_rk4_integrator(Function f /*transition map*/ , int M /*number of integration steps*/, Function g /*guadrature*/)
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
	for(int i=1; i<=4; i++)
	{
		MXDict k1 = fg(MXDict{{"x", x}, {"u", u}, {"t", t}});
		MXDict k2 = fg(MXDict{{"x", x + k1["k_x"] * dt/2}, {"u", u}, {"t", t}});
		MXDict k3 = fg(MXDict{{"x", x + k2["k_x"] * dt/2}, {"u", u}, {"t", t}});
		MXDict k4 = fg(MXDict{{"x", x + k3["k_x"] * dt}, {"u", u}, {"t", t}});

		x = x + dt/6 * (k1["k_x"] + 2 * k2["k_x"] + 2 * k3["k_x"] + k4["k_x"]);
		q = q + dt/6 * (k1["k_q"] + 2 * k2["k_q"] + 2 * k3["k_q"] + k4["k_q"]);
		
	}



	return Function("fgd", {x0, Ts,  u, t0}, {x, q});
}

//int main()
//{
//	MX t = MX::sym("t", 1);
//	MX px = MX::sym("px",1);
//	MX py = MX::sym("py",1);
//	MX theta = MX::sym("theta",1);
//	MX vx = MX::sym("vx",1);
//	MX vy = MX::sym("vy",1);
//	MX omega = MX::sym("omega",1);
//	MX x = MX::vertcat({px, py, theta, vx, vy, omega});
//	MX ode = 6*x*x*t;
//	Function f = Function("f", {x}, {ode});
//
//	return 0;
//}
//
