// Cost.cpp

#include "Cost.hpp"

Cost::Cost(Reference ref)
{
	// create cost
	MX u = MX::sym("u", 2);
	MX t = MX::sym("t", 1);
	
	MX px = MX::sym("px", 1);
	MX py = MX::sym("py", 1);
	MX theta = MX::sym("theta", 1);
	MX vx = MX::sym("vx", 1);
	MX vy = MX::sym("vy", 1);
	MX omega = MX::sym("omega", 1);
	
	MX x = MX::vertcat({px, py, theta, vx, vy, omega});
	
	cout << ref.ref(t) << endl;
	MX reft = ref.ref(t)[0];

	MX cost = 100 * (px - pow(reft(1), 2)) +
		100 * (py - pow(reft(2), 2)); // +
	// theta^2 +
	// vx^2 + 
	// vy^2 +
	// u(1)^2 +
	// u(2)^2
	
	Function L = Function("L",{x, u, t}, {cost});
}


