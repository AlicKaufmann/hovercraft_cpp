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
	
	//cout << ref.ref << endl;	
	//cout << ref.ref(t) << endl;
	//Function aux = ref.ref;
	//cout << aux(DMVector{0}) << endl;
	//cout << aux(DMVector{1}) << endl;

	MX reft = ref.ref(t)[0];
	cout << reft << endl;

	MX cost = 100 *pow((px - reft(0)), 2) +
		100 *pow((py - reft(1)), 2); // +
	// theta^2 +
	// vx^2 + 
	// vy^2 +
	// u(1)^2 +
	// u(2)^2
	
	L = Function("L",{x, u, t}, {cost});
	cout << L << endl;
	cout << L(DMVector{DM::vertcat({1,2,3,4,5,6}),DM::vertcat({10,20}),8}) << endl;
}


