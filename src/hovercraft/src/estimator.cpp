#include "casadi/casadi.hpp"
#include <integrator.hpp>
#include <iostream>
//#include <casadi_limits.hpp>

using namespace std;                                                                                    
using namespace casadi;

int main()      
{       
	//horizon
	int N = 100;

	//time
	SX t = SX::sym("t",2);

	// manipulated variable
	SX u = SX::sym("u",2);

	//define the states
	SX px = SX::sym("px",1);
	SX py = SX::sym("py",1);
	SX theta = SX::sym("theta",1);
	SX vx = SX::sym("vx",1);
	SX vy = SX::sym("vy",1);
	SX omega = SX::sym("omega",1);
	
	//process and measurment covariant noise
	SX Q = 0.1 * SX::eye(6);
	SX R = 0.1 * SX::eye(3);

	vector<double> x_post = {0,0,0,0,0,0};
		

	cout << "hello world" << endl;

	return 0;	
}   
