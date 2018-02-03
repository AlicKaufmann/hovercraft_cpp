#include <iostream>
#include <casadi/casadi.hpp>
#include <Kalman_filter.hpp>
#include <Hovercraft_model.hpp>
#include <Reference.hpp>
#include <Cost.hpp>

using namespace std;
using namespace casadi;

int main()
{
	MX px = MX::sym("px", 1);
	MX py = MX::sym("py", 1);
	MX theta = MX::sym("theta", 1);
	MX vx = MX::sym("vx", 1);
	MX vy = MX::sym("vy", 1);
	MX omega = MX::sym("omega", 1);
	MX x = MX::vertcat({px, py, theta, vx, vy, omega});
	MX ode = MX::vertcat({px * py, py * vx, theta * omega, vx * px, vy * vy, omega});
	MX u = MX::sym("u", 2);
	MX t0 = MX::sym("t0", 1);
	MX Ts = MX::sym("Ts", 1);

	MX y = MX::vertcat({px,py,theta,omega});

	MX Q = MX::eye(6);
	MX R = MX::eye(4);


	Function fd = Function("fd", {x,Ts, u,t0}, {ode});
	Function h = Function("h", {x}, {y});
	Function jac = fd.jacobian_old(0,0);
	vector<MX> jac_return = jac(MXVector{x, Ts, u,t0});
	MX jacx = jac_return[0];
	MX F = jac_return[0];
	cout << "le jacobien évalué en x vaut " << jacx << endl;
	
	Function dummy = Function("dummy", {x}, {jacx});
	DM arg = DM(6,1);
	arg(0,0)=1;
	arg(1,0)=2;
	arg(2,0)=3;
	arg(3,0)=4;
	arg(4,0)=5;
	arg(5,0)=6;
	cout << "ma fonction dummy est " << dummy << endl;
	cout << dummy(arg) << endl;
	
	MX L = MX::eye(6);
	MX P_prior = mtimes(L, F);

	//Function jac = myFunc.jacobian();
	
	double a = 1;
       	double b = 2;
       	double c = 3;
	//vector<DM> arg = {a,b,c};
	cout << "le jacobien en (" << a << "," << b << "," << c << ") vaut " << endl;
	cout << jac << endl;
	//cout << jac(arg);
	
	
	cout << "creation de la reference" << endl;
	Reference ref;

	cout << "crer le cout" << endl;
	Cost myCost(ref);

	cout << "mon cout est de : " << myCost.L << endl;

	cout << "creating Hovercraft_model object" << endl;
	Hovercraft_model dynamics;

	cout << "creating Kalman_filter object" << endl;
	Kalman_filter ekf(fd, h, Q, R);	
	
	return 0;

}
