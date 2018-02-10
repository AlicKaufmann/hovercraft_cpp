//Hovercraft_model.cpp

#include "Hovercraft_model.hpp"

Hovercraft_model::Hovercraft_model()
{
	MX u = MX::sym("u", 2);
	MX t = MX::sym("t", 1);
	
	MX px = MX::sym("px", 1);
	MX py = MX::sym("py", 1);
	MX theta = MX::sym("theta", 1);
	MX vx = MX::sym("vx", 1);
	MX vy = MX::sym("vy", 1);
	MX omega = MX::sym("omega", 1);
	
            
	vector<double> uTp = {-100, -90, -80, -70, -60, -50, -30, -20, 0,20,30,50,60,70,80,90,100};
	vector<double> uT = {-1.5340, -1.3959, -1.1630, -1.022, -0.7839, -0.6309, -0.3850, -0.2244, 0, 0.2244, 0.3850, 0.6309, 0.7839, 1.022, 1.1630, 1.3959, 1.5340};
	vector<double> uRp = {-100, -90, -80, -70, -60, -50, -40, -30,-20,-10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
	vector<double> uR = {-96.1055, -89.0708, -83.5957, -60.5996, -53.0103, -45.7588, -35.5810, -25.6822, -16.8374, -6.3632, 0, 6.3632, 16.8374, 25.6822, 35.5810, 45.7588, 53.0103, 60.5996, 83.5957, 89.0708, 96.1055};
	
	Function transMap = interpolant("transMap", "bspline", {uTp}, uT);
	Function rotMap = interpolant("rotMap", "bspline", {uRp}, uR);
	
        MX states = MX::vertcat({px, py, theta, vx, vy, omega});
        MX rhs = MX::vertcat({
	    vx,
            vy,
            omega,
            transMap(u(0))[0]*cos(theta + pi/2),
            transMap(u(0))[0]*sin(theta + pi/2),
            rotMap(u(1))[0]});
	
	MX y = MX::vertcat({px, py, theta, omega});
        
        f = Function("f", {states, u},{rhs},{"x","u"},{"rhs"}); // fonction of the dynamics
        h = Function("h", {states},{y}); // measurment function
	cout << f(DMVector{DM::vertcat({1,2,3,4,5,6}), DM::vertcat({10,20})}) << endl;
}
