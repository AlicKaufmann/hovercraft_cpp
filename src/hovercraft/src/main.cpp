#include <iostream>
#include <casadi/casadi.hpp>
#include <Kalman_filter.hpp>
#include <Hovercraft_model.hpp>
#include <Reference.hpp>
#include <Cost.hpp>
#include <MPC.hpp>
#include <create_integrator.hpp>
#include <HC_node.hpp>

using namespace std;
using namespace casadi;

int main(int argc, char** argv)
{
	
	int N = 3; // horizon for MPC controller
	double T = 10; // final time
	int Ns = 100; // number of shooting nodes
	double dt = T/Ns; // time step
	double noise_prefac = 0.1;

	MX Q = MX::eye(6);
	MX R = MX::eye(4);

	DM u = DM(2,1); // control input for main loop
	
	// create object with transition and measurement map
	cout << "creating Hovercraft_model object" << endl;
	Hovercraft_model dynamics;

	//create an integrator
	int M = 4;
	Function fd = create_integrator_rk4(dynamics.f, M);

	cout << "creating Kalman_filter object" << endl;
	Kalman_filter ekf(fd, dynamics.h, Q, R);	

	cout << "creation de la reference" << endl;
	Reference ref;
	cout << ref.ref(DMVector{3.58}) << endl;

	cout << "crer le cout" << endl;
	Cost myCost(ref);

	cout << "create MPC class" << endl;
	MPC MPC_controller(dynamics, myCost, N, dt);

	// MPC control, should be put into separate function afterwards
	double t = 0;
	DM P_upd = DM::eye(6);
	DM x_upd = DM::vertcat({2,0,0,0,0,0});
	DMDict updated = {{"x_upd", x_upd}, {"P_upd", P_upd}};
	DM x_pre = DM(6,1);
	DM P_pre = DM(6,6);
	DMDict prediction = {{"x_pre", x_pre}, {"P_pre", P_pre}};
	vector<DM> x_collect(Ns+1);
	vector<DM> P_collect(Ns+1);
	x_collect[0] = x_upd;
	P_collect[0] = P_upd;
	vector<double> y_double; // vector of  measurments	
	DM y = DM(4,1); // DM-version of vector of measurments 

	// initialise ROS
	ros::init(argc, argv, "listenerMultipleTopics");

	// create the ROS node for receiving the y datas and sending the u commands.
	HC_node Ros_node; 
	
	// test the predict and update function, predict gives same as matlab but update not. Which is wrong? 
	DM P = DM::vertcat({DM::horzcat({1,2,3,4,5,6}), DM::horzcat({7,8,9,10,11,12}), DM::horzcat({13,14,15,16,17,18}), DM::horzcat({19,20,21,22,23,24}), DM::horzcat({25,26,27,28,29,30}), DM::horzcat({31,32,33,34,35,36})});
	cout << ekf.predict(DMVector{DM::vertcat({1,2,3,4,5,6}), P, DM{0.1}, DM::vertcat({10,20})}) << endl;
	cout << ekf.update(DMVector{DM::vertcat({1,2,3,4,5,6}), P, DM::vertcat({1,2,3,4})});

	for(int i=1; i<=Ns; i++)
	{
		DM u_plan = MPC_controller.planning(x_upd,t);
		// cout << u_plan << endl;
		u = u_plan(Slice(0,2,1));
		
		// apply the input
		Ros_node.send_cmd(0,10);
	
		// EKF
		// prediction step 
		prediction = ekf.predict(DMDict{{"x_upd", prediction["x_upd"]}, {"P_upd", prediction["P_upd"]}, {"Ts", dt}, {"u", u}});	
		
		//here we should wait until we are a time d+dt
		t = t + dt;

		// collect the vector of outputs
		ros::spinOnce();
		y_double = Ros_node.y;
		DM y = DM::vertcat(DMVector{y_double});
		cout << y << endl;
		
		// update step
		updated = ekf.update(DMDict{{"x_pre", prediction["x_pre"]},{"P_pre", P_upd},{"y", y}});
	}

	return 0;

}
