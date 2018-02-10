//MPC.cpp
#include <MPC.hpp>
#include <create_integrator.hpp> 

MPC::MPC(Hovercraft_model dynamics, Cost Tracking_cost, int N, double dt)
	: Tracking_cost(Tracking_cost)
	, N(N)
	, dt(dt)
	, u_all({})
{
	int M = 4; // change if need more integration steps
	qd = create_integrator_cost(dynamics.f, M, Tracking_cost.L);
	cout << "qd au point .. vaut : " << qd(DMVector{DM::vertcat({2,0,0,0,0,0}), 0.1, DM::vertcat({50,50}), 0}) << endl;
}

DM MPC::planning(MX x, double t)
{
	MX xk = x;

	//initialise time in prediction to initial time
	MX pred_t = t;

	// initialise values of optimal problem
	vector<MX> w = {};
	vector<DM> w0 = {};
	vector<DM> lbw = {};
	vector<DM> ubw = {};
	MX J = MX::zeros(1,1);
	vector<MX> g = {};
	vector<DM> lbg = {};
	vector<DM> ubg = {};

	for(int k=1; k<=N-1; k++)
	{
		cout << dt << endl;
		MX uk = MX::sym("u" + to_string(k), 2);
		w.push_back(uk);
		//put this outside loop when you have time
		lbw.push_back(DMVector{-100, -100});
		ubw.push_back(DMVector{100, 100});
		w0.push_back(DMVector{0,0});

		// integrate on one timestep
		MXDict fdk = qd(MXDict{{"x0", xk},{"Ts", dt},{"u", uk},{"t0", pred_t}});
		xk = fdk["xf"];
		J = J + fdk["qf"];

		//add inequality constraints
		g.push_back(xk(Slice(0,2,1)));	//constrained by size of table
		lbg.push_back(DMVector{-4,-6});
		ubg.push_back(DMVector{4,6});
		pred_t = pred_t + dt;
	}
	cout << MX::vertcat(w) << endl;
	cout << DM::vertcat(w0) << endl;
	cout << MX::vertcat(g) << endl;
	// solve optimal problem
	
	MXDict nlp = {{"f", J}, {"g", MX::vertcat(g)}, {"x", MX::vertcat(w)}};
	Function solver = nlpsol("S", "ipopt", nlp);
	
	DMDict arg ={{"x0", DM::vertcat(w0)}, {"lbg", DM::vertcat(lbg)}, {"ubg", DM::vertcat(ubg)}, {"lbx", DM::vertcat(lbw)}, {"ubx", DM::vertcat(ubw)}};
	DMDict res = solver(arg);
	//u_plan = res["x"]; 
	cout << res << endl;
	cout << "le résultat de l'optimisation est : " << res["x"] << endl; 
	return res["x"];
}
