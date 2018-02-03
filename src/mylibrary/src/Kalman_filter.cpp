//Kalman_filter.cpp
#include "Kalman_filter.hpp"

Kalman_filter::Kalman_filter(Function fd, Function h, MX Q, MX R)
	: Q(Q)
	, R(R)
	 
{
	// create predict function
	MX x_post = MX::sym("x_post", 6);
	MX P_post = MX::sym("P_post", 6, 6);
	MX Ts = MX::sym("Ts", 1);
	MX u = MX::sym("u", 2);
	MX t0 = MX::sym("t0", 1);

	Function jac = fd.jacobian_old(0,0);
	cout << fd << endl;
	cout << jac << endl;
	vector<MX> jac_return = jac(MXVector{x_post, Ts, u, t0});
	MX jacx = jac_return[0];
	//cout << "le jacobien évalué en x vaut " << jacx << endl;
	

	MX F = jac_return[0];
	MX L = MX::eye(6);
	MX P_prior = mtimes(mtimes(F, P_post), F.T()) + mtimes(mtimes(L, Q), L.T());
	cout << "le machin vaut " << fd << endl;
	//vector<MX> x_prior = fd(x_post, Ts, u, t0);
	//cout << "x_prior : " << x_prior << endl;
	//Function predict = Function("predict", {x_post, P_post, Ts, u, t0}, {x_prior, P_prior},{"x_post", "P_post", "Ts", "u", "t0"}, {"x_prior", "P_prior"});
}
