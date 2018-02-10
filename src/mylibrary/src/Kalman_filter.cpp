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
	vector<MX> jac_return = jac(MXVector{x_post, Ts, u});
	MX jacx = jac_return[0];
	cout << "le jacobien évalué en x vaut " << jacx << endl;
	

	MX F = jac_return[0];
	MX L = MX::eye(6);
	MX P_prior = mtimes(mtimes(F, P_post), F.T()) + mtimes(mtimes(L, Q), L.T());
	MX x_prior = fd(MXVector{x_post, Ts, u})[0];
	cout << "x_prior : " << x_prior << endl;
	predict = Function("predict", {x_post, P_post, Ts, u}, {x_prior, P_prior},{"x_upd", "P_upd", "Ts", "u"}, {"x_pre", "P_pre"});

	// update function
	MX M = MX::eye(4);
	DM H = DM::vertcat({DM::horzcat({1,0,0,0,0,0}), DM::horzcat({0,1,0,0,0,0}), DM::horzcat({0,0,1,0,0,0}), DM::horzcat({0,0,0,0,0,1})});
	MX y = MX::sym("y", 4);
	MX P_prior_ = MX::sym("P_prior", 6, 6);
	MX x_prior_ = MX::sym("x_prior", 6, 1);
	
	// Kalman Gain. To solve, AB = C for A, use forward slash A=C/B or backward slash A=(B'\C')', i.e. A=solve(B.T(),C.T()).T() in C++
	//DM B = DM(2,3);
	//B(0,0)=1;B(0,1)=2;B(0,2)=3;B(1,0)=4;B(1,1)=5;B(1,2)=6;
	//DM A = DM(3,2);
	//A(0,0)=1;A(0,1)=7;A(1,0)=8;A(1,1)=9;A(2,0)=10;A(2,1)=11;
	//cout << A.size() << endl;
	//cout << B.size() << endl;
	//DM C = mtimes(A,B);
	//cout << C << endl;
	//DM AA = solve(mtimes(B,B.T()), mtimes(B,C.T())).T();	
	//cout << A << endl;
	//cout << AA << endl;
	//Solve normal equations AB=C => B'A'=C' => BB'A'=BC' => A=(BB'\BC')'
	MX A = mtimes(mtimes(H,P_prior_),H.T()) + times(times(M,R), M.T());
	MX K = solve(mtimes(A, A.T()), mtimes(mtimes(A,H),P_prior_.T()), "ldl").T();
	// MX K = solve((mtimes(mtimes(H,P_prior_), H.T()) + mtimes(mtimes(M,R), M.T())).T(), (mtimes(P_prior_, H.T())).T()).T(); // Kalman gain
	cout << K << endl;
	cout << K.size() << endl;
	x_post = x_prior_ + mtimes(K, y - h(x_prior_)[0]);
	P_post = mtimes(MX::eye(6)-mtimes(K,H), P_prior_);
	update = Function("update", {x_prior_, P_prior_, y}, {x_post, P_post}, {"x_pre", "P_pre", "y"}, {"x_upd", "P_upd"});
}
