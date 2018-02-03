//MPC.cpp

MPC::MPC(Hovercraft_model dynamics, Cost tracking_cost, int N, int dt)
	: cost(tracking_cost)
	; N(N)
	; dt(dt)
	; M(4)
	; u_all({})
{
	
}
