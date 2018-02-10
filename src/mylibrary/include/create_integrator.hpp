// header file for integration routines

// creates an integrator for non-auatonomous ODE which also returns the cost quadrature
casadi::Function create_integrator_cost(casadi::Function f, int M, casadi::Function g);

//creates an integrator for autonomous ODE which doesn't return the cost quadratur but is more efficient
casadi::Function create_integrator_rk4(casadi::Function f /*transition map*/ , int M /*number of integration steps*/);
