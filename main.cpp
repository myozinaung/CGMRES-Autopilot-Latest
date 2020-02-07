#include "nmpc_model.hpp"
#include "continuation_gmres.hpp"
 #include "multiple_shooting_cgmres.hpp"
#include "cgmres_simulator.hpp"
#include <string>
#include <direct.h>

int main()
{
	// Define Simulation Parameters
	double tF = 200;
	double dt = 0.1;
	int solver = 2; // 1 = Single C/GMRES, 2 = Multi C/GMRES 
	std::string filename = "esso_osaka";

	// Initial Conditions
	double x_pos0 = 0;
	double y_pos0 = 0;
	double psi0   = 0 * (pi / 180);
	double u_vel0 = 0.4;
	double v_vel0 = 0;
	double r0     = 0;

	double delta_guess  = 0.01;

	double T_f          = 15; // Prediction Horizon (sec)
	double alpha        = 1.0;
	int horizon_divs    = 40;
	double FD_step      = 1e-6;
	double zeta         = 10; // = (1 / dt)
	int kmax            = 10; // GMRES max iteration no.

		// Set the initial state.
	double initial_state[6] = { x_pos0, y_pos0, psi0, u_vel0, v_vel0, r0 };

	// Set the initial guess of the solution.
	double initial_guess_solution[3] = { delta_guess, delta_guess, delta_guess };

	// Define the model in NMPC.
    cgmres::NMPCModel nmpc_model;

    // Define the solver.
    cgmres::ContinuationGMRES nmpc_solver(T_f, alpha, horizon_divs, FD_step, zeta, kmax);
	// Initialize the solution of the C/GMRES method.
	nmpc_solver.setParametersForInitialization(initial_guess_solution, 1e-06, 100); // tol_res, max_itr (for Newton iteration)

	if (solver == 2) {
		cgmres::MultipleShootingCGMRES nmpc_solver(T_f, alpha, horizon_divs, FD_step, zeta, kmax);

		std::cout << "Multiple Shooting C/GMRES Solver selected." << std::endl;
		filename = "esso_osaka_multi";
	}
	else {

		std::cout << "Single Shooting C/GMRES Solver selected." << std::endl;
		filename = "esso_osaka_single";
	}

        // Makes a directory for saving simulation results.
    std::string save_dir_name("../simulation_result");
    int mkdir_err = _mkdir(save_dir_name.c_str());
	
	// Perform a numerical simulation.
    cgmres::simulation(nmpc_solver, initial_state, 0, tF, dt, save_dir_name, filename); // time step cannot be larger then 0.003
    // t0, tf, dt

    return 0;
}
