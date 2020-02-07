//
// Define parameters and equations of your model in this file.
//

#ifndef NMPC_MODEL_H
#define NMPC_MODEL_H

#include "mmg_eo.hpp" // StateODE class
#include "hamiltonian.hpp" // hamiltonian class

namespace cgmres {
// This class stores parameters of NMPC and equations of NMPC.
class NMPCModel{
private:

    static constexpr double dhu = 1e-4;
    static constexpr double dhx = 1e-4;
    // Define parameters of your model here using "static constexpr".
    
    hamiltonian ham;
    const int dim_state_         = ham.dim_state_;
    const int dim_control_input_ = ham.dim_control_input_;
    const int dim_constraints_   = ham.dim_constraints_;

    double* q_terminal = ham.q_terminal;
    double* x_ref      = ham.x_ref;
    
public:
    // State equation of the model.
    void stateFunc(const double t, const double* x, const double* u, double* f);

    // Partial derivative of the terminal cost with respect to state.
    void phixFunc(const double t, const double* x, double* phix);

    // Partial derivative of the Hamiltonian with respect to state.
    void hxFunc(const double t, const double* x, const double* u, const double* lmd, double* hx);

    // Partial derivative of the Hamiltonian with respect to control input and constraints.
    void huFunc(const double t, const double* x, const double* u, const double* lmd, double* hu);


    // Returns the dimension of the state.
    int dim_state() const;

    // Returns the dimension of the contorl input.
    int dim_control_input() const;

    // Returns the dimension of the constraints.
    int dim_constraints() const;

};

} // namespace cgmres

#endif
