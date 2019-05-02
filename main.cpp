/*
Example of using ACADO for MPC of a tank-drive ground vehicle.
*/
#include <ctime> // just for testing run-times
#include <acado_toolkit.hpp> // everything you really need
#include <acado_gnuplot.hpp> // just for convenient plotting within C++

using namespace ACADO;
int main(/*int argc, char** argv*/) {

    // Declare optimal control problem
    float horizon = 10.0; // time-window of problem relevance, s
    uint mesh = 20; // number of grid points to discretize the horizon into, s/s
    OCP ocp(0.0, horizon, mesh); // args: t_init, t_fin, n_points

    // Declare variables for the problem
    // (the order of declaration here will determine their ordering in later functionality)
    DifferentialState x, y, q, v, w; // variables continuous with time: xpos, ypos, heading, vel, angvel
    IntermediateState bx, by; // variables fully-defined by others: payload_xpos, payload_ypos
    Control a_v, a_w; // decision variables discontinuous with time: linaccel, angaccel
    float l, rx, ry; // non-symbolic parameters (wont be optimized): length, target_xpos, target_ypos

    // Assign any known values or expressions to the problem variables
    // (non-symbolics must be assigned first since they are used at face-value in expressions)
    l = 0.5; // m
    rx = -7.0; // m
    ry = 7.0; // m
    bx = x + l*cos(q); // (this is an expression), m
    by = y + l*sin(q); // m

    // Provide the problem with differential equations that define the states over the horizon
    // (the order of these statements is irrelevant)
    DifferentialEquation dynamic(0.0, horizon); // args define time-window over which it is valid
    dynamic << dot(x) == v*cos(q);
    dynamic << dot(y) == v*sin(q);
    dynamic << dot(q) == w;
    dynamic << dot(v) == a_v;
    dynamic << dot(w) == a_w;
    ocp.subjectTo(dynamic);

    // Provide the problem with some bounds on particular expressions
    // (here the expressions happen to all be single variables)
    ocp.subjectTo(-2.0 <= v <= 2.0); // m/s
    ocp.subjectTo(-1.0 <= w <= 1.0); // rad/s
    ocp.subjectTo(-1.0 <= a_v <= 1.0); // (m/s)/s
    ocp.subjectTo(-0.5 <= a_w <= 0.5); // (rad/s)/s

    // Provide the problem with an objective
    Function error; // 'Function' is the algebraic version of 'DifferentialEquation'
    uint error_dim = 4;
    DMatrix weight(error_dim, error_dim); // weight matrix for least-squares
    error << rx - bx; weight(0,0) = 1.0; // want position of payload...
    error << ry - by; weight(1,1) = 1.0; // ... to match position of target
    error <<     a_v; weight(2,2) = 0.5; // regularize the controls, and notice the ordering...
    error <<     a_w; weight(3,3) = 0.5; // ... of these equations matches the corresponding weight entries
    ocp.minimizeLSQ(weight, error, DVector(error_dim)); // args: weight_matrix, residual_function, reference (zero here)

    // Define a solver for this problem
    float mpc_rate = 10.0; // Hz
    RealTimeAlgorithm solver(ocp, 1/mpc_rate); // args: problem, mpc_period
    solver.set(PRINT_COPYRIGHT, false);
    solver.set(PRINTLEVEL, 0);
    solver.set(INTEGRATOR_TYPE, INT_RK45);
    solver.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON); // GAUSS_NEWTON is for LSQ only, else I recommend BLOCK_BFGS_UPDATE
    solver.set(DISCRETIZATION_TYPE, SINGLE_SHOOTING); // listen bro *direct* multiple-shooting is overrated

    /*************************************************
    At this point we could do code-generation,
    but runtime solving is probably fast enough.
    *************************************************/

    // For convenience, add a plot to our solver for later viewing of the solutions
    GnuplotWindow plot(PLOT_NEVER); // we will manually trigger the plotting
    plot.addSubplot(bx, by, "by | bx"); // any expressions you want by the way!
    plot.addSubplot(q, "q | t");
    plot.addSubplot(v, "v | t");
    plot.addSubplot(w, "w | t");
    plot.addSubplot(a_v, "a_v | t");
    plot.addSubplot(a_w, "a_w | t");
    solver << plot;
    solver.set(PLOT_RESOLUTION, HIGH);

    // Initial condition vector (the order here is the order of 'DifferentialState' declarations)
    DVector x0(5);
    x0(0) = 0.0; // x, m
    x0(1) = 3.0; // y, m
    x0(2) = 0.0; // q, rad
    x0(3) = 0.0; // v, m/s
    x0(4) = 0.0; // w, rad/s

    // Each call to 'solve' does one iteration of optimization,
    // so loop it to watch a solution converge, and modify x0
    // using a state estimator for online applications
    bool failed = false;
    uint max_iters = 20;
    std::cout << std::endl;
    for(uint i=0; i<max_iters; ++i) {
        // Run solve step and time it
        std::clock_t begin_time = std::clock();
        failed = solver.solve(0.0, x0); // args: t_now, state_now
        std::clock_t end_time = std::clock();
        // Print status
        std::cout << "Iter: " << i << " | "
                  << "Cost: " << solver.getObjectiveValue() << " | "
                  << "Delay: " << 1000*double(end_time-begin_time)/CLOCKS_PER_SEC << " ms" << std::endl;
        // Error handling
        if(failed) {
            std::cout << "Solve failed! (ACADO prints the full details)" << std::endl;
            break;
        }
        std::cout << std::endl;
        /* How to get info out of the solver:
        solver.getDifferentialStates(states); // put state info into a VariablesGrid
        x0 = states.getVector(1); // "simulate" by assigning predicted next state to current state
        solver.getU(u); // put control info into a DVector, in the order of Control declarations
        */
    }

    // Plots will remain visible as they are run by a separate GNUplot process
    solver.plot(); // non-blocking
    return 0;
}
