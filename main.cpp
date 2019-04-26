#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

//////////////////////////////////////////////////

using Real = double;
using namespace ACADO;

//////////////////////////////////////////////////

int main(/*int argc, char** argv*/) {

    DifferentialState x, y, q, v, w;
    Control a_v, a_w;
    Real l = 0.5; // m

    IntermediateState bx = x + l*cos(q);
    IntermediateState by = y + l*sin(q);

    Real rx = -7; // m
    Real ry = 7; // m

    Real horizon = 10; // s
    Real mesh = 20;
    OCP ocp(0.0, horizon, mesh);

    DifferentialEquation diffeq(0.0, horizon);
    diffeq << dot(x) == v*cos(q);
    diffeq << dot(y) == v*sin(q);
    diffeq << dot(q) == w;
    diffeq << dot(v) == a_v;
    diffeq << dot(w) == a_w;
    ocp.subjectTo(diffeq);

    ocp.subjectTo(-2.0 <= v <= 2.0); // m/s
    ocp.subjectTo(-1.0 <= w <= 1.0); // rad/s
    ocp.subjectTo(-1.0 <= a_v <= 1.0); // (m/s)/s
    ocp.subjectTo(-0.5 <= a_w <= 0.5); // (rad/s)/s

    Function error;
    uint error_dim = 4;
    DMatrix weight(error_dim, error_dim);
    error << rx - bx; weight(0,0) = 1.0;
    error << ry - by; weight(1,1) = 1.0;
    error <<     a_v; weight(2,2) = 0.5;
    error <<     a_w; weight(3,3) = 0.5;
    ocp.minimizeLSQ(weight, error, DVector(error_dim));

    ocp.subjectTo(AT_START, x == 0.0); // m
    ocp.subjectTo(AT_START, y == 0.0); // m
    ocp.subjectTo(AT_START, q == 0.0); // rad
    ocp.subjectTo(AT_START, v == 0.0); // m/s
    ocp.subjectTo(AT_START, w == 0.0); // rad/s

    OptimizationAlgorithm solver(ocp);
    solver.set(PRINT_COPYRIGHT, false);
    solver.set(INTEGRATOR_TYPE, INT_RK45);
    solver.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    solver.set(DISCRETIZATION_TYPE, SINGLE_SHOOTING);
    solver.set(KKT_TOLERANCE, 1e-6);
    // solver.set(PRINTLEVEL, 0);

    GnuplotWindow window;
    window.addSubplot(bx, by, "by | bx");
    window.addSubplot(q, "q | t");
    window.addSubplot(v, "v | t");
    window.addSubplot(w, "w | t");
    window.addSubplot(a_v, "a_v | t");
    window.addSubplot(a_w, "a_w | t");
    solver << window;

    solver.solve();

    return 0;
}
