#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

//////////////////////////////////////////////////

using Real = double;
using namespace ACADO;

//////////////////////////////////////////////////

int main(int argc, char** argv) {

    DifferentialState x, y, q;
    Control v, w;
    Real l = 0.5; // m

    auto bx = x + l*cos(q);
    auto by = y + l*sin(q);

    Real rx = -7; // m
    Real ry = 7; // m

    Real tf = 5; // s
    Real dt = 0.1; // s
    Real nt = tf / dt;

    DifferentialEquation f(0.0, tf);
    f << dot(x) == v*cos(q);
    f << dot(y) == v*sin(q);
    f << dot(q) == w;

    OCP ocp(0.0, tf, nt);
    ocp.subjectTo(f);

    ocp.subjectTo(-2.0 <= v <= 2.0); // m/s
    ocp.subjectTo(-1.5 <= w <= 1.5); // rad/s

    Function ex; ex << rx-bx;
    Function ey; ey << ry-by;
    ocp.minimizeLSQ(ex);
    ocp.minimizeLSQ(ey);

    Real reg = 0.1;
    Function ev; ev << reg*v;
    Function ew; ew << reg*w;
    ocp.minimizeLSQ(ev);
    ocp.minimizeLSQ(ew);

    ocp.subjectTo(AT_START, x == 0.0); // m
    ocp.subjectTo(AT_START, y == 0.0); // m
    ocp.subjectTo(AT_START, q == 0.0); // rad

    OptimizationAlgorithm solver(ocp);
    solver.set(INTEGRATOR_TYPE, INT_RK23);
    solver.set(HESSIAN_APPROXIMATION, BLOCK_BFGS_UPDATE);
    solver.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    solver.set(KKT_TOLERANCE, 1e-1);

    GnuplotWindow window;
    window.addSubplot(x, "x");
    window.addSubplot(y, "y");
    window.addSubplot(q, "q");
    window.addSubplot(v, "v");
    window.addSubplot(w, "w");
    solver << window;

    solver.solve();

    return 0;
}
