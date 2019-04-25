# acado_demo
Example stand-alone project using the *Automatic Control and Dynamic Optimization* (ACADO) Toolkit.

## Instructions (for Linux)
1. Visit the [official ACADO website](https://acado.github.io/) to learn about it
2. Install dependencies with `sudo apt-get install gcc g++ cmake git doxygen gnuplot graphviz`
3. Navigate to the directory where you like to keep third-party source repositories
4. Get the ACADO source code with `git clone https://github.com/acado/acado.git -b stable ACADOtoolkit`
5. Prepare a build directory for ACADO with `cd ACADOtoolkit && mkdir build && cd build`
6. Build the ACADO source code with `cmake .. && make -j`
7. Run a test with `../examples/getting_started/simple_ocp` (should result in a nice plot)
8. If the test was successful, install the ACADO build globally with `sudo make install && sudo ldconfig`
9. Navigate back to the `acado_demo` root directory
10. Examine the source files to understand the ACADO interface
11. Examine the build script to understand minimally compiling with ACADO
12. Run `./build.sh -r`
