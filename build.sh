#!/usr/bin/env bash

# Compiles the main source file into an executable in a build directory.
# Optionally runs that executable if the -r flag is provided.

#####

# Silently enter the directory of this script
pushd $(dirname ${0}) > /dev/null

# Make a build directory if it doesn't already exist
mkdir -p build

#####

# Compilation Arguments:

SOURCES="main.cpp"  # example source code
OUTPUTS="-o build/main"  # example executable

STANDARDS="-std=c++11"  # necessary to be >= 11 for ACADO
OPTIMIZATIONS="-O3 -ffast-math"  # optional but highly preferred
WARNINGS="-Wall -Wextra"  # completely optional

# (assuming you built and default `sudo make install`ed ACADO)
INCLUDE_ACADO="-I /usr/local/include/acado"
LINK_ACADO="-l acado_toolkit_s"  # the "_s" stands for "shared library" (runtime linking)

#####

# Compile / link, and record return status
g++ ${SOURCES} ${OUTPUTS} ${STANDARDS} ${OPTIMIZATIONS} ${WARNINGS} ${INCLUDE_ACADO} ${LINK_ACADO}
COMPILE_ERROR=${?}

#####

# If compilation succeeded
if [[ ${COMPILE_ERROR} == 0 ]]; then
    # If -r flag was provided
    if [[ ${1} == "-r" ]]; then
        # Run a test
        echo "Running!"
        echo "====="
        ./build/main
        echo "====="
    fi
else
    echo "(will not run)"
fi

#####

# Silently return to caller's directory
popd > /dev/null
