#!/usr/bin/env bash

#####

pushd $(dirname ${0}) > /dev/null

mkdir -p build

#####

SOURCES="main.cpp"
OUTPUTS="-o build/main"

STANDARDS="-std=c++11"
OPTIMIZATIONS="-O3 -ffast-math"
WARNINGS="-Wall -Wextra"

INCLUDE_ACADO="-I /usr/local/include/acado"
LINK_ACADO="-l acado_toolkit_s"

#####

g++ ${SOURCES} ${OUTPUTS} ${STANDARDS} ${OPTIMIZATIONS} ${WARNINGS} ${INCLUDE_ACADO} ${LINK_ACADO}
COMPILE_ERROR=${?}

#####

if [[ ${COMPILE_ERROR} == 0 ]]; then
    if [[ ${1} == "-r" ]]; then
        echo "====="
        ./build/main
        echo "====="
        ./plot_results.py build/results.txt
        echo "====="
    fi
else
    echo "(will not run)"
fi

#####

popd > /dev/null
