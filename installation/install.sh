#!/bin/bash

default=n

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd "$MY_PATH"

CERES_PATH=$MY_PATH/../lib
CERES_VERSION=1.14.0

# IMPORTANT: These variables should match the settings of your catkin workspace
PROFILE="RelWithDebInfo" # RelWithDebInfo, Release, Debug
BUILD_WITH_MARCH_NATIVE=false
if [ ! -z "$PCL_CROSS_COMPILATION" ]; then
  BUILD_WITH_MARCH_NATIVE=$PCL_CROSS_COMPILATION
fi
CMAKE_STANDARD=17

#Ceres Solver
while true; do
  [[ -t 0 ]] && { read -t 5 -n 2 -p $'\e[1;32mInstall Ceres-Solver(required for VINS-Mono)? [y/n] (default: '"$default"$')\e[0m\n' resp || resp=$default ; }
  response=`echo $resp | sed -r 's/(.*)$/\1=/'`

  if [[ $response =~ ^(y|Y)=$ ]]
  then
    cd ~/git

    echo "Installing ceres solver"
    # Ceres installation - dependencies

    # CMake
    sudo apt-get -y install cmake
    # google-glog + gflags
    sudo apt-get -y install libgoogle-glog-dev
    # BLAS & LAPACK
    sudo apt-get -y install libatlas-base-dev
    # Eigen3
    sudo apt-get -y install libeigen3-dev
    # SuiteSparse and CXSparse (optional)
    # - If you want to build Ceres as a *static* library (the default)
    #   you can use the SuiteSparse package in the main Ubuntu package
    #   repository:
    sudo apt-get -y install libsuitesparse-dev
    # - However, if you want to build Ceres as a *shared* library, you must
    #   add the following PPA:
    # sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
    # sudo apt-get update
    # sudo apt-get install libsuitesparse-dev

    # Build with march native?
    if $BUILD_WITH_MARCH_NATIVE; then
      echo "Building ceres optimizer with -march=native"
      CMAKE_MARCH_NATIVE="-march=native"
    else
      echo "Building ceres optimizer without -march=native"
      CMAKE_MARCH_NATIVE=""
    fi

    # Profile-dependent flags
    if [[ "$PROFILE" == "RelWithDebInfo" ]]; then
      BUILD_FLAGS_PROFILE=(
                      -DCMAKE_CXX_FLAGS_${PROFILE^^}="-O2 -g"
                      -DCMAKE_C_FLAGS_${PROFILE^^}="-O2 -g")
    elif [[ "$PROFILE" == "Release" ]]; then
      BUILD_FLAGS_PROFILE=(
                      -DCMAKE_CXX_FLAGS_${PROFILE^^}="-O3"
                      -DCMAKE_C_FLAGS_${PROFILE^^}="-O3")
    else
      BUILD_FLAGS_PROFILE=(
                      -DCMAKE_CXX_FLAGS_${PROFILE^^}="-O0 -g"
                      -DCMAKE_C_FLAGS_${PROFILE^^}="-O0 -g")
    fi

    # Defaults taken from mrs_workspace building flags
    BUILD_FLAGS_GENERAL=(
                  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
                  -DCMAKE_CXX_STANDARD=$CMAKE_STANDARD
                  -DCMAKE_BUILD_TYPE=$PROFILE
                  -DCMAKE_CXX_FLAGS="-std=c++$CMAKE_STANDARD $CMAKE_MARCH_NATIVE"
                  -DCMAKE_C_FLAGS="$CMAKE_MARCH_NATIVE"
                  -DBUILD_TESTING=OFF
                  -DBUILD_DOCUMENTATION=OFF
                  -DBUILD_BENCHMARKS=OFF
                  -DBUILD_EXAMPLES=OFF
                  -DSCHUR_SPECIALIZATIONS=ON
                )

    # download ceres solver
    echo "Downloading ceres solver"
    [ ! -d $CERES_PATH ] && mkdir -p $CERES_PATH
    cd $CERES_PATH

    if [ ! -d $CERES_PATH/ceres-solver-$CERES_VERSION ]
    then
      # unpack source files
      wget -O "$CERES_PATH/ceres-solver-$CERES_VERSION.tar.gz" http://ceres-solver.org/ceres-solver-$CERES_VERSION.tar.gz
      tar zxf ceres-solver-$CERES_VERSION.tar.gz
      rm -f ceres-solver-$CERES_VERSION.tar.gz
    fi

    # install ceres solver
    echo "Compiling ceres solver"
    cd $CERES_PATH/ceres-solver-$CERES_VERSION
    [ ! -d "build" ] && mkdir build
    cd build
    cmake "${BUILD_FLAGS_GENERAL[@]}" "${BUILD_FLAGS_PROFILE[@]}" ../

    [ -z "$GITHUB_CI" ] && N_PROC="-j$[$(nproc) - 1]"
    [ ! -z "$GITHUB_CI" ] && N_PROC="-j$[$(nproc) / 2]"

    echo "building with $N_PROC processes"

    make ${N_PROC}
    sudo make install

    echo "Done installing prerequisities for vins-mono"

    # remove the ceres solver source and build files
    rm -rf $CERES_PATH

    break
  elif [[ $response =~ ^(n|N)=$ ]]
  then
    break
  else
    echo " What? \"$resp\" is not a correct answer. Try y+Enter."
  fi
done

