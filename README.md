# PDDL planner:

## Installation

```
mkdir build
cd build
cmake .. -DBUILD_OPTIMIZER=OFF
make -j8
```

## Running
```
bin/pddl resources/domain.pddl resources/problem.pddl
```

# Optimizer:

Download a proprietary linear solver for IPOPT (using MA57):
https://www.coin-or.org/Ipopt/documentation/node6.html
http://www.hsl.rl.ac.uk/ipopt/

Before running cmake, run external/make\_hsl.sh

#For HSL:
#- ./configure
#- make
#- sudo make install
#
#sudo apt install flex bison
