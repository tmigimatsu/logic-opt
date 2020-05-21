# Object-Centric TAMP in Dynamic Environments

This code implements the algorithm in the 2020 IEEE RA-L paper [Object-Centric Task and Motion Planning in Dynamic Environments](https://sites.google.com/stanford.edu/objectcentrictamp).

## Citation

```
@article{migimatsu2019objectcentric,
    title={Object-Centric Task and Motion Planning in Dynamic Environments},
    author={Toki Migimatsu and Jeannette Bohg},
    journal={IEEE Robotics and Automation Letters},
    year={2020},
    volume={5},
    number={2},
    pages={844-851},
    doi={10.1109/LRA.2020.2965875},
}
```

# Installation

## Ipopt

`logic-opt` uses the Ipopt solver with MA-57. Because it is a proprietary
solver, it needs to be downloaded separately.

Obtain an academic license for the full HSL library:
http://www.hsl.rl.ac.uk/ipopt/

Before compiling Ipopt, make sure the necessary packages are installed:
```
sudo apt install pkg-config gfortran automake liblapack-dev libmetis-dev
```

Save the `coinhsl.tgz` file into the `logic-opt` repository and run the install script:
```
cp coinhsl.tgz logic-opt
cd logic-opt
./install_ipopt.sh
```

## logic-opt

Before compiling `logic-opt`, make sure the following packages are installed:
```
sudo apt install python3-dev curl redis-server
```

Next, download the [Rust compiler](https://www.rust-lang.org/tools/install):
```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env
```

Finally, build `logic-opt` with `cmake`:
```
cd logic-opt
mkdir build
cd build
cmake ..  # first time may take around 5 min
make
```

## redis-gl

The visualizer uses [redis-gl](https://github.com/tmigimatsu/redis-gl), a
browser-based interface.
```
git clone https://github.com/tmigimatsu/redis-gl.git
```

Follow the installation instructions in `redis-gl`'s `README.md`.

# Execution

The planner, visualizer, and robot controller communicate via Redis. First, make
sure the server is running.

```
redis-server
```

Next, run the visualizer server.

```
cd redis-gl
pipenv run ./server.py
```

Open the visualizer in the browser at [http://localhost:8000](http://localhost:8000).

Next, run the robot controller. This can remain running in the background.
```
cd logic-opt/bin
./franka_panda_opspace
```

In the visualizer, you should now be able to perturb the robot by ctrl-clicking
a link on the robot.

Finally, we can run the TAMP solver with the Workspace Reach demo.
```
cd logic-opt/bin
./lgp ../resources/config.yaml
```

This will play all the candidate plans in the visualizer in order of
optimization completion time.

The Towers of Hanoi demo can be run with a different config file.
```
cd logic-opt/bin
./lgp ../resources/hanoi_config.yaml
```

# Disclaimer

Recent versions of the collision checker, [ncollide](https://www.ncollide.org), seem to cause instability issues with the nonlinear optimizer. We are currently working to improve the optimization stability.
