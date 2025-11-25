# Simulation

This simulation is developed  with the environment:

- **OS**: Ubuntu 22.04
- **ROS2**: Humble
- **Isaac Sim**: 4.5
- **Isaac Lab**: 2.1.0

## Setup enviorment

**Step 0:** Install [Isaac Sim 4.5](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html) (Download and extract contents in `${HOME}/isaacsim`)

### Verifying the Isaac Sim installation
```bash
# Isaac Sim root directory
export ISAACSIM_PATH="${HOME}/isaacsim"
# Isaac Sim python executable
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
```

Check that the simulator runs as expected:

```bash
# note: you can pass the argument "--help" to see all arguments possible.
${ISAACSIM_PATH}/isaac-sim.sh
```

Check that the simulator runs from a standalone python script:

```bash
# checks that python path is set correctly
${ISAACSIM_PYTHON_EXE} -c "print('Isaac Sim configuration is now complete.')"
# checks that Isaac Sim can be launched from python
${ISAACSIM_PYTHON_EXE} ${ISAACSIM_PATH}/standalone_examples/api/isaacsim.core.api/add_cubes.py
```

**Step 1:** Install Isaac Lab 2.1.0 


## Installing Isaac Lab

You can follow the [IsaacLab](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/binaries_installation.html#verifying-the-isaac-sim-installation) or just follow the tutorial below

Clone the Isaac Lab repository into your project’s workspace:

```bash
./isaaclab.sh --help

usage: isaaclab.sh [-h] [-i] [-f] [-p] [-s] [-t] [-o] [-v] [-d] [-n] [-c] -- Utility to manage Isaac Lab.

optional arguments:
   -h, --help           Display the help content.
   -i, --install [LIB]  Install the extensions inside Isaac Lab and learning frameworks (rl_games, rsl_rl, sb3, skrl) as extra dependencies. Default is 'all'.
   -f, --format         Run pre-commit to format the code and check lints.
   -p, --python         Run the python executable provided by Isaac Sim or virtual environment (if active).
   -s, --sim            Run the simulator executable (isaac-sim.sh) provided by Isaac Sim.
   -t, --test           Run all python pytest tests.
   -o, --docker         Run the docker container helper script (docker/container.sh).
   -v, --vscode         Generate the VSCode settings file from template.
   -d, --docs           Build the documentation from source using sphinx.
   -n, --new            Create a new external project or internal task from template.
   -c, --conda [NAME]   Create the conda environment for Isaac Lab. Default name is 'env_isaaclab'.
   -u, --uv [NAME]      Create the uv environment for Isaac Lab. Default name is 'env_isaaclab'.
```

### Creating the Isaac Sim Symbolic Link

Set up a symbolic link between the installed Isaac Sim root folder and _isaac_sim in the Isaac Lab directory. This makes it convenient to index the python modules and look for extensions shipped with Isaac Sim.

```bash 
# enter the cloned repository
cd IsaacLab
# create a symbolic link
ln -s ${ISAACSIM_PATH} _isaac_sim

ln -s ${HOME}/isaacsim _isaac_sim

```

### Creating a Conda Environment (optional)

```bash
#Default environment name 'env_isaaclab'
./isaaclab.sh --conda
### Activate environment
conda activate env_isaaclab 
```

### Installation

```bash
./isaaclab.sh --install
```

### Verifying the Isaac Lab installation

```bash
python scripts/tutorials/00_sim/create_empty.py
```
![Verify IsaacLab](./verify_isaaclab.jpg)

If you see this, congratulations!

**Step 2:** Install Ros2 Humble

please install [Ros2 Humble](https://docs.ros.org/en/humble/index.html) with the offical installation guide

**Step 3:** Run Unitree Go2 Simulation

```bash
#default conda env
conda activate env_isaaclab
#Clone the repo to you local directory
git clone https://github.com/Zhefan-Xu/isaac-go2-ros2.git

cd isaac-go2-ros2

python isaac_go2_ros2.py
```
Once the simulation is loaded, the robot can be teleoperated by the keyboard
