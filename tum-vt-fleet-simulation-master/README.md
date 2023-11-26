# TUM-VT Fleet Simulation
See documentation/ directory for more detailed descriptions.

## Dependency installation

### Conda workflow for virtual environment
Prerequisite: anaconda installed; Example in this instruction: 
**Anaconda version: 201903; conda version: 4.9.2**

After installing the Anaconda, we could open the *Anaconda Prompt* to execute the following codes. First, let's change the working directory.
```
cd <working_directory>
```

#### Control the channel priority
It is strongly recommended by [GeoPandas](https://geopandas.org/install.html "geopandas_installation") to either install everything from the defaults channel, or everything from the *conda-forge* channel. Ending up with a mixture of packages from both channels for the dependencies of *GeoPandas* can lead to importing problems.

In this instruction, we choose the channel *conda-forge*. To achieve this, first, we could add the channel *conda-forge* by
```
conda config --env --add channels conda-forge
```

You should check all your channels by:

```
conda config --show channels
```

If *conda-forge* is not on top of your list, please add the channel once more to put it on top of the list.


To restrict the channel, use the following code:
```
conda config --env --set channel_priority strict
```

Which basically installs packages with same names strictly by channel priority, and as it is on top of the list, the higher channel is the *conda-forge*. To check more detailed information about the channel priority, input the following code:

```
conda config --describe channel_priority
```

#### Create the virtual environment
Create a new virtual environment in an *Anaconda Prompt* with a Python version 3.7.X.

```
conda create -n <new_env> python=3.7
```

Checkout to the created virtual environment

```
conda activate <new_env>
```

Check the default packages in your new and clean virtual environment :smile:

```
conda list
```

Install packages direcly using *Conda*:

```
conda install --file requirements.txt
```

It works! Looks like every package is successfully installed! Now check the installed packages again :wink:

```
conda list
```

Everything is set up! :thumbsup: Now you could run your first simulation!

### C++ Router
<!-- waiting for Roman and Yunfei to supplement -->

### Optimizer

* Gurobi:
Set gurobi channel on top of your channel list by twice calling
```
conda config --add channels http://conda.anaconda.org/gurobi
```
Install gurobi package by
```
conda install gurobi
```
Free academic licenses of Gurobi can be acquired. See https://www.gurobi.com/academia/academic-program-and-licenses/ for more details in installation instructions.

<!-- waiting for Yunfei to supplement; check the packages gurobi and cplex -->


## Data Preparation
... (prepare study by config.csv and scenarios.csv)
... (necessary modules for preprocessing can be installed by "pip3 install -r requirements_with_pp.txt")


## IDE Setup
Additionally, to the tum-vt-fleet-simulation directory, you should set the 'FleetPy' submodule directory as source directory to not receive import warnings.
If you start simulations via 'run_dev.py', the path is automatically adjusted.


## Usage
<!-- waiting for GUI Scenario Creator for further information -->
.. (run from cs file)
```
python3 app.py
```

## Tested on

Windows 10 Pro x64
Chrome 79.0.3945
Python 3.7