
# What it is

[o80](https://github.com/intelligent-soft-robots/roboball2d) is a software for synchronizing and communicating with processes.
[roboball2d](https://github.com/intelligent-soft-robots/roboball2d) is a lightweight simulation of a 3dofs robot playing with balls.

This repository is used to test idea of software architecture using o80 for synchronisation. 

It is not meant for any other purpose, and no documentation is offered.

# Installation

Installation requires a machine installed with ubuntu 18.04. 

## installing dependencies

After cloning of [ubuntu installation scripts](https://github.com/machines-in-motion/ubuntu_installation_scripts.git), the script setup_ubuntu (in the 'official' folder) can be run, passing the option 'ros':

```bash
sudo ./setup_ubuntu install ros
```

### Note 1
This installation script does not do anything fancy. It (almost) just install software using aptitute and pip, i.e. it does not create some complicated systems difficult to track, or that could break anything else you already have installed on your machine.

### Note 2
If you'd rather not have ros installed, you may run instead:

```bash
sudo ./setup_ubuntu install core
```
Ros is not a required dependency, but is convenient (e.g. rosrun, roslaunch, etc)

### Note 3

You may also pull the corresponding docker image : docker.is.localnet:5000/amd/18.04:ros
You need first to login to docker.is.localnet:

```bash
docker login https://docker.is.localnet:5000
```

In case of issue, you may try to add to /etc/docker/daemon.json:

```bash
{
    "insecure-registries": [ "docker.is.localnet:5000" ]
}
```

and restart docker:

```bash
sudo systemctl daemon-reload
sudo systemctl restart docker
```

## SSH setup

You need a github account. Set up your ssh key to github : [github help](https://help.github.com/en/enterprise/2.17/user/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)

## Cloning treep project

Create a new workspace and clone treep_isr ('isr'='intelligent soft robots')

```bash
mkdir Software # you may call the folder whatever you like
cd Software
git clone git@github.com:intelligent-soft-robots/treep_isr.git
treep --clone O80_ROBOBALL2D
```

## Compiling

```bash
cd workspace
source /opt/ros/melodic/setup.bash 
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 install
```

## Setting up bashrc file

To make sure things are activated properly in all new terminals you open, you may add to your ~/.bashrc file
(replacing <absolute path to Software> by what makes sense):

```bash
echo "sourcing o80 roboball2d"
source /opt/ros/melodic/setup.bash
LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:<absolute path to Software>/workspace/devel/lib
source <absolute path to Software>/workspace/install/setup.bash
```
Before continuing, source ~/.bashrc or open a new terminal.

# Running the example

All python scripts are in :

```bash
cd <absolute path to Software>/workspace/src/o80/o80_roboball2d/scripts
```

Must be run in this order:

## starting the pseudo real robot

In a terminal:

```bash
python3 ./run_reality.py
```

A window showing a robot should pop up.

## starting simulated robot and simulated ball

In another terminal:

```bash
python3 ./run_simulation.py
```

A window showing a robot should pop up.

## starting the experiment:

In another terminal:

```bash
python3 ./env.py
```

Ball should be flying around and the real robot / simulated robot moving

## Exit

- close the window in which env.py run
- close the window in which run_simulation.py run
- cltr-c for exiting run_reality.py and run_vision.py

It is possible to exit env.py and run_simulation.py, but keeping run_reality.py running (and starting run_simulation.py and env.py again)

# Troubleshooting

Issues may occur if programs are not exited cleanly, or not in the right order. This may block the applications to be started again. If this happens, cleaning the shared memory should help:

```bash
sudo rm /dev/shm/*
```
 
# Limitations

This software has been created for the purpose of testing o80. It is not expected to be used beyond this, and no documentation is offered.
 
 # Author
 
 Vincent Berenz, Max Planck Institute for Intelligent Systems
 
