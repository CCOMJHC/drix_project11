# Project11 on Drix 8

Details on Project11's installation on Drix 8.

## ROS namespaces

DriX uses ROS running a core on the a machine called the mdt on the drix as well as a core on the robobox at the operator's location.  In order to minimize name conflicts, Project11 uses the `/project11` topic prefix for most topics. A robot part (`/project11/drix_8`) is added on DriX and an operator part (`/project11/operator`) is added on the operator station.

## On Drix 8

An Intel NUC running Ubuntu 20.04 with ROS Noetic is installed as a backseat driver on Drix #8.

### Network config

IP addresses configured on a single NIC with associated names in standard CCOM/JHC Project11 host file.

- 192.168.8.180/24 echod (Drix 8 network)
- 192.168.50.180/24 echo (Project11 operator network)
- 192.168.179.180/24 (Sonardyne transducer network)

### Startup

The following scripts are run automatically by cron on startup.

`@reboot /bin/bash /home/field/project11/catkin_ws/src/drix_project11/scripts/start_tmux_project11.bash`
`@reboot /bin/bash /home/field/km_convert/autostart_km_convert.bash`

The km_convert script listens for packets from the Phins motion sensor and converts the data to a form acceptable be the Sonardyne Ranger software.

The Project11 script launches the `drix_project11` package's `drix.launch` file.

### Drix launch file

The `drix.launch` file requires two arguments that are set by the startup script. The `drixNumber` argument allows the launch file to be used with the simulator. The `logDirectory` is used to specify the location where to save bag files.

The optional `operator_ip` argument specifies where the udp_bridge sends data and is set to 192.168.8.182 (as of this writing) in the startup script.

## Operator station

The Project11 user interface `camp` can be used to track Drix even without a backseat driver by interfacing with the robobox. A backseat driver is needed to send and execute missions on DriX.

### Robobox

The robobox is iXblue's operator side interface box. It runs a ROS core which Project11 also uses on the operator side. The `robobox_interface` node of the `drix_project11` package rebroadcasts navigation data from iXblue proprietary message formats to ROS formats supports by `camp`.

### Operator laptop

One laptop used for Project11 operators is Snowpetrel. Its IP addresses are:

- 192.168.50.142/24 snowpetrel (Project11 operator network)
- 192.168.0.182/24 snowpetrelr (Robobox network)
- 192.168.8.182/24 snowpetreld (Drix network)

Scripts have been written to help operator laptops point to the proper ROS core. To help those scripts, a few environment variables are set in `.bashrc`.

`export DRIX_ROS_IP=192.168.8.182`
`export ROBOBOX_ROS_IP=192.168.0.182`
`export DRIX_NUMBER=8`

ROS uses the ROS_IP environment variable to tell other hosts how to reach a given host.

In this case, the `robobox_as_core.bash` script in the `drix_project11` package's `scripts` directory uses the ROBOBOX_ROS_IP environment variable to set the ROS_IP environment. The script also points us to the correct ROS core.

It's a good idea to have `.bashrc` source `robobox_as_core.bash` while conducting DriX ops so that new terminals have ROS automatically pointing to the robobox.

Use the `start_tmux_robobox_interface.sh` from the `drix_project11` package's `scripts` directory to start the robobox interface node in a tmux session.

