# Remote Launch

## Abstracts

* Kick remote ros nodes by using `roslaunch`

## Requirements

### Common

* ROS Noetic
* [tf2_sensor_msgs](https://index.ros.org/p/tf2_sensor_msgs/)
  * `sudo apt-get install ros-$ROS_DISTRO-tf2-sensor-msgs`

## Dependencies

* [ROS](https://github.com/ros/ros)
  * Noetic
  * BSD-3-Clause license

## How to run?

#### 1. Setuo remote side

1. Deploy [remote](./remote) directory to remote machine. It should be deployed to same location.
2. Modify [remote/setup.sh](./remote/setup.sh) on all remote machines. Fix location of `setup.sh`.
3. Run [remote/build.sh](./remote/build.sh) on all remote machines.
4. Modify `~/.bashrc`. Add the following contents. `ROS_MASTER_URI` is local machine to kick `roscore` and `roslaunch `.

````txt
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.11.114:11311
export ROS_IP=`hostname -I | cut -d' ' -f1`
````

#### 2. Setup local side

1. Get hostname of all remote machine.
2. Modify [local/start.launch](./local/start.launch).
  * Fix `machine.name`, `node.machine` and `machine.address` attributes.
  * Fix `machine.user` and `machine.env-loader` attributes. `machine.env-loader` attribute shall be exact location of [remote/setup.sh](./remote/setup.sh) on remote machine.
3. Modify `/etc/hosts`. Associate `machine.address` attribute with `machine.name` attribute.

For example,

````
192.168.11.115 ESXI-VM15
192.168.11.116 ESXI-VM16
````

This step is very important. Otherwisa, you will see the following error message when launch remote nodes.

````
  File "/usr/lib/python3.8/http/client.py", line 922, in connect
    self.sock = self._create_connection(
  File "/usr/lib/python3.8/socket.py", line 787, in create_connection
    for res in getaddrinfo(host, port, 0, SOCK_STREAM):
  File "/usr/lib/python3.8/socket.py", line 918, in getaddrinfo
    for res in _socket.getaddrinfo(host, port, family, type, proto, flags):
socket.gaierror: [Errno -3] Temporary failure in name resolution
````

4. Create ssh key. Type `ssh-keygen -t rsa -m PEM -b4096` on local machine. `-m PEM` option shall be specified. If you does not use `-m PEM` option, you will see `not a valid RSA private key file` error message when connecting to remote machine.
5. Copy public key to all remote machines. Type `ssh-copy-id <username>@<ip-address>` for all remote machines.
6. Modify `~/.bashrc`. Add the following contents.

````txt
source /opt/ros/noetic/setup.bash
export ROSLAUNCH_SSH_UNKNOWN=1
````

#### 3. Launch all remote nodes

Launch all remote nodes. You need not to launch `roscore`.

````bash
$ roslaunch start.launch
... logging to /home/xxxxx/.ros/log/30ff2b98-028b-11f0-92ee-e30ed6ddb65e/roslaunch-ESXI-VM14-5179.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://192.168.11.114:44449/
remote[192.168.11.115-0] starting roslaunch
remote[192.168.11.115-0]: creating ssh connection to 192.168.11.115:22, user[xxxxx]
launching remote roslaunch child with command: [env ROS_MASTER_URI=http://192.168.11.114:11311 /home/xxxxx/work/setup.bash roslaunch -c 192.168.11.115-0 -u http://192.168.11.114:44449/ --run_id 30ff2b98-028b-11f0-92ee-e30ed6ddb65e --sigint-timeout 15.0 --sigterm-timeout 2.0]
remote[192.168.11.115-0]: ssh connection created
remote[192.168.11.116-1] starting roslaunch
remote[192.168.11.116-1]: creating ssh connection to 192.168.11.116:22, user[xxxxx]
launching remote roslaunch child with command: [env ROS_MASTER_URI=http://192.168.11.114:11311 /home/xxxxx/work/setup.bash roslaunch -c 192.168.11.116-1 -u http://192.168.11.114:44449/ --run_id 30ff2b98-028b-11f0-92ee-e30ed6ddb65e --sigint-timeout 15.0 --sigterm-timeout 2.0]
remote[192.168.11.116-1]: ssh connection created

SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.17.0

MACHINES
 * ESXI-VM15
 * ESXI-VM16

NODES
  /
    hello1 (hello/hello)
    hello2 (hello/hello)

auto-starting new master
process[master]: started with pid [5203]
ROS_MASTER_URI=http://192.168.11.114:11311

setting /run_id to 30ff2b98-028b-11f0-92ee-e30ed6ddb65e
process[rosout-1]: started with pid [5218]
started core service [/rosout]
[192.168.11.115-0]: launching nodes...
[192.168.11.115-0]: ROS_MASTER_URI=http://192.168.11.114:11311
[192.168.11.115-0]: process[hello1-1]: started with pid [4323]
[192.168.11.115-0]: ... done launching nodes
[192.168.11.116-1]: launching nodes...
[192.168.11.116-1]: ROS_MASTER_URI=http://192.168.11.114:11311
[192.168.11.116-1]: process[hello2-1]: started with pid [4546]
[192.168.11.116-1]: ... done launching nodes
````

You can see that remote nodes are running by using `rosnode`.

````bash
$ rosnode list
/hello1
/hello2
/rosout
````