# Kubernetes Get Started

## Abstracts

* Install scripts

## Dependencies

* [docker-ce](https://github.com/docker/docker-ce)
  * Apache-2.0 license
* [docker-ce-cli](https://github.com/docker/cli)
  * Apache-2.0 license
* [containerd.io](https://github.com/containerd/containerd)
  * Apache-2.0 license
* [Kubernetes](https://github.com/kubernetes/kubernetes)
  * Apache-2.0 license

## How to use?

Install user must be in sudoers.
And you can modify CIDR of pod network.

This statement is written in each scripts.

````sh
echo $1 | sudo kubeadm init --pod-network-cidr=172.24.0.0/16
````

### Ubuntu 20.04

````sh
$ cd InstallScripts/Ubuntu-20.04
$ ./install.sh <sudo password> <kubernetes version>
````

For examples,

````sh
$ cd InstallScripts/Ubuntu-20.04
$ ./install.sh password 1.25.7-00
$ cd InstallScripts/Ubuntu-20.04
$ ./install.sh <sudo password>

$ kubectl version --output=json
{
  "clientVersion": {
    "major": "1",
    "minor": "25",
    "gitVersion": "v1.25.7",
    "gitCommit": "723bcdb232300aaf5e147ff19b4df7ec8a20278d",
    "gitTreeState": "clean",
    "buildDate": "2023-02-22T14:05:25Z",
    "goVersion": "go1.19.6",
    "compiler": "gc",
    "platform": "linux/amd64"
  },
  "kustomizeVersion": "v4.5.7",
  "serverVersion": {
    "major": "1",
    "minor": "25",
    "gitVersion": "v1.25.7",
    "gitCommit": "723bcdb232300aaf5e147ff19b4df7ec8a20278d",
    "gitTreeState": "clean",
    "buildDate": "2023-02-22T13:58:23Z",
    "goVersion": "go1.19.6",
    "compiler": "gc",
    "platform": "linux/amd64"
  }
}
````