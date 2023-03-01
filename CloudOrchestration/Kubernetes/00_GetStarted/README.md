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

````bat
$ cd InstallScripts/Ubuntu-20.04
$ ./install.sh <sudo password>
````