#!/bin/bash +x

# This script is confirmed on 2023/03/01

if [ $# -ne 2 ]; then
  echo "<Pass password of running user to execute command as root> <kubernetes version to be installed>" 1>&2
  exit 1
fi

echo $1 | sudo apt update
echo $1 | sudo apt install -y \
               apt-transport-https \
               ca-certificates \
               curl \
               gnupg-agent \
               software-properties-common

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
echo $1 | sudo apt-key fingerprint 0EBFCD88

echo $1 | sudo add-apt-repository \
               "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
               $(lsb_release -cs) \
               stable"

echo $1 | sudo apt update
echo $1 | sudo apt install -y docker-ce docker-ce-cli

echo $1 | sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

echo $1 | sudo apt install containerd.io
echo $1 | sudo systemctl restart containerd

echo $1 | sudo usermod -aG docker $USER

curl -s https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
echo $1 | sudo apt-add-repository "deb http://apt.kubernetes.io/ kubernetes-xenial main"

echo $1 | sudo apt update
echo $1 | sudo apt install -y kubeadm=$2 kubelet=$2 kubectl=$2

echo $1 | sudo sed -ri '/\sswap\s/s/^#?/#/' /etc/fstab
echo $1 | sudo swapoff -a

# https://github.com/containerd/containerd/issues/4581
echo $1 | sudo kubeadm init --pod-network-cidr=172.24.0.0/16 --cri-socket=/run/containerd/containerd.sock
# try again
echo $1 | sudo rm /etc/containerd/config.toml
echo $1 | sudo systemctl restart containerd
echo $1 | sudo kubeadm init --pod-network-cidr=172.24.0.0/16 --cri-socket=/run/containerd/containerd.sock

mkdir -p $HOME/.kube
echo $1 | sudo cp -i /etc/kubernetes/admin.conf $HOME/.kube/config
echo $1 | sudo chown $(id -u):$(id -g) $HOME/.kube/config