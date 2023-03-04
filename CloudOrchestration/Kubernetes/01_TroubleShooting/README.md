# Trouble Shootring

## Abstracts

* How to resolve troubles of Kubernetes

## The connection to the server XXX.XXX.XXX.XXX:6443 was refused - did you specify the right host or port?

You can use `crictl` commands to investigate issue.

````sh
$ sudo crictl pods
WARN[0000] runtime connect using default endpoints: [unix:///var/run/dockershim.sock unix:///run/containerd/containerd.sock unix:///run/crio/crio.sock unix:///var/run/cri-dockerd.sock]. As the default settings are now deprecated, you should set the endpoint instead. 
E0304 18:22:42.420134    3653 remote_runtime.go:277] "ListPodSandbox with filter from runtime service failed" err="rpc error: code = Unavailable desc = connection error: desc = \"transport: Error while dialing dial unix /var/run/dockershim.sock: connect: no such file or directory\"" filter="&PodSandboxFilter{Id:,State:nil,LabelSelector:map[string]string{},}"
FATA[0000] listing pod sandboxes: rpc error: code = Unavailable desc = connection error: desc = "transport: Error while dialing dial unix /var/run/dockershim.sock: connect: no such file or directory"
````

````sh
$ sudo kubelet --container-runtime-endpoint="unix:///run/containerd/containerd.sock"
I0304 18:37:42.350216    4856 server.go:412] "Kubelet version" kubeletVersion="v1.26.2"
I0304 18:37:42.350290    4856 server.go:414] "Golang settings" GOGC="" GOMAXPROCS="" GOTRACEBACK=""
I0304 18:37:42.350686    4856 server.go:575] "Standalone mode, no API client"
I0304 18:37:42.350881    4856 server.go:631] "Failed to get the kubelet's cgroup. Kubelet system container metrics may be missing." err="cpu and memory cgroup hierarchy not unified.  cpu: /user.slice, memory: /user.slice/user-1000.slice/session-2.scope"
I0304 18:37:42.358222    4856 server.go:463] "No api server defined - no events will be sent to API server"
I0304 18:37:42.358284    4856 server.go:659] "--cgroups-per-qos enabled, but --cgroup-root was not specified.  defaulting to /"
E0304 18:37:42.358852    4856 run.go:74] "command failed" err="failed to run Kubelet: running with swap on is not supported, please disable swap! or set --fail-swap-on flag to false. /proc/swaps contained: [Filename\t\t\t\tType\t\tSize\t\tUsed\t\tPriority /swapfile                               file\t\t2097148\t\t0\t\t-2]"
````

````sh
$ swapon
NAME      TYPE SIZE USED PRIO
/swapfile file   2G   0B   -2
````

`swapoff -a` is not permanetly. So you have to edit `/etc/fstab`.

````diff
# /etc/fstab: static file system information.
#
# Use 'blkid' to print the universally unique identifier for a
# device; this may be used with UUID= as a more robust way to name devices
# that works even if disks are added and removed. See fstab(5).
#
# <file system> <mount point>   <type>  <options>       <dump>  <pass>
# / was on /dev/sda5 during installation
UUID=13e9efee-6fa8-480c-a795-337e6672181a /               ext4    errors=remount-ro 0       1
# /boot/efi was on /dev/sda1 during installation
UUID=F30F-1DEB  /boot/efi       vfat    umask=0077      0       1
- /swapfile                                 none            swap    sw              0       0
+ #/swapfile                                 none            swap    sw              0       0
````

sudo sed -ri '/\sswap\s/s/^#?/#/' /etc/fstab

````sh
$ sudo docker info | grep -i cgroup
Cgroup Driver: cgroupfs
Cgroup Version: 1
````

Since Kubernetes v1.21 , kubeadm uses `systemd` as Cgroup Driver instead of `cgroupfs`.

systemctl stop docker

/etc/systemd/system/multi-user.target.wants/docker.service

````diff
- ExecStart=/usr/bin/dockerd -H fd:// --containerd=/run/containerd/containerd.sock
+ ExecStart=/usr/bin/dockerd -H fd:// --containerd=/run/containerd/containerd.sock --exec-opt native.cgroupdriver=systemd
````

systemctl daemon-reload
systemctl start docker
kubeadm init phase kubelet-start