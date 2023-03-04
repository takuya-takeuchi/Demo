# Octant

## Abstracts

* Visualize kubernetes cluster

## Dependencies

* [Kubernetes](https://github.com/kubernetes/kubernetes)
  * Apache-2.0 license
* [Kubernetes Dashboard](https://github.com/kubernetes/dashboard)
  * 2.7.0
  * Apache-2.0 license

## Compatibility

|   |1.8|1.9|1.1|1.11|1.12|1.13|1.14|1.15|1.16|1.17|1.18|1.19|1.2|1.21|1.22|1.23|1.24|1.25|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|v2.7.0|-|-|-|-|-|-|-|-|-|-|-|-|-|-|?|?|?|✓|
|v2.6.1|-|-|-|-|-|-|-|-|-|-|-|-|-|?|?|?|✓|-|
|v2.6.0|-|-|-|-|-|-|-|-|-|-|-|-|-|?|?|?|✓|-|
|v2.5.1|-|-|-|-|-|-|-|-|-|-|-|-|?|?|?|✓|-|-|
|v2.5.0|-|-|-|-|-|-|-|-|-|-|-|-|?|?|?|✓|-|-|
|v2.4.0|-|-|-|-|-|-|-|-|-|-|?|?|✓|✓|-|-|-|-|
|v2.3.1|-|-|-|-|-|-|-|-|-|-|?|?|✓|✓|-|-|-|-|
|v2.3.0|-|-|-|-|-|-|-|-|-|-|?|?|✓|✓|-|-|-|-|
|v2.2.0|-|-|-|-|-|-|-|-|-|?|?|?|✓|-|-|-|-|-|
|v2.1.0|-|-|-|-|-|-|-|-|-|?|?|?|✓|-|-|-|-|-|
|v2.0.5|-|-|-|-|-|-|-|-|?|?|?|✓|-|-|-|-|-|-|
|v2.0.4|-|-|-|-|-|-|-|-|?|?|?|✓|-|-|-|-|-|-|
|v2.0.3|-|-|-|-|-|-|-|?|?|?|✓|-|-|-|-|-|-|-|
|v2.0.2|-|-|-|-|-|-|-|?|?|?|✓|-|-|-|-|-|-|-|
|v2.0.1|-|-|-|-|-|-|-|?|?|?|✓|-|-|-|-|-|-|-|
|v2.0.0|-|-|-|-|-|-|-|?|?|?|✓|-|-|-|-|-|-|-|
|v2.0.0-rc7|-|-|-|-|-|-|?|?|?|✓|-|-|-|-|-|-|-|-|
|v2.0.0-rc6|-|-|-|-|-|-|?|?|?|✓|-|-|-|-|-|-|-|-|
|v2.0.0-rc5|-|-|-|-|-|-|?|?|?|✓|-|-|-|-|-|-|-|-|
|v2.0.0-rc4|-|-|-|-|-|-|?|?|?|✓|-|-|-|-|-|-|-|-|
|v2.0.0-rc3|-|-|-|-|?|?|?|?|✓|-|-|-|-|-|-|-|-|-|
|v2.0.0-rc2|-|-|-|-|?|?|?|?|✓|-|-|-|-|-|-|-|-|-|
|v2.0.0-rc1|-|-|-|-|?|?|?|?|✓|-|-|-|-|-|-|-|-|-|
|v2.0.0-beta8|-|-|-|-|?|?|?|?|✓|-|-|-|-|-|-|-|-|-|-|
|v2.0.0-beta7|-|-|-|-|?|?|?|?|✓|-|-|-|-|-|-|-|-|-|-|
|v2.0.0-beta6|-|-|-|-|?|?|?|?|✓|-|-|-|-|-|-|-|-|-|-|
|v2.0.0-beta5|-|-|-|-|?|?|?|?|✓|-|-|-|-|-|-|-|-|-|-|
|v2.0.0-beta4|-|-|-|?|?|?|?|✓|-|-|-|-|-|-|-|-|-|-|-|
|v2.0.0-beta3|-|-|-|?|?|?|?|✓|-|-|-|-|-|-|-|-|-|-|-|
|v2.0.0-beta2|-|-|-|?|?|?|?|✓|-|-|-|-|-|-|-|-|-|-|-|
|v2.0.0-beta1|-|-|-|?|?|?|✓|?|-|-|-|-|-|-|-|-|-|-|-|
|v1.10.1|✓|✓|✓|?|?|✕|-|-|-|-|-|-|-|-|-|-|-|-|-|
|v1.10.0|✓|✓|✓|?|?|✕|-|-|-|-|-|-|-|-|-|-|-|-|-|
|v1.8.3|✓|✓|✕|✕|✕|✕|-|-|-|-|-|-|-|-|-|-|-|-|-|

* ✓ Fully supported version range.
* ? Due to breaking changes between Kubernetes API versions, some features might not work correctly in the Dashboard.
* ✕ Unsupported version range.

## How to install?

````sh
$ kubectl apply -f https://raw.githubusercontent.com/kubernetes/dashboard/v2.7.0/aio/deploy/recommended.yaml
````

## How to use?

You can only kick octant.

````sh
$ kubectl proxy
````

And then, you can access `http://127.0.0.1:8001/api/v1/namespaces/kube-system/services/https:kubernetes-dashboard:/proxy/`.

## Note

### return 503 error

You may see this response.

````json
{
  "kind": "Status",
  "apiVersion": "v1",
  "metadata": {},
  "status": "Failure",
  "message": "no endpoints available for service \"kubernetes-dashboard\"",
  "reason": "ServiceUnavailable",
  "code": 503
}
````

You must check status of nodes.

````sh
$ kubectl -n kubernetes-dashboard get all
NAME                                            READY   STATUS    RESTARTS   AGE
pod/dashboard-metrics-scraper-7bc864c59-zjrlw   0/1     Pending   0          33m
pod/kubernetes-dashboard-6c7ccbcf87-stbx6       0/1     Pending   0          33m

NAME                                TYPE        CLUSTER-IP       EXTERNAL-IP   PORT(S)    AGE
service/dashboard-metrics-scraper   ClusterIP   10.104.165.135   <none>        8000/TCP   33m
service/kubernetes-dashboard        ClusterIP   10.108.41.194    <none>        443/TCP    33m

NAME                                        READY   UP-TO-DATE   AVAILABLE   AGE
deployment.apps/dashboard-metrics-scraper   0/1     1            0           33m
deployment.apps/kubernetes-dashboard        0/1     1            0           33m

NAME                                                  DESIRED   CURRENT   READY   AGE
replicaset.apps/dashboard-metrics-scraper-7bc864c59   1         1         0       33m
replicaset.apps/kubernetes-dashboard-6c7ccbcf87       1         1         0       33m
````

````sh
$ kubectl describe pod kubernetes-dashboard -n kubernetes-dashboard
Name:             kubernetes-dashboard-6c7ccbcf87-stbx6
Namespace:        kubernetes-dashboard
Priority:         0
Service Account:  kubernetes-dashboard
Node:             <none>
Labels:           k8s-app=kubernetes-dashboard
                  pod-template-hash=6c7ccbcf87
Annotations:      <none>
Status:           Pending
IP:               
IPs:              <none>
Controlled By:    ReplicaSet/kubernetes-dashboard-6c7ccbcf87
Containers:
  kubernetes-dashboard:
    Image:      kubernetesui/dashboard:v2.7.0
    Port:       8443/TCP
    Host Port:  0/TCP
    Args:
      --auto-generate-certificates
      --namespace=kubernetes-dashboard
    Liveness:     http-get https://:8443/ delay=30s timeout=30s period=10s #success=1 #failure=3
    Environment:  <none>
    Mounts:
      /certs from kubernetes-dashboard-certs (rw)
      /tmp from tmp-volume (rw)
      /var/run/secrets/kubernetes.io/serviceaccount from kube-api-access-pjxwz (ro)
Conditions:
  Type           Status
  PodScheduled   False 
Volumes:
  kubernetes-dashboard-certs:
    Type:        Secret (a volume populated by a Secret)
    SecretName:  kubernetes-dashboard-certs
    Optional:    false
  tmp-volume:
    Type:       EmptyDir (a temporary directory that shares a pod's lifetime)
    Medium:     
    SizeLimit:  <unset>
  kube-api-access-pjxwz:
    Type:                    Projected (a volume that contains injected data from multiple sources)
    TokenExpirationSeconds:  3607
    ConfigMapName:           kube-root-ca.crt
    ConfigMapOptional:       <nil>
    DownwardAPI:             true
QoS Class:                   BestEffort
Node-Selectors:              kubernetes.io/os=linux
Tolerations:                 node-role.kubernetes.io/master:NoSchedule
                             node.kubernetes.io/not-ready:NoExecute op=Exists for 300s
                             node.kubernetes.io/unreachable:NoExecute op=Exists for 300s
Events:
  Type     Reason            Age                From               Message
  ----     ------            ----               ----               -------
  Warning  FailedScheduling  72s (x7 over 31m)  default-scheduler  0/1 nodes are available: 1 node(s) had untolerated taint {node-role.kubernetes.io/control-plane: }. preemption: 0/1 nodes are available: 1 Preemption is not helpful for scheduling..
````

### container runtime network not ready: NetworkReady=false reason:NetworkPluginNotReady message:Network plugin returns error: cni plugin not initialized


````
$ kubectl describe node esxi-vm03
Name:               esxi-vm03
Roles:              control-plane
Labels:             beta.kubernetes.io/arch=amd64
                    beta.kubernetes.io/os=linux
                    kubernetes.io/arch=amd64
                    kubernetes.io/hostname=esxi-vm03
                    kubernetes.io/os=linux
                    node-role.kubernetes.io/control-plane=
                    node.kubernetes.io/exclude-from-external-load-balancers=
Annotations:        kubeadm.alpha.kubernetes.io/cri-socket: unix:///run/containerd/containerd.sock
                    node.alpha.kubernetes.io/ttl: 0
                    volumes.kubernetes.io/controller-managed-attach-detach: true
CreationTimestamp:  Sat, 04 Mar 2023 23:50:16 +0900
Taints:             node.kubernetes.io/not-ready:NoSchedule
Unschedulable:      false
Lease:
  HolderIdentity:  esxi-vm03
  AcquireTime:     <unset>
  RenewTime:       Sun, 05 Mar 2023 00:27:13 +0900
Conditions:
  Type             Status  LastHeartbeatTime                 LastTransitionTime                Reason                       Message
  ----             ------  -----------------                 ------------------                ------                       -------
  MemoryPressure   False   Sun, 05 Mar 2023 00:22:25 +0900   Sat, 04 Mar 2023 23:50:15 +0900   KubeletHasSufficientMemory   kubelet has sufficient memory available
  DiskPressure     False   Sun, 05 Mar 2023 00:22:25 +0900   Sat, 04 Mar 2023 23:50:15 +0900   KubeletHasNoDiskPressure     kubelet has no disk pressure
  PIDPressure      False   Sun, 05 Mar 2023 00:22:25 +0900   Sat, 04 Mar 2023 23:50:15 +0900   KubeletHasSufficientPID      kubelet has sufficient PID available
  Ready            False   Sun, 05 Mar 2023 00:22:25 +0900   Sat, 04 Mar 2023 23:50:15 +0900   KubeletNotReady              container runtime network not ready: NetworkReady=false reason:NetworkPluginNotReady message:Network plugin returns error: cni plugin not initialized
````

You have to install cni.

````sh
// デフォルトだとamd64向けになっているので、ラズパイが使っているarmに変更する
curl https://raw.githubusercontent.com/coreos/flannel/master/Documentation/kube-flannel.yml \
| sed "s/amd64/arm/g" | sed "s/vxlan/host-gw/g" > kube-flannel.yml

$ kubectl apply -f kube-flannel.yml
````

### taint

check `Taints`

````sh
 kubectl describe node
Name:               esxi-vm03
Roles:              control-plane
Labels:             beta.kubernetes.io/arch=amd64
                    beta.kubernetes.io/os=linux
                    kubernetes.io/arch=amd64
                    kubernetes.io/hostname=esxi-vm03
                    kubernetes.io/os=linux
                    node-role.kubernetes.io/control-plane=
                    node.kubernetes.io/exclude-from-external-load-balancers=
Annotations:        kubeadm.alpha.kubernetes.io/cri-socket: unix:///run/containerd/containerd.sock
                    node.alpha.kubernetes.io/ttl: 0
                    volumes.kubernetes.io/controller-managed-attach-detach: true
CreationTimestamp:  Sat, 04 Mar 2023 23:50:16 +0900
Taints:             node-role.kubernetes.io/control-plane:NoSchedule
                    node.kubernetes.io/not-ready:NoSchedule
Unschedulable:      false
Lease:
  HolderIdentity:  esxi-vm03
  AcquireTime:     <unset>
  RenewTime:       Sun, 05 Mar 2023 00:21:37 +0900
Conditions:
  Type             Status  LastHeartbeatTime                 LastTransitionTime                Reason                       Message
  ----             ------  -----------------                 ------------------                ------                       -------
  MemoryPressure   False   Sun, 05 Mar 2023 00:17:19 +0900   Sat, 04 Mar 2023 23:50:15 +0900   KubeletHasSufficientMemory   kubelet has sufficient memory available
  DiskPressure     False   Sun, 05 Mar 2023 00:17:19 +0900   Sat, 04 Mar 2023 23:50:15 +0900   KubeletHasNoDiskPressure     kubelet has no disk pressure
  PIDPressure      False   Sun, 05 Mar 2023 00:17:19 +0900   Sat, 04 Mar 2023 23:50:15 +0900   KubeletHasSufficientPID      kubelet has sufficient PID available
  Ready            False   Sun, 05 Mar 2023 00:17:19 +0900   Sat, 04 Mar 2023 23:50:15 +0900   KubeletNotReady              container runtime network not ready: NetworkReady=false reason:NetworkPluginNotReady message:Network plugin returns error: cni plugin not initialized
Addresses:
  InternalIP:  192.168.11.103
  Hostname:    esxi-vm03
Capacity:
  cpu:                4
  ephemeral-storage:  65221196Ki
  hugepages-1Gi:      0
  hugepages-2Mi:      0
  memory:             24618524Ki
  pods:               110
Allocatable:
  cpu:                4
  ephemeral-storage:  60107854135
  hugepages-1Gi:      0
  hugepages-2Mi:      0
  memory:             24516124Ki
  pods:               110
System Info:
  Machine ID:                 de79524be7074ae99d1953bf961e5e05
  System UUID:                d4f84d56-5681-bd55-b6a2-c78aae3188a3
  Boot ID:                    6df1f99c-938a-4673-864a-a687e4a22dec
  Kernel Version:             5.15.0-67-generic
  OS Image:                   Ubuntu 20.04.5 LTS
  Operating System:           linux
  Architecture:               amd64
  Container Runtime Version:  containerd://1.6.18
  Kubelet Version:            v1.25.7
  Kube-Proxy Version:         v1.25.7
PodCIDR:                      172.24.0.0/24
PodCIDRs:                     172.24.0.0/24
Non-terminated Pods:          (5 in total)
  Namespace                   Name                                 CPU Requests  CPU Limits  Memory Requests  Memory Limits  Age
  ---------                   ----                                 ------------  ----------  ---------------  -------------  ---
  kube-system                 etcd-esxi-vm03                       100m (2%)     0 (0%)      100Mi (0%)       0 (0%)         31m
  kube-system                 kube-apiserver-esxi-vm03             250m (6%)     0 (0%)      0 (0%)           0 (0%)         31m
  kube-system                 kube-controller-manager-esxi-vm03    200m (5%)     0 (0%)      0 (0%)           0 (0%)         31m
  kube-system                 kube-proxy-cz2qk                     0 (0%)        0 (0%)      0 (0%)           0 (0%)         31m
  kube-system                 kube-scheduler-esxi-vm03             100m (2%)     0 (0%)      0 (0%)           0 (0%)         31m
Allocated resources:
  (Total limits may be over 100 percent, i.e., overcommitted.)
  Resource           Requests    Limits
  --------           --------    ------
  cpu                650m (16%)  0 (0%)
  memory             100Mi (0%)  0 (0%)
  ephemeral-storage  0 (0%)      0 (0%)
  hugepages-1Gi      0 (0%)      0 (0%)
  hugepages-2Mi      0 (0%)      0 (0%)
Events:
  Type     Reason                   Age                From             Message
  ----     ------                   ----               ----             -------
  Normal   Starting                 31m                kube-proxy       
  Normal   Starting                 19m                kube-proxy       
  Normal   NodeHasSufficientMemory  31m                kubelet          Node esxi-vm03 status is now: NodeHasSufficientMemory
  Warning  InvalidDiskCapacity      31m                kubelet          invalid capacity 0 on image filesystem
  Normal   NodeHasNoDiskPressure    31m                kubelet          Node esxi-vm03 status is now: NodeHasNoDiskPressure
  Normal   NodeHasSufficientPID     31m                kubelet          Node esxi-vm03 status is now: NodeHasSufficientPID
  Normal   NodeAllocatableEnforced  31m                kubelet          Updated Node Allocatable limit across pods
  Normal   Starting                 31m                kubelet          Starting kubelet.
  Normal   RegisteredNode           31m                node-controller  Node esxi-vm03 event: Registered Node esxi-vm03 in Controller
  Normal   Starting                 20m                kubelet          Starting kubelet.
  Warning  InvalidDiskCapacity      20m                kubelet          invalid capacity 0 on image filesystem
  Normal   NodeAllocatableEnforced  20m                kubelet          Updated Node Allocatable limit across pods
  Normal   NodeHasSufficientMemory  20m (x8 over 20m)  kubelet          Node esxi-vm03 status is now: NodeHasSufficientMemory
  Normal   NodeHasNoDiskPressure    20m (x7 over 20m)  kubelet          Node esxi-vm03 status is now: NodeHasNoDiskPressure
  Normal   NodeHasSufficientPID     20m (x7 over 20m)  kubelet          Node esxi-vm03 status is now: NodeHasSufficientPID
  Normal   RegisteredNode           19m                node-controller  Node esxi-vm03 event: Registered Node esxi-vm03 in Controller
````

````sh
$ kubectl describe pod -n kubernetes-dashboard
Name:             dashboard-metrics-scraper-64bcc67c9c-5mttm
Namespace:        kubernetes-dashboard
Priority:         0
Service Account:  kubernetes-dashboard
Node:             <none>
Labels:           k8s-app=dashboard-metrics-scraper
                  pod-template-hash=64bcc67c9c
Annotations:      <none>
Status:           Pending
IP:               
IPs:              <none>
Controlled By:    ReplicaSet/dashboard-metrics-scraper-64bcc67c9c
Containers:
  dashboard-metrics-scraper:
    Image:        kubernetesui/metrics-scraper:v1.0.8
    Port:         8000/TCP
    Host Port:    0/TCP
    Liveness:     http-get http://:8000/ delay=30s timeout=30s period=10s #success=1 #failure=3
    Environment:  <none>
    Mounts:
      /tmp from tmp-volume (rw)
      /var/run/secrets/kubernetes.io/serviceaccount from kube-api-access-f4r5l (ro)
Conditions:
  Type           Status
  PodScheduled   False 
Volumes:
  tmp-volume:
    Type:       EmptyDir (a temporary directory that shares a pod's lifetime)
    Medium:     
    SizeLimit:  <unset>
  kube-api-access-f4r5l:
    Type:                    Projected (a volume that contains injected data from multiple sources)
    TokenExpirationSeconds:  3607
    ConfigMapName:           kube-root-ca.crt
    ConfigMapOptional:       <nil>
    DownwardAPI:             true
QoS Class:                   BestEffort
Node-Selectors:              kubernetes.io/os=linux
Tolerations:                 node-role.kubernetes.io/master:NoSchedule
                             node.kubernetes.io/not-ready:NoExecute op=Exists for 300s
                             node.kubernetes.io/unreachable:NoExecute op=Exists for 300s
Events:
  Type     Reason            Age    From               Message
  ----     ------            ----   ----               -------
  Warning  FailedScheduling  3m53s  default-scheduler  0/1 nodes are available: 1 node(s) had untolerated taint {node-role.kubernetes.io/control-plane: }. preemption: 0/1 nodes are available: 1 Preemption is not helpful for scheduling.


Name:             kubernetes-dashboard-5c8bd6b59-4xvgs
Namespace:        kubernetes-dashboard
Priority:         0
Service Account:  kubernetes-dashboard
Node:             <none>
Labels:           k8s-app=kubernetes-dashboard
                  pod-template-hash=5c8bd6b59
Annotations:      <none>
Status:           Pending
IP:               
IPs:              <none>
Controlled By:    ReplicaSet/kubernetes-dashboard-5c8bd6b59
Containers:
  kubernetes-dashboard:
    Image:      kubernetesui/dashboard:v2.7.0
    Port:       8443/TCP
    Host Port:  0/TCP
    Args:
      --auto-generate-certificates
      --namespace=kubernetes-dashboard
    Liveness:     http-get https://:8443/ delay=30s timeout=30s period=10s #success=1 #failure=3
    Environment:  <none>
    Mounts:
      /certs from kubernetes-dashboard-certs (rw)
      /tmp from tmp-volume (rw)
      /var/run/secrets/kubernetes.io/serviceaccount from kube-api-access-zcq26 (ro)
Conditions:
  Type           Status
  PodScheduled   False 
Volumes:
  kubernetes-dashboard-certs:
    Type:        Secret (a volume populated by a Secret)
    SecretName:  kubernetes-dashboard-certs
    Optional:    false
  tmp-volume:
    Type:       EmptyDir (a temporary directory that shares a pod's lifetime)
    Medium:     
    SizeLimit:  <unset>
  kube-api-access-zcq26:
    Type:                    Projected (a volume that contains injected data from multiple sources)
    TokenExpirationSeconds:  3607
    ConfigMapName:           kube-root-ca.crt
    ConfigMapOptional:       <nil>
    DownwardAPI:             true
QoS Class:                   BestEffort
Node-Selectors:              kubernetes.io/os=linux
Tolerations:                 node-role.kubernetes.io/master:NoSchedule
                             node.kubernetes.io/not-ready:NoExecute op=Exists for 300s
                             node.kubernetes.io/unreachable:NoExecute op=Exists for 300s
Events:
  Type     Reason            Age    From               Message
  ----     ------            ----   ----               -------
  Warning  FailedScheduling  3m54s  default-scheduler  0/1 nodes are available: 1 node(s) had untolerated taint {node-role.kubernetes.io/control-plane: }. preemption: 0/1 nodes are available: 1 Preemption is not helpful for scheduling.
````

````sh
$ kubectl taint nodes esxi-vm03 node-role.kubernetes.io/control-plane:NoSchedule-
node/esxi-vm03 untainted
$ kubectl taint nodes esxi-vm03 node.kubernetes.io/not-ready:NoSchedule-
node/esxi-vm03 untainted
````

````sh
 kubectl describe pod -n kubernetes-dashboard
Name:             dashboard-metrics-scraper-64bcc67c9c-5mttm
Namespace:        kubernetes-dashboard
Priority:         0
Service Account:  kubernetes-dashboard
Node:             esxi-vm03/192.168.11.103
Start Time:       Sun, 05 Mar 2023 00:24:32 +0900
Labels:           k8s-app=dashboard-metrics-scraper
                  pod-template-hash=64bcc67c9c
Annotations:      <none>
Status:           Pending
IP:               
IPs:              <none>
Controlled By:    ReplicaSet/dashboard-metrics-scraper-64bcc67c9c
Containers:
  dashboard-metrics-scraper:
    Container ID:   
    Image:          kubernetesui/metrics-scraper:v1.0.8
    Image ID:       
    Port:           8000/TCP
    Host Port:      0/TCP
    State:          Waiting
      Reason:       ContainerCreating
    Ready:          False
    Restart Count:  0
    Liveness:       http-get http://:8000/ delay=30s timeout=30s period=10s #success=1 #failure=3
    Environment:    <none>
    Mounts:
      /tmp from tmp-volume (rw)
      /var/run/secrets/kubernetes.io/serviceaccount from kube-api-access-f4r5l (ro)
Conditions:
  Type              Status
  Initialized       True 
  Ready             False 
  ContainersReady   False 
  PodScheduled      True 
Volumes:
  tmp-volume:
    Type:       EmptyDir (a temporary directory that shares a pod's lifetime)
    Medium:     
    SizeLimit:  <unset>
  kube-api-access-f4r5l:
    Type:                    Projected (a volume that contains injected data from multiple sources)
    TokenExpirationSeconds:  3607
    ConfigMapName:           kube-root-ca.crt
    ConfigMapOptional:       <nil>
    DownwardAPI:             true
QoS Class:                   BestEffort
Node-Selectors:              kubernetes.io/os=linux
Tolerations:                 node-role.kubernetes.io/master:NoSchedule
                             node.kubernetes.io/not-ready:NoExecute op=Exists for 300s
                             node.kubernetes.io/unreachable:NoExecute op=Exists for 300s
Events:
  Type     Reason                  Age                    From               Message
  ----     ------                  ----                   ----               -------
  Warning  FailedScheduling        47m (x2 over 52m)      default-scheduler  0/1 nodes are available: 1 node(s) had untolerated taint {node-role.kubernetes.io/control-plane: }. preemption: 0/1 nodes are available: 1 Preemption is not helpful for scheduling.
  Normal   Scheduled               43m                    default-scheduler  Successfully assigned kubernetes-dashboard/dashboard-metrics-scraper-64bcc67c9c-5mttm to esxi-vm03
  Warning  FailedMount             42m (x7 over 43m)      kubelet            MountVolume.SetUp failed for volume "kube-api-access-f4r5l" : object "kubernetes-dashboard"/"kube-root-ca.crt" not registered
  Warning  NetworkNotReady         13m (x902 over 43m)    kubelet            network is not ready: container runtime network not ready: NetworkReady=false reason:NetworkPluginNotReady message:Network plugin returns error: cni plugin not initialized
  Warning  FailedCreatePodSandBox  8m2s                   kubelet            Failed to create pod sandbox: rpc error: code = Unknown desc = failed to setup network for sandbox "01cffd2c2824d3cd246d6399eeca4178eca55c068678264bf13c238a05fd3fd2": plugin type="flannel" failed (add): loadFlannelSubnetEnv failed: open /run/flannel/subnet.env: no such file or directory
  Warning  FailedCreatePodSandBox  3m1s (x16 over 6m29s)  kubelet            (combined from similar events): Failed to create pod sandbox: rpc error: code = Unknown desc = failed to setup network for sandbox "2635fce6066ae37d115c1bf53213f793b5247d8d12936307fb445cc8ee517ffe": plugin type="flannel" failed (add): loadFlannelSubnetEnv failed: open /run/flannel/subnet.env: no such file or directory


Name:             kubernetes-dashboard-5c8bd6b59-4xvgs
Namespace:        kubernetes-dashboard
Priority:         0
Service Account:  kubernetes-dashboard
Node:             esxi-vm03/192.168.11.103
Start Time:       Sun, 05 Mar 2023 00:24:32 +0900
Labels:           k8s-app=kubernetes-dashboard
                  pod-template-hash=5c8bd6b59
Annotations:      <none>
Status:           Pending
IP:               
IPs:              <none>
Controlled By:    ReplicaSet/kubernetes-dashboard-5c8bd6b59
Containers:
  kubernetes-dashboard:
    Container ID:  
    Image:         kubernetesui/dashboard:v2.7.0
    Image ID:      
    Port:          8443/TCP
    Host Port:     0/TCP
    Args:
      --auto-generate-certificates
      --namespace=kubernetes-dashboard
    State:          Waiting
      Reason:       ContainerCreating
    Ready:          False
    Restart Count:  0
    Liveness:       http-get https://:8443/ delay=30s timeout=30s period=10s #success=1 #failure=3
    Environment:    <none>
    Mounts:
      /certs from kubernetes-dashboard-certs (rw)
      /tmp from tmp-volume (rw)
      /var/run/secrets/kubernetes.io/serviceaccount from kube-api-access-zcq26 (ro)
Conditions:
  Type              Status
  Initialized       True 
  Ready             False 
  ContainersReady   False 
  PodScheduled      True 
Volumes:
  kubernetes-dashboard-certs:
    Type:        Secret (a volume populated by a Secret)
    SecretName:  kubernetes-dashboard-certs
    Optional:    false
  tmp-volume:
    Type:       EmptyDir (a temporary directory that shares a pod's lifetime)
    Medium:     
    SizeLimit:  <unset>
  kube-api-access-zcq26:
    Type:                    Projected (a volume that contains injected data from multiple sources)
    TokenExpirationSeconds:  3607
    ConfigMapName:           kube-root-ca.crt
    ConfigMapOptional:       <nil>
    DownwardAPI:             true
QoS Class:                   BestEffort
Node-Selectors:              kubernetes.io/os=linux
Tolerations:                 node-role.kubernetes.io/master:NoSchedule
                             node.kubernetes.io/not-ready:NoExecute op=Exists for 300s
                             node.kubernetes.io/unreachable:NoExecute op=Exists for 300s
Events:
  Type     Reason                  Age                     From               Message
  ----     ------                  ----                    ----               -------
  Warning  FailedScheduling        47m (x2 over 52m)       default-scheduler  0/1 nodes are available: 1 node(s) had untolerated taint {node-role.kubernetes.io/control-plane: }. preemption: 0/1 nodes are available: 1 Preemption is not helpful for scheduling.
  Normal   Scheduled               43m                     default-scheduler  Successfully assigned kubernetes-dashboard/kubernetes-dashboard-5c8bd6b59-4xvgs to esxi-vm03
  Warning  FailedMount             42m (x6 over 43m)       kubelet            MountVolume.SetUp failed for volume "kubernetes-dashboard-certs" : object "kubernetes-dashboard"/"kubernetes-dashboard-certs" not registered
  Warning  FailedMount             42m (x6 over 43m)       kubelet            MountVolume.SetUp failed for volume "kube-api-access-zcq26" : object "kubernetes-dashboard"/"kube-root-ca.crt" not registered
  Warning  NetworkNotReady         13m (x902 over 43m)     kubelet            network is not ready: container runtime network not ready: NetworkReady=false reason:NetworkPluginNotReady message:Network plugin returns error: cni plugin not initialized
  Warning  FailedCreatePodSandBox  7m59s                   kubelet            Failed to create pod sandbox: rpc error: code = Unknown desc = failed to setup network for sandbox "3ff9a88066ee5211dddad6e8a54708b3ce9aedfa5e52d15e5cc96962e7a8309b": plugin type="flannel" failed (add): loadFlannelSubnetEnv failed: open /run/flannel/subnet.env: no such file or directory
  Warning  FailedCreatePodSandBox  2m52s (x18 over 6m30s)  kubelet            (combined from similar events): Failed to create pod sandbox: rpc error: code = Unknown desc = failed to setup network for sandbox "b02d75efb18bcc76f4302f71e699254255cb4d29d8dea9bf03270cdd40561076": plugin type="flannel" failed (add): loadFlannelSubnetEnv failed: open /run/flannel/subnet.env: no such file or directory
````

create subnet.env

````txt
$ cat /run/flannel/subnet.env
FLANNEL_NETWORK=172.24.0.0/16
FLANNEL_SUBNET=172.24.4.1/24
FLANNEL_MTU=1450
FLANNEL_IPMASQ=false
````

### Login token

````yaml
apiVersion: v1
kind: ServiceAccount
metadata:
  name: admin-user
  namespace: kubernetes-dashboard
---
apiVersion: rbac.authorization.k8s.io/v1beta1
kind: ClusterRoleBinding
metadata:
  name: admin-user
roleRef:
  apiGroup: rbac.authorization.k8s.io
  kind: ClusterRole
  name: cluster-admin
subjects:
- kind: ServiceAccount
  name: admin-user
  namespace: kubernetes-dashboard
````