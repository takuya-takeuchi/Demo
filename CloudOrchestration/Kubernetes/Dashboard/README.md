# Octant

## Abstracts

* Visualize kubernetes cluster

## Dependencies

* [Kubernetes](https://github.com/kubernetes/kubernetes)
  * Apache-2.0 license
* [Kubernetes Dashboard](https://github.com/kubernetes/dashboard)
  * 2.7.0
  * Apache-2.0 license

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