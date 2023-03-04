# Kubernetes Dashboard

## Abstracts

* Visualize kubernetes cluster

## Dependencies

* [Kubernetes](https://github.com/kubernetes/kubernetes)
  * Apache-2.0 license
* [Kubernetes Dashboard](https://github.com/kubernetes/dashboard)
  * 2.7.0
  * Apache-2.0 license

## Compatibility

You should select proper Kubernetes for Dashboard.

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

### Create Account

````sh
$ kubectl create serviceaccount -n kubernetes-dashboard admin-user
$ cat <<EOF | kubectl apply -f -
apiVersion: rbac.authorization.k8s.io/v1
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
EOF
clusterrolebinding.rbac.authorization.k8s.io/admin-user created

$ kubectl -n kubernetes-dashboard create token admin-user
eyJhbGciOiJSUzI1NiIsImtpZCI6Ilc5cWRaS1VZbkpld1RjWm1mZF95QnhlbGFuNWMyTmo0dVV0UTFDdVpDRk0ifQ.eyJhdWQiOlsiaHR0cHM6Ly9rdWJlcm5ldGVzLmRlZmF1bHQuc3ZjLmNsdXN0ZXIubG9jYWwiXSwiZXhwIjoxNjc3OTYwNzcxLCJpYXQiOjE2Nzc5NTcxNzEsImlzcyI6Imh0dHBzOi8va3ViZXJuZXRlcy5kZWZhdWx0LnN2Yy5jbHVzdGVyLmxvY2FsIiwia3ViZXJuZXRlcy5pbyI6eyJuYW1lc3BhY2UiOiJrdWJlcm5ldGVzLWRhc2hib2FyZCIsInNlcnZpY2VhY2NvdW50Ijp7Im5hbWUiOiJhZG1pbi11c2VyIiwidWlkIjoiOTg5YjMwYmItOTI0Yy00NTg2LThlZTUtMzA5ODllOTUxN2FiIn19LCJuYmYiOjE2Nzc5NTcxNzEsInN1YiI6InN5c3RlbTpzZXJ2aWNlYWNjb3VudDprdWJlcm5ldGVzLWRhc2hib2FyZDphZG1pbi11c2VyIn0.kZ9MT9MFypKZKS1XHMPsWolGAN-7_m1M9HR05Ft3_uD258EWozgxae-4P2dmtdz7ouMWz0xW0y0jJ55Z1vojo2s8gR8fuDRoNE5vV2UhXlchNZRl7A7pNWNevY6s2uPrqjiXXIvRrDBBfAJCFXHvGk_Gn6ftQdLqbTVxYqqWKH-OcuBG6GMUSVHVoLngtLsqlPCRW2O7DwsmgFYzLm_wReBgWQ3Tx2mqiUTm_73Vse5umD6sswVLxTBI4Ok7ODMk4_wUyalM1waLXeE-kVLNkiP3YcDHZlQVd8xEtmSglGZLgo3tZTrRL64WCK09go19FQBNVb7-l_QpYgqydL4MSQ
````

### Start Dashboard

````sh
$ kubectl proxy
````

You can access `http://127.0.0.1:8001/api/v1/namespaces/kubernetes-dashboard/services/https:kubernetes-dashboard:/proxy/` and input previous token in login screen.

<img src="./images/login.png" />

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

You must check status of nodes and resolve why pods are not running.

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

### untolerated taint

You can see some pods us not running deu to FailedScheduling.

````sh
$ kubectl describe pod kubernetes-dashboard -n kubernetes-dashboard
...

Events:
  Type     Reason            Age                From               Message
  ----     ------            ----               ----               -------
  Warning  FailedScheduling  72s (x7 over 31m)  default-scheduler  0/1 nodes are available: 1 node(s) had untolerated taint {node-role.kubernetes.io/control-plane: }. preemption: 0/1 nodes are available: 1 Preemption is not helpful for scheduling..
````

It means that Pods will not be running on tainted node.
So system adminitrator can control node is tained or untainted.

You can see details of taints.

````sh
$ kubectl describe node
...

CreationTimestamp:  Sun, 05 Mar 2023 02:56:21 +0900
Taints:             node-role.kubernetes.io/control-plane:NoSchedule
Unschedulable:      false
````

And you can mark this node as untained by the following

````sh
$ kubectl taint nodes <node> node-role.kubernetes.io/control-plane:NoSchedule-
<node> untainted
````

After this, you can see taints values are disappeared.

````sh
$ kubectl describe node
...

CreationTimestamp:  Sun, 05 Mar 2023 02:56:21 +0900
Taints:             <none>
Unschedulable:      false
````

### Access from outer host

`kubernetes-dashboard` is ClusterIP by default.
So you can access to Dashboard from only host machine.

````sh
$ kubectl patch service kubernetes-dashboard -p '{"spec":{"type": "NodePort"}}' -n kubernetes-dashboard
service/kubernetes-dashboard patched
````