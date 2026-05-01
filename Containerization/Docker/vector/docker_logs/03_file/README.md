# from docker_logs to file

## Abstracts

* use `docker_logs` input to collect app's stdin logs via docker log driver
* use `remap` transform to remove unnecessary elements and screen out input containers
* use `file` sink to write logs to storage

## Dependencies

* [vector](https://github.com/vectordotdev/vector)
  * v0.55.0
  * Mozilla Public License 2.0

## How to use?

At first, build app image.

````bash
$ pwsh build.ps1
````

Then, lanuch container and show logs.

````bash
$ docker compose up -d
````

Then, you can see logs in [logs](./logs)

````
{"container_name":"demo-vector-app1","message":"[Info] Starting loop","timestamp":"2026-05-01T07:50:07.280936447Z"}
{"container_name":"demo-vector-app1","message":"[Info] Tick 1","timestamp":"2026-05-01T07:50:07.281209682Z"}
{"container_name":"demo-vector-app1","message":"[Info] Tick 2","timestamp":"2026-05-01T07:50:08.281101766Z"}
{"container_name":"demo-vector-app1","message":"[Info] Tick 3","timestamp":"2026-05-01T07:50:09.281473013Z"}
{"container_name":"demo-vector-app1","message":"[Info] Tick 4","timestamp":"2026-05-01T07:50:10.281552780Z"}
{"container_name":"demo-vector-app1","message":"[Info] Tick 5","timestamp":"2026-05-01T07:50:11.281698658Z"}
{"container_name":"demo-vector-app1","message":"[Info] Finished loop","timestamp":"2026-05-01T07:50:12.287411974Z"}
{"container_name":"demo-vector-app1","message":"[Info] Finished loop","timestamp":"2026-05-01T07:50:12.287411974Z"}
````