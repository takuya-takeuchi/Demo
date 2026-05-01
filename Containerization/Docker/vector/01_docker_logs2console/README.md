# from docker_logs to console

## Abstracts

* use `docker_logs` input to collect app's stdin logs via docker log driver
* use `remap` transform to remove unnecessary elements and screen out input containers

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
$ docker compose up -d && docker logs -f demo-vector-vector

2026-04-30T20:26:39.348137Z  INFO source{component_kind="source" component_id=sidercar_logs component_type=docker_logs}: vector::internal_events::docker_logs: Internal log [Started watching for container logs.] has been suppressed 1 times.
2026-04-30T20:26:39.348153Z  INFO source{component_kind="source" component_id=sidercar_logs component_type=docker_logs}: vector::internal_events::docker_logs: Started watching for container logs. container_id=e01c6b92f79bf7215283ad0873b9bb9b4556f2e668ba90b27f12fda4cc3f47ba
{"container_name":"demo-vector-app1","message":"[Info] Starting loop","timestamp":"2026-04-30T20:26:39.341921540Z"}
{"container_name":"demo-vector-app1","message":"[Info] Tick 1","timestamp":"2026-04-30T20:26:39.341958833Z"}
{"container_name":"demo-vector-app1","message":"[Info] Tick 2","timestamp":"2026-04-30T20:26:40.342114130Z"}
{"container_name":"demo-vector-app1","message":"[Info] Tick 3","timestamp":"2026-04-30T20:26:41.342341643Z"}
{"container_name":"demo-vector-app1","message":"[Info] Tick 4","timestamp":"2026-04-30T20:26:42.342483888Z"}
{"container_name":"demo-vector-app1","message":"[Info] Tick 5","timestamp":"2026-04-30T20:26:43.342790668Z"}
{"container_name":"demo-vector-app1","message":"[Info] Finished loop","timestamp":"2026-04-30T20:26:44.343093015Z"}
2026-04-30T20:26:44.395571Z  INFO source{component_kind="source" component_id=sidercar_logs component_type=docker_logs}: vector::internal_events::docker_logs: Internal log [Stopped watching for container logs.] has been suppressed 1 times.
2026-04-30T20:26:44.395589Z  INFO source{component_kind="source" component_id=sidercar_logs component_type=docker_logs}: vector::internal_events::docker_logs: Stopped watching for container logs. container_id=e01c6b92f79bf7215283ad0873b9bb9b4556f2e668ba90b27f12fda4cc3f47ba
2026-04-30T20:26:44.395653Z  INFO source{component_kind="source" component_id=sidercar_logs component_type=docker_logs}: vector::internal_events::docker_logs: Internal log [Started watching for container logs.] is being suppressed to avoid flooding.
2026-04-30T20:26:44.494482Z  INFO source{component_kind="source" component_id=sidercar_logs component_type=docker_logs}: vector::internal_events::docker_logs: Internal log [Stopped watching for container logs.] is being suppressed to avoid flooding.
{"container_name":"demo-vector-app1","message":"[Info] Finished loop","timestamp":"2026-04-30T20:26:44.343093015Z"}
````

The above logs are transformed by `remap` transformer.
Original `docker_logs` input produces following log.

````json
{
  "container_created_at": "2026-04-30T19:16:12.375822147Z",
  "container_id": "9af4f2e643d5e1769c9237a2c073a94930f1c245963660b2a2948fabf28417f4",
  "container_name": "demo-vector-app1",
  "host": "a14cd9709425",
  "image": "demo-vector-app",
  "label": {
    "com.docker.compose.config-hash": "1e61a8c985e3d1f3fe24fb689ae47d5950dbc5b565a950ae88513636a0f7a0e8",
    "com.docker.compose.container-number": "1",
    "com.docker.compose.depends_on": "",
    "com.docker.compose.image": "sha256:00d0690a0412f05355d0e4c3b7baffad155df27ebab7f8e577e5cca92f05843f",
    "com.docker.compose.oneoff": "False",
    "com.docker.compose.project": "vector",
    "com.docker.compose.project.config_files": "/data/work/oss/Demo/Containerization/Docker/vector/compose.yaml",
    "com.docker.compose.project.working_dir": "/data/work/oss/Demo/Containerization/Docker/vector",
    "com.docker.compose.service": "app1",
    "com.docker.compose.version": "2.29.7",
    "maintainer": "Takuya Takeuchi <takuya.takeuchi.dev@gmail.com>",
    "org.opencontainers.image.version": "24.04"
  },
  "message": "[Info] Finished loop",
  "source_type": "docker_logs",
  "stream": "stdout",
  "timestamp": "2026-04-30T20:12:52.953729796Z"
}
````

But `remap` transformer can delete elements you don't want.
[vector.yaml](./vector.yaml) contains the following section.

````yaml
transforms:
  clean_logs:
    type: remap
    inputs: ["sidercar_logs"]
    source: |
      del(.container_created_at)
      del(.container_id)
      del(.host)
      del(.image)
      del(.label)
      del(.source_type)
      del(.stream)
````

This section can delete `container_created_at`, `container_id`, `host`, `image`, `label`, `source_type` and `stream` elements from origin log.