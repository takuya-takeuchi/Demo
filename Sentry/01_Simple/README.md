# Tasting Sentry on Windows

## Abstacts

* Try Sentry on Windows
  * Use via C#

## Requirements

* Visual Studio 2022
* .NET 6.0
* Windows Subsystem for Linux 2
  * You can check whether Ubuntu works on WSL 2 or not by `wsl -l -v`.
* Docker desktop
  * Docker 19.03.6+
  * Compose 1.28.0+

## Dependencies

* [Sentry](https://github.com/getsentry/self-hosted)
  * Business Source License 1.1

## Install

### 1. Run install.sh on Linux shell

You can install without `--skip-user-prompt` but this instruction demonstarates author's result.

````sh
$ ./install.sh --skip-user-prompt
▶ Parsing command line ...

▶ Initializing Docker Compose ...

▶ Setting up error handling ...

▶ Checking for latest commit ...
skipped

▶ Checking minimum requirements ...
Found Docker version 20.10.14
Found Docker Compose version 2.4.1
Unable to find image 'busybox:latest' locally
latest: Pulling from library/busybox
50e8d59317eb: Pulling fs layer
50e8d59317eb: Verifying Checksum
50e8d59317eb: Download complete
50e8d59317eb: Pull complete
Digest: sha256:d2b53584f580310186df7a2055ce3ff83cc0df6caacf1e3489bff8cf5d0af5d8
Status: Downloaded newer image for busybox:latest

▶ Turning things off ...
sentry-self-hosted  Warning: No resource found to remove
sentry_onpremise  Warning: No resource found to remove

▶ Creating volumes for persistent storage ...
Created sentry-clickhouse.
Created sentry-data.
Created sentry-kafka.
Created sentry-postgres.
Created sentry-redis.
Created sentry-symbolicator.
Created sentry-zookeeper.

▶ Ensuring files from examples ...
Creating ../sentry/sentry.conf.py...
Creating ../sentry/config.yml...
Creating ../symbolicator/config.yml...
Creating ../sentry/requirements.txt...

▶ Ensuring Relay credentials ...
Creating ../relay/config.yml...
relay Pulling
c32ce6654453 Pulling fs layer
f43a1064aeac Pulling fs layer
d34b02acf618 Pulling fs layer
13e8a56469ec Pulling fs layer
38dee6f9a35a Pulling fs layer
4270e2b619a4 Pulling fs layer
aeab01cd6e68 Pulling fs layer
13e8a56469ec Waiting
4270e2b619a4 Waiting
38dee6f9a35a Waiting
f43a1064aeac Downloading [>                                                  ]  51.93kB/5.132MB
c32ce6654453 Downloading [>                                                  ]  272.8kB/27.14MB
d34b02acf618 Downloading [=======>                                           ]     658B/4.598kB
d34b02acf618 Downloading [==================================================>]  4.598kB/4.598kB
d34b02acf618 Verifying Checksum
d34b02acf618 Download complete
f43a1064aeac Downloading [=====>                                             ]  535.8kB/5.132MB
c32ce6654453 Downloading [==>                                                ]  1.395MB/27.14MB
f43a1064aeac Downloading [==============>                                    ]  1.453MB/5.132MB
c32ce6654453 Downloading [=====>                                             ]  3.062MB/27.14MB
f43a1064aeac Downloading [====================>                              ]    2.1MB/5.132MB
c32ce6654453 Downloading [========>                                          ]  4.455MB/27.14MB
f43a1064aeac Downloading [============================>                      ]  2.882MB/5.132MB
c32ce6654453 Downloading [==========>                                        ]  5.847MB/27.14MB
f43a1064aeac Downloading [===================================>               ]  3.603MB/5.132MB
c32ce6654453 Downloading [=============>                                     ]  7.248MB/27.14MB
f43a1064aeac Downloading [==========================================>        ]  4.324MB/5.132MB
f43a1064aeac Verifying Checksum
c32ce6654453 Downloading [===============>                                   ]  8.653MB/27.14MB
c32ce6654453 Downloading [====================>                              ]  10.89MB/27.14MB
c32ce6654453 Downloading [=========================>                         ]  13.69MB/27.14MB
c32ce6654453 Downloading [=============================>                     ]  16.21MB/27.14MB
c32ce6654453 Downloading [=================================>                 ]  18.44MB/27.14MB
c32ce6654453 Downloading [=======================================>           ]  21.24MB/27.14MB
c32ce6654453 Downloading [===========================================>       ]  23.47MB/27.14MB
c32ce6654453 Downloading [==============================================>    ]  25.41MB/27.14MB
c32ce6654453 Verifying Checksum
c32ce6654453 Download complete
c32ce6654453 Extracting [>                                                  ]  294.9kB/27.14MB
c32ce6654453 Extracting [=====>                                             ]  3.244MB/27.14MB
c32ce6654453 Extracting [=============>                                     ]  7.078MB/27.14MB
c32ce6654453 Extracting [==================>                                ]  10.03MB/27.14MB
13e8a56469ec Downloading [==================================================>]     184B/184B
13e8a56469ec Verifying Checksum
13e8a56469ec Download complete
c32ce6654453 Extracting [========================>                          ]  13.57MB/27.14MB
38dee6f9a35a Downloading [>                                                  ]  107.7kB/10.62MB
c32ce6654453 Extracting [==============================>                    ]  16.52MB/27.14MB
c32ce6654453 Extracting [===================================>               ]  19.46MB/27.14MB
38dee6f9a35a Downloading [======>                                            ]  1.439MB/10.62MB
c32ce6654453 Extracting [=========================================>         ]  22.71MB/27.14MB
38dee6f9a35a Downloading [============>                                      ]  2.573MB/10.62MB
c32ce6654453 Extracting [=============================================>     ]  24.77MB/27.14MB
38dee6f9a35a Downloading [========================>                          ]  5.158MB/10.62MB
38dee6f9a35a Downloading [==================================>                ]  7.382MB/10.62MB
c32ce6654453 Extracting [===============================================>   ]  25.95MB/27.14MB
38dee6f9a35a Downloading [=============================================>     ]  9.594MB/10.62MB
38dee6f9a35a Verifying Checksum
38dee6f9a35a Download complete
c32ce6654453 Extracting [================================================>  ]  26.25MB/27.14MB
aeab01cd6e68 Downloading [==================================>                ]     659B/960B
c32ce6654453 Extracting [==================================================>]  27.14MB/27.14MB
4270e2b619a4 Downloading [>                                                  ]  529.3kB/108.1MB
aeab01cd6e68 Downloading [==================================================>]     960B/960B
aeab01cd6e68 Verifying Checksum
aeab01cd6e68 Download complete
c32ce6654453 Pull complete
f43a1064aeac Extracting [>                                                  ]  65.54kB/5.132MB
4270e2b619a4 Downloading [>                                                  ]  1.066MB/108.1MB
f43a1064aeac Extracting [=================================>                 ]  3.408MB/5.132MB
4270e2b619a4 Downloading [>                                                  ]  1.594MB/108.1MB
f43a1064aeac Extracting [==================================================>]  5.132MB/5.132MB
f43a1064aeac Pull complete
d34b02acf618 Extracting [==================================================>]  4.598kB/4.598kB
d34b02acf618 Extracting [==================================================>]  4.598kB/4.598kB
4270e2b619a4 Downloading [>                                                  ]  2.127MB/108.1MB
d34b02acf618 Pull complete
13e8a56469ec Extracting [==================================================>]     184B/184B
13e8a56469ec Extracting [==================================================>]     184B/184B
13e8a56469ec Pull complete
4270e2b619a4 Downloading [=>                                                 ]    3.2MB/108.1MB
38dee6f9a35a Extracting [>                                                  ]  131.1kB/10.62MB
38dee6f9a35a Extracting [==============>                                    ]  3.015MB/10.62MB
4270e2b619a4 Downloading [==>                                                ]  4.806MB/108.1MB
38dee6f9a35a Extracting [=============================>                     ]   6.16MB/10.62MB
38dee6f9a35a Extracting [=============================================>     ]  9.699MB/10.62MB
38dee6f9a35a Extracting [==================================================>]  10.62MB/10.62MB
4270e2b619a4 Downloading [==>                                                ]  5.883MB/108.1MB
38dee6f9a35a Pull complete
4270e2b619a4 Downloading [===>                                               ]  7.488MB/108.1MB
4270e2b619a4 Downloading [===>                                               ]  8.017MB/108.1MB
4270e2b619a4 Downloading [====>                                              ]  9.631MB/108.1MB
4270e2b619a4 Downloading [=====>                                             ]  11.23MB/108.1MB
4270e2b619a4 Downloading [=====>                                             ]  12.83MB/108.1MB
4270e2b619a4 Downloading [======>                                            ]  14.43MB/108.1MB
4270e2b619a4 Downloading [=======>                                           ]  16.04MB/108.1MB
4270e2b619a4 Downloading [========>                                          ]  17.66MB/108.1MB
4270e2b619a4 Downloading [========>                                          ]  18.74MB/108.1MB
4270e2b619a4 Downloading [========>                                          ]  19.27MB/108.1MB
4270e2b619a4 Downloading [=========>                                         ]  20.35MB/108.1MB
4270e2b619a4 Downloading [=========>                                         ]  21.44MB/108.1MB
4270e2b619a4 Downloading [==========>                                        ]  22.52MB/108.1MB
4270e2b619a4 Downloading [===========>                                       ]  24.14MB/108.1MB
4270e2b619a4 Downloading [===========>                                       ]  25.74MB/108.1MB
4270e2b619a4 Downloading [============>                                      ]  27.33MB/108.1MB
4270e2b619a4 Downloading [=============>                                     ]  28.92MB/108.1MB
4270e2b619a4 Downloading [==============>                                    ]  30.53MB/108.1MB
4270e2b619a4 Downloading [==============>                                    ]  31.07MB/108.1MB
4270e2b619a4 Downloading [===============>                                   ]  32.67MB/108.1MB
4270e2b619a4 Downloading [===============>                                   ]  33.73MB/108.1MB
4270e2b619a4 Downloading [===============>                                   ]  34.26MB/108.1MB
4270e2b619a4 Downloading [================>                                  ]  35.85MB/108.1MB
4270e2b619a4 Downloading [=================>                                 ]  36.92MB/108.1MB
4270e2b619a4 Downloading [=================>                                 ]     38MB/108.1MB
4270e2b619a4 Downloading [==================>                                ]  40.13MB/108.1MB
4270e2b619a4 Downloading [===================>                               ]   42.8MB/108.1MB
4270e2b619a4 Downloading [====================>                              ]  44.95MB/108.1MB
4270e2b619a4 Downloading [======================>                            ]  47.61MB/108.1MB
4270e2b619a4 Downloading [=======================>                           ]  50.27MB/108.1MB
4270e2b619a4 Downloading [========================>                          ]  52.42MB/108.1MB
4270e2b619a4 Downloading [=========================>                         ]  55.09MB/108.1MB
4270e2b619a4 Downloading [==========================>                        ]  57.76MB/108.1MB
4270e2b619a4 Downloading [===========================>                       ]  59.92MB/108.1MB
4270e2b619a4 Downloading [============================>                      ]  62.04MB/108.1MB
4270e2b619a4 Downloading [=============================>                     ]  64.72MB/108.1MB
4270e2b619a4 Downloading [==============================>                    ]  66.84MB/108.1MB
4270e2b619a4 Downloading [================================>                  ]  69.51MB/108.1MB
4270e2b619a4 Downloading [=================================>                 ]  72.18MB/108.1MB
4270e2b619a4 Downloading [==================================>                ]  74.31MB/108.1MB
4270e2b619a4 Downloading [===================================>               ]  77.53MB/108.1MB
4270e2b619a4 Downloading [====================================>              ]  79.66MB/108.1MB
4270e2b619a4 Downloading [======================================>            ]  82.37MB/108.1MB
4270e2b619a4 Downloading [=======================================>           ]  85.06MB/108.1MB
4270e2b619a4 Downloading [========================================>          ]  87.19MB/108.1MB
4270e2b619a4 Downloading [=========================================>         ]  89.85MB/108.1MB
4270e2b619a4 Downloading [==========================================>        ]  92.54MB/108.1MB
4270e2b619a4 Downloading [===========================================>       ]  94.69MB/108.1MB
4270e2b619a4 Downloading [=============================================>     ]   97.9MB/108.1MB
4270e2b619a4 Downloading [==============================================>    ]    100MB/108.1MB
4270e2b619a4 Downloading [===============================================>   ]  102.7MB/108.1MB
4270e2b619a4 Downloading [================================================>  ]  105.4MB/108.1MB
4270e2b619a4 Downloading [=================================================> ]  107.5MB/108.1MB
4270e2b619a4 Verifying Checksum
4270e2b619a4 Download complete
4270e2b619a4 Extracting [>                                                  ]  557.1kB/108.1MB
4270e2b619a4 Extracting [======>                                            ]  14.48MB/108.1MB
4270e2b619a4 Extracting [===========>                                       ]  25.07MB/108.1MB
4270e2b619a4 Extracting [=================>                                 ]  37.32MB/108.1MB
4270e2b619a4 Extracting [======================>                            ]  49.02MB/108.1MB
4270e2b619a4 Extracting [==============================>                    ]  65.18MB/108.1MB
4270e2b619a4 Extracting [===================================>               ]  76.32MB/108.1MB
4270e2b619a4 Extracting [======================================>            ]  83.56MB/108.1MB
4270e2b619a4 Extracting [============================================>      ]  96.93MB/108.1MB
4270e2b619a4 Extracting [================================================>  ]  104.7MB/108.1MB
4270e2b619a4 Extracting [==================================================>]  108.1MB/108.1MB
4270e2b619a4 Pull complete
aeab01cd6e68 Extracting [==================================================>]     960B/960B
aeab01cd6e68 Extracting [==================================================>]     960B/960B
aeab01cd6e68 Pull complete
relay Pulled
Network sentry-self-hosted_default  Creating
Network sentry-self-hosted_default  Created
Volume "sentry-self-hosted_sentry-clickhouse-log"  Creating
Volume "sentry-self-hosted_sentry-clickhouse-log"  Created
Volume "sentry-self-hosted_sentry-smtp-log"  Creating
Volume "sentry-self-hosted_sentry-smtp-log"  Created
Volume "sentry-self-hosted_sentry-kafka-log"  Creating
Volume "sentry-self-hosted_sentry-kafka-log"  Created
Volume "sentry-self-hosted_sentry-secrets"  Creating
Volume "sentry-self-hosted_sentry-secrets"  Created
Volume "sentry-self-hosted_sentry-smtp"  Creating
Volume "sentry-self-hosted_sentry-smtp"  Created
Volume "sentry-self-hosted_sentry-zookeeper-log"  Creating
Volume "sentry-self-hosted_sentry-zookeeper-log"  Created
Relay credentials written to ../relay/credentials.json.

▶ Generating secret key ...
Secret key written to ../sentry/config.yml

▶ Replacing TSDB ...

▶ Fetching and updating Docker images ...
22.5.0: Pulling from getsentry/sentry
Digest: sha256:d3d99e21c92fc6016f4f55169f9c97448461c9c5ed37aa9f11df7b805c5bd96d
Status: Image is up to date for getsentry/sentry:22.5.0
docker.io/getsentry/sentry:22.5.0

▶ Building and tagging Docker images ...

#1 [symbolicator-cleanup-self-hosted-local internal] load build definition from Dockerfile
#1 transferring dockerfile: 250B 0.0s done
#1 DONE 0.0s

#2 [snuba-cleanup-self-hosted-local internal] load build definition from Dockerfile
#2 transferring dockerfile: 250B done
#2 DONE 0.1s

#3 [snuba-cleanup-self-hosted-local internal] load .dockerignore
#3 ...

#4 [sentry-cleanup-self-hosted-local internal] load build definition from Dockerfile
#4 transferring dockerfile: 250B done
#4 DONE 0.2s

#5 [sentry-cleanup-self-hosted-local internal] load .dockerignore
#5 transferring context: 2B done
#5 DONE 0.1s

#6 [symbolicator-cleanup-self-hosted-local internal] load .dockerignore
#6 transferring context: 2B done
#6 DONE 0.1s

#3 [snuba-cleanup-self-hosted-local internal] load .dockerignore
#3 transferring context: 2B done
#3 DONE 0.1s

#7 [sentry-cleanup-self-hosted-local internal] load metadata for docker.io/getsentry/sentry:22.5.0
#7 DONE 0.0s

#8 [symbolicator-cleanup-self-hosted-local internal] load metadata for docker.io/getsentry/symbolicator:0.5.0
#8 DONE 0.0s

#9 [snuba-cleanup-self-hosted-local internal] load metadata for docker.io/getsentry/snuba:22.5.0
#9 DONE 0.0s

#10 [snuba-cleanup-self-hosted-local 1/3] FROM docker.io/getsentry/snuba:22.5.0
#10 ...

#11 [symbolicator-cleanup-self-hosted-local internal] load build context
#11 transferring context: 662B 0.0s done
#11 DONE 0.2s

#12 [snuba-cleanup-self-hosted-local internal] load build context
#12 transferring context: 662B done
#12 DONE 0.3s

#13 [sentry-cleanup-self-hosted-local internal] load build context
#13 transferring context: 662B done
#13 DONE 0.4s

#14 [sentry-cleanup-self-hosted-local 1/3] FROM docker.io/getsentry/sentry:22.5.0
#14 ...

#15 [symbolicator-cleanup-self-hosted-local 1/3] FROM docker.io/getsentry/symbolicator:0.5.0
#15 DONE 1.2s

#16 [symbolicator-cleanup-self-hosted-local 2/3] RUN apt-get update && apt-get install -y --no-install-recommends cron &&     rm -r /var/lib/apt/lists/*
#16 0.429 Get:1 http://security.debian.org/debian-security stretch/updates InRelease [53.0 kB]
#16 0.435 Ign:2 http://deb.debian.org/debian stretch InRelease
#16 0.517 Get:3 http://security.debian.org/debian-security stretch/updates/main amd64 Packages [766 kB]
#16 0.683 Get:4 http://deb.debian.org/debian stretch-updates InRelease [93.6 kB]
#16 0.957 Get:5 http://deb.debian.org/debian stretch Release [118 kB]
#16 ...

#10 [snuba-cleanup-self-hosted-local 1/3] FROM docker.io/getsentry/snuba:22.5.0
#10 DONE 2.1s

#16 [symbolicator-cleanup-self-hosted-local 2/3] RUN apt-get update && apt-get install -y --no-install-recommends cron &&     rm -r /var/lib/apt/lists/*
#16 1.218 Get:6 http://deb.debian.org/debian stretch Release.gpg [3177 B]
#16 1.296 Get:7 http://deb.debian.org/debian stretch/main amd64 Packages [7080 kB]
#16 ...

#14 [sentry-cleanup-self-hosted-local 1/3] FROM docker.io/getsentry/sentry:22.5.0
#14 DONE 2.7s

#17 [snuba-cleanup-self-hosted-local 2/3] RUN apt-get update && apt-get install -y --no-install-recommends cron &&     rm -r /var/lib/apt/lists/*
#0 0.444 Get:1 http://deb.debian.org/debian bullseye InRelease [116 kB]
#0 0.446 Get:2 http://security.debian.org/debian-security bullseye-security InRelease [44.1 kB]
#0 0.477 Get:3 http://deb.debian.org/debian bullseye-updates InRelease [39.4 kB]
#0 0.564 Get:4 http://security.debian.org/debian-security bullseye-security/main amd64 Packages [147 kB]
#0 0.665 Get:5 http://deb.debian.org/debian bullseye/main amd64 Packages [8182 kB]
#17 1.353 Get:6 http://deb.debian.org/debian bullseye-updates/main amd64 Packages [2596 B]
#17 2.251 Fetched 8532 kB in 2s (4529 kB/s)
#17 2.251 Reading package lists...
#17 2.655 Reading package lists...
#17 3.089 Building dependency tree...
#17 3.189 Reading state information...
#17 3.296 The following additional packages will be installed:
#17 3.296   sensible-utils
#17 3.297 Suggested packages:
#17 3.297   anacron logrotate checksecurity
#17 3.297 Recommended packages:
#17 3.297   default-mta | mail-transport-agent
#17 3.324 The following NEW packages will be installed:
#17 3.324   cron sensible-utils
#17 3.359 0 upgraded, 2 newly installed, 0 to remove and 2 not upgraded.
#17 3.359 Need to get 114 kB of archives.
#17 3.359 After this operation, 314 kB of additional disk space will be used.
#17 3.359 Get:1 http://deb.debian.org/debian bullseye/main amd64 sensible-utils all 0.0.14 [14.8 kB]
#17 3.371 Get:2 http://deb.debian.org/debian bullseye/main amd64 cron amd64 3.0pl1-137 [99.6 kB]
#17 4.876 debconf: delaying package configuration, since apt-utils is not installed
#17 5.970 Fetched 114 kB in 0s (2106 kB/s)
#17 ...

#16 [symbolicator-cleanup-self-hosted-local 2/3] RUN apt-get update && apt-get install -y --no-install-recommends cron &&     rm -r /var/lib/apt/lists/*
#16 2.698 Fetched 8114 kB in 2s (3478 kB/s)
#16 2.698 Reading package lists...
#16 3.234 Reading package lists...
#16 3.673 Building dependency tree...
#16 3.751 Reading state information...
#16 3.809 Suggested packages:
#16 3.809   anacron logrotate checksecurity
#16 3.809 Recommended packages:
#16 3.809   exim4 | postfix | mail-transport-agent
#16 3.815 The following NEW packages will be installed:
#16 3.815   cron
#16 3.842 0 upgraded, 1 newly installed, 0 to remove and 2 not upgraded.
#16 3.842 Need to get 96.6 kB of archives.
#16 3.842 After this operation, 257 kB of additional disk space will be used.
#16 3.842 Get:1 http://security.debian.org/debian-security stretch/updates/main amd64 cron amd64 3.0pl1-128+deb9u2 [96.6 kB]
#16 3.935 debconf: delaying package configuration, since apt-utils is not installed
#16 3.954 Fetched 96.6 kB in 0s (2691 kB/s)
#16 4.004 Selecting previously unselected package cron.
(Reading database ... 6672 files and directories currently installed.)
#16 4.007 Preparing to unpack .../cron_3.0pl1-128+deb9u2_amd64.deb ...
#16 4.022 Unpacking cron (3.0pl1-128+deb9u2) ...
#16 4.085 Setting up cron (3.0pl1-128+deb9u2) ...
#16 4.226 Adding group `crontab' (GID 101) ...
#16 4.254 Done.
#16 4.360 update-rc.d: warning: start and stop actions are no longer supported; falling back to defaults
#16 4.362 invoke-rc.d: could not determine current runlevel
#16 4.364 invoke-rc.d: policy-rc.d denied execution of start.
#16 DONE 7.6s

#18 [sentry-cleanup-self-hosted-local 2/3] RUN apt-get update && apt-get install -y --no-install-recommends cron &&     rm -r /var/lib/apt/lists/*
#0 0.314 Get:1 http://security.debian.org/debian-security bullseye-security InRelease [44.1 kB]
#0 0.316 Get:2 http://deb.debian.org/debian bullseye InRelease [116 kB]
#0 0.351 Get:3 http://deb.debian.org/debian bullseye-updates InRelease [39.4 kB]
#0 0.428 Get:4 http://security.debian.org/debian-security bullseye-security/main amd64 Packages [147 kB]
#0 0.516 Get:5 http://deb.debian.org/debian bullseye/main amd64 Packages [8182 kB]
#0 1.431 Get:6 http://deb.debian.org/debian bullseye-updates/main amd64 Packages [2596 B]
#0 2.254 Fetched 8532 kB in 2s (4335 kB/s)
#0 2.254 Reading package lists...
#0 2.712 Reading package lists...
#0 3.098 Building dependency tree...
#0 3.182 Reading state information...
#0 3.261 The following additional packages will be installed:
#0 3.261   sensible-utils
#0 3.261 Suggested packages:
#0 3.261   anacron logrotate checksecurity
#0 3.261 Recommended packages:
#0 3.261   default-mta | mail-transport-agent
#0 3.287 The following NEW packages will be installed:
#0 3.288   cron sensible-utils
#0 3.319 0 upgraded, 2 newly installed, 0 to remove and 14 not upgraded.
#0 3.319 Need to get 114 kB of archives.
#0 3.319 After this operation, 314 kB of additional disk space will be used.
#0 3.319 Get:1 http://deb.debian.org/debian bullseye/main amd64 sensible-utils all 0.0.14 [14.8 kB]
#0 3.332 Get:2 http://deb.debian.org/debian bullseye/main amd64 cron amd64 3.0pl1-137 [99.6 kB]
#0 4.484 debconf: delaying package configuration, since apt-utils is not installed
#0 5.775 Fetched 114 kB in 0s (2036 kB/s)
#18 ...

#19 [symbolicator-cleanup-self-hosted-local 3/3] COPY entrypoint.sh /entrypoint.sh
#19 DONE 0.1s

#18 [sentry-cleanup-self-hosted-local 2/3] RUN apt-get update && apt-get install -y --no-install-recommends cron &&     rm -r /var/lib/apt/lists/*
#18 5.972 Selecting previously unselected package sensible-utils.
(Reading database ... 12313 files and directories currently installed.)
#18 5.979 Preparing to unpack .../sensible-utils_0.0.14_all.deb ...
#18 5.996 Unpacking sensible-utils (0.0.14) ...
#18 ...

#17 [snuba-cleanup-self-hosted-local 2/3] RUN apt-get update && apt-get install -y --no-install-recommends cron &&     rm -r /var/lib/apt/lists/*
#17 6.512 Selecting previously unselected package sensible-utils.
(Reading database ... 7023 files and directories currently installed.)
#17 6.517 Preparing to unpack .../sensible-utils_0.0.14_all.deb ...
#17 6.536 Unpacking sensible-utils (0.0.14) ...
#17 6.699 Selecting previously unselected package cron.
#17 6.700 Preparing to unpack .../cron_3.0pl1-137_amd64.deb ...
#17 6.715 Unpacking cron (3.0pl1-137) ...
#17 ...

#20 [symbolicator-cleanup-self-hosted-local] exporting to image
#20 exporting layers 0.1s done
#20 writing image sha256:55e5ee92a99c44b3ec96305d2c0819f2725542b0531c2e0778985736a83e182a done
#20 naming to docker.io/library/symbolicator-cleanup-self-hosted-local done
#20 DONE 0.2s

#17 [snuba-cleanup-self-hosted-local 2/3] RUN apt-get update && apt-get install -y --no-install-recommends cron &&     rm -r /var/lib/apt/lists/*
#17 6.834 Setting up sensible-utils (0.0.14) ...
#17 6.890 Setting up cron (3.0pl1-137) ...
#17 7.094 Adding group `crontab' (GID 101) ...
#17 7.149 Done.
#17 7.174 invoke-rc.d: could not determine current runlevel
#17 7.176 invoke-rc.d: policy-rc.d denied execution of start.
#17 DONE 7.3s

#18 [sentry-cleanup-self-hosted-local 2/3] RUN apt-get update && apt-get install -y --no-install-recommends cron &&     rm -r /var/lib/apt/lists/*
#18 6.160 Selecting previously unselected package cron.
#18 6.161 Preparing to unpack .../cron_3.0pl1-137_amd64.deb ...
#18 6.177 Unpacking cron (3.0pl1-137) ...
#18 6.297 Setting up sensible-utils (0.0.14) ...
#18 6.352 Setting up cron (3.0pl1-137) ...
#18 6.555 Adding group `crontab' (GID 101) ...
#18 6.610 Done.
#18 6.635 invoke-rc.d: could not determine current runlevel
#18 6.638 invoke-rc.d: policy-rc.d denied execution of start.
#18 DONE 6.8s

#21 [sentry-cleanup-self-hosted-local 3/3] COPY entrypoint.sh /entrypoint.sh
#21 DONE 0.2s

#22 [snuba-cleanup-self-hosted-local 3/3] COPY entrypoint.sh /entrypoint.sh
#22 DONE 0.1s

#20 [snuba-cleanup-self-hosted-local] exporting to image
#20 exporting layers 0.2s done
#20 exporting layers 0.1s done
#20 writing image sha256:b6d0d102fdc734471ba6740b911379b54d3766103e9ca18aae43a0b36108f0c8 done
#20 naming to docker.io/library/snuba-cleanup-self-hosted-local done
#20 writing image sha256:a58c3967aef67ef84ac404e1bee9e60acb85a61e2703c5d0a1e899376de96c23
#20 writing image sha256:a58c3967aef67ef84ac404e1bee9e60acb85a61e2703c5d0a1e899376de96c23 done
#20 naming to docker.io/library/sentry-cleanup-self-hosted-local done
#20 DONE 0.6s

Use 'docker scan' to run Snyk tests against images to find vulnerabilities and learn how to fix them

Docker images built.

▶ Setting up Zookeeper ...

▶ Downloading and installing wal2json ...
Unable to find image 'curlimages/curl:7.77.0' locally
7.77.0: Pulling from curlimages/curl
339de151aab4: Pulling fs layer
9576f9a419e9: Pulling fs layer
5b67e0e9bce9: Pulling fs layer
7c0640a6c519: Pulling fs layer
1d8f7d50664a: Pulling fs layer
3ce74da83aeb: Pulling fs layer
7094fabf5e16: Pulling fs layer
2f4d0b649c1f: Pulling fs layer
110e7f874674: Pulling fs layer
7c0640a6c519: Waiting
1d8f7d50664a: Waiting
3ce74da83aeb: Waiting
7094fabf5e16: Waiting
2f4d0b649c1f: Waiting
110e7f874674: Waiting
9576f9a419e9: Verifying Checksum
9576f9a419e9: Download complete
339de151aab4: Verifying Checksum
339de151aab4: Download complete
339de151aab4: Pull complete
5b67e0e9bce9: Verifying Checksum
5b67e0e9bce9: Download complete
9576f9a419e9: Pull complete
5b67e0e9bce9: Pull complete
7c0640a6c519: Verifying Checksum
7c0640a6c519: Download complete
7c0640a6c519: Pull complete
1d8f7d50664a: Verifying Checksum
1d8f7d50664a: Download complete
1d8f7d50664a: Pull complete
3ce74da83aeb: Download complete
3ce74da83aeb: Pull complete
7094fabf5e16: Verifying Checksum
7094fabf5e16: Download complete
7094fabf5e16: Pull complete
2f4d0b649c1f: Verifying Checksum
2f4d0b649c1f: Download complete
2f4d0b649c1f: Pull complete
110e7f874674: Download complete
110e7f874674: Pull complete
Digest: sha256:6e0a786e3e5181df00eaf3a0a1749c18a6bb20b01c9bd192ea72176ce8a1c94b
Status: Downloaded newer image for curlimages/curl:7.77.0
  % Total    % Received % Xferd  Average Speed   Time    Time     Time  Current
                                 Dload  Upload   Total   Spent    Left  Speed
100  5417  100  5417    0     0  16818      0 --:--:-- --:--:-- --:--:-- 16822
  % Total    % Received % Xferd  Average Speed   Time    Time     Time  Current
                                 Dload  Upload   Total   Spent    Left  Speed
  0     0    0     0    0     0      0      0 --:--:-- --:--:-- --:--:--     0
100  101k  100  101k    0     0   140k      0 --:--:-- --:--:-- --:--:--  140k

▶ Bootstrapping and migrating Snuba ...
Container sentry-self-hosted-redis-1  Creating
Container sentry-self-hosted-zookeeper-1  Creating
Container sentry-self-hosted-clickhouse-1  Creating
Container sentry-self-hosted-redis-1  Created
Container sentry-self-hosted-clickhouse-1  Created
Container sentry-self-hosted-zookeeper-1  Created
Container sentry-self-hosted-kafka-1  Creating
Container sentry-self-hosted-kafka-1  Created
Container sentry-self-hosted-redis-1  Starting
Container sentry-self-hosted-clickhouse-1  Starting
Container sentry-self-hosted-zookeeper-1  Starting
Container sentry-self-hosted-redis-1  Started
Container sentry-self-hosted-zookeeper-1  Started
Container sentry-self-hosted-zookeeper-1  Waiting
Container sentry-self-hosted-clickhouse-1  Started
Container sentry-self-hosted-zookeeper-1  Healthy
Container sentry-self-hosted-kafka-1  Starting
Container sentry-self-hosted-kafka-1  Started
2022-05-21 13:24:19,186 Attempting to connect to Kafka (attempt 0)...
2022-05-21 13:24:19,201 Connected to Kafka on attempt 0
2022-05-21 13:24:19,202 Creating Kafka topics...
2022-05-21 13:24:19,737 Topic events created
2022-05-21 13:24:19,737 Topic event-replacements created
2022-05-21 13:24:19,737 Topic snuba-commit-log created
2022-05-21 13:24:19,737 Topic cdc created
2022-05-21 13:24:19,737 Topic snuba-metrics created
2022-05-21 13:24:19,737 Topic outcomes created
2022-05-21 13:24:19,738 Topic ingest-sessions created
2022-05-21 13:24:19,738 Topic snuba-sessions-commit-log created
2022-05-21 13:24:19,738 Topic snuba-metrics-commit-log created
2022-05-21 13:24:19,738 Topic scheduled-subscriptions-events created
2022-05-21 13:24:19,738 Topic scheduled-subscriptions-transactions created
2022-05-21 13:24:19,738 Topic scheduled-subscriptions-sessions created
2022-05-21 13:24:19,738 Topic scheduled-subscriptions-metrics created
2022-05-21 13:24:19,738 Topic events-subscription-results created
2022-05-21 13:24:19,738 Topic transactions-subscription-results created
2022-05-21 13:24:19,738 Topic sessions-subscription-results created
2022-05-21 13:24:19,738 Topic metrics-subscription-results created
2022-05-21 13:24:19,738 Topic snuba-queries created
2022-05-21 13:24:19,738 Topic processed-profiles created
2022-05-21 13:24:19,739 Topic snuba-dead-letter-inserts created
2022-05-21 13:24:19,739 Topic snuba-dead-letter-metrics created
2022-05-21 13:24:19,739 Topic snuba-dead-letter-sessions created
Container sentry-self-hosted-zookeeper-1  Running
Container sentry-self-hosted-clickhouse-1  Running
Container sentry-self-hosted-kafka-1  Running
Container sentry-self-hosted-redis-1  Running
Container sentry-self-hosted-zookeeper-1  Waiting
Container sentry-self-hosted-zookeeper-1  Healthy
2022-05-21 13:24:24,559 Running migration: 0001_migrations
2022-05-21 13:24:24,567 Finished: 0001_migrations
/usr/local/lib/python3.8/site-packages/clickhouse_driver/columns/datetimecolumn.py:199: PytzUsageWarning: The zone attribute is specific to pytz's interface; please migrate to a new time zone provider. For more details on how to do so, see https://pytz-deprecation-shim.readthedocs.io/en/latest/migration.html
  local_timezone = get_localzone().zone
2022-05-21 13:24:24,574 Running migration: 0001_events_initial
2022-05-21 13:24:24,588 Finished: 0001_events_initial
2022-05-21 13:24:24,593 Running migration: 0002_events_onpremise_compatibility
2022-05-21 13:24:24,683 Finished: 0002_events_onpremise_compatibility
2022-05-21 13:24:24,688 Running migration: 0003_errors
2022-05-21 13:24:24,700 Finished: 0003_errors
2022-05-21 13:24:24,705 Running migration: 0004_errors_onpremise_compatibility
2022-05-21 13:24:24,718 Finished: 0004_errors_onpremise_compatibility
2022-05-21 13:24:24,723 Running migration: 0005_events_tags_hash_map
2022-05-21 13:24:24,740 Finished: 0005_events_tags_hash_map
2022-05-21 13:24:24,745 Running migration: 0006_errors_tags_hash_map
2022-05-21 13:24:24,757 Finished: 0006_errors_tags_hash_map
2022-05-21 13:24:24,762 Running migration: 0007_groupedmessages
2022-05-21 13:24:24,774 Finished: 0007_groupedmessages
2022-05-21 13:24:24,778 Running migration: 0008_groupassignees
2022-05-21 13:24:24,788 Finished: 0008_groupassignees
2022-05-21 13:24:24,793 Running migration: 0009_errors_add_http_fields
2022-05-21 13:24:24,812 Finished: 0009_errors_add_http_fields
2022-05-21 13:24:24,817 Running migration: 0010_groupedmessages_onpremise_compatibility
2022-05-21 13:24:24,822 Finished: 0010_groupedmessages_onpremise_compatibility
2022-05-21 13:24:24,827 Running migration: 0011_rebuild_errors
2022-05-21 13:24:24,858 Finished: 0011_rebuild_errors
2022-05-21 13:24:24,863 Running migration: 0012_errors_make_level_nullable
2022-05-21 13:24:24,875 Finished: 0012_errors_make_level_nullable
2022-05-21 13:24:24,879 Running migration: 0013_errors_add_hierarchical_hashes
2022-05-21 13:24:24,909 Finished: 0013_errors_add_hierarchical_hashes
2022-05-21 13:24:24,914 Running migration: 0014_backfill_errors
2022-05-21 13:24:24,919 Starting migration from 2022-05-16
2022-05-21 13:24:24,932 Migrated 2022-05-16. (1 of 13 partitions done)
2022-05-21 13:24:24,944 Migrated 2022-05-09. (2 of 13 partitions done)
2022-05-21 13:24:24,958 Migrated 2022-05-02. (3 of 13 partitions done)
2022-05-21 13:24:24,970 Migrated 2022-04-25. (4 of 13 partitions done)
2022-05-21 13:24:24,983 Migrated 2022-04-18. (5 of 13 partitions done)
2022-05-21 13:24:24,996 Migrated 2022-04-11. (6 of 13 partitions done)
2022-05-21 13:24:25,009 Migrated 2022-04-04. (7 of 13 partitions done)
2022-05-21 13:24:25,022 Migrated 2022-03-28. (8 of 13 partitions done)
2022-05-21 13:24:25,035 Migrated 2022-03-21. (9 of 13 partitions done)
2022-05-21 13:24:25,048 Migrated 2022-03-14. (10 of 13 partitions done)
2022-05-21 13:24:25,061 Migrated 2022-03-07. (11 of 13 partitions done)
2022-05-21 13:24:25,074 Migrated 2022-02-28. (12 of 13 partitions done)
2022-05-21 13:24:25,087 Migrated 2022-02-21. (13 of 13 partitions done)
2022-05-21 13:24:25,087 Done. Optimizing.
2022-05-21 13:24:25,089 Finished: 0014_backfill_errors
2022-05-21 13:24:25,094 Running migration: 0015_truncate_events
2022-05-21 13:24:25,098 Finished: 0015_truncate_events
2022-05-21 13:24:25,104 Running migration: 0016_drop_legacy_events
2022-05-21 13:24:25,108 Finished: 0016_drop_legacy_events
2022-05-21 13:24:25,113 Running migration: 0001_transactions
2022-05-21 13:24:25,124 Finished: 0001_transactions
2022-05-21 13:24:25,129 Running migration: 0002_transactions_onpremise_fix_orderby_and_partitionby
2022-05-21 13:24:25,134 Finished: 0002_transactions_onpremise_fix_orderby_and_partitionby
2022-05-21 13:24:25,139 Running migration: 0003_transactions_onpremise_fix_columns
2022-05-21 13:24:25,266 Finished: 0003_transactions_onpremise_fix_columns
2022-05-21 13:24:25,270 Running migration: 0004_transactions_add_tags_hash_map
2022-05-21 13:24:25,281 Finished: 0004_transactions_add_tags_hash_map
2022-05-21 13:24:25,286 Running migration: 0005_transactions_add_measurements
2022-05-21 13:24:25,297 Finished: 0005_transactions_add_measurements
2022-05-21 13:24:25,301 Running migration: 0006_transactions_add_http_fields
2022-05-21 13:24:25,319 Finished: 0006_transactions_add_http_fields
2022-05-21 13:24:25,324 Running migration: 0007_transactions_add_discover_cols
2022-05-21 13:24:25,359 Finished: 0007_transactions_add_discover_cols
2022-05-21 13:24:25,363 Running migration: 0008_transactions_add_timestamp_index
2022-05-21 13:24:25,375 Finished: 0008_transactions_add_timestamp_index
2022-05-21 13:24:25,379 Running migration: 0009_transactions_fix_title_and_message
2022-05-21 13:24:25,401 Finished: 0009_transactions_fix_title_and_message
2022-05-21 13:24:25,405 Running migration: 0010_transactions_nullable_trace_id
2022-05-21 13:24:25,416 Finished: 0010_transactions_nullable_trace_id
2022-05-21 13:24:25,420 Running migration: 0011_transactions_add_span_op_breakdowns
2022-05-21 13:24:25,431 Finished: 0011_transactions_add_span_op_breakdowns
2022-05-21 13:24:25,436 Running migration: 0012_transactions_add_spans
2022-05-21 13:24:25,499 Finished: 0012_transactions_add_spans
2022-05-21 13:24:25,504 Running migration: 0013_transactions_reduce_spans_exclusive_time
2022-05-21 13:24:25,526 Finished: 0013_transactions_reduce_spans_exclusive_time
2022-05-21 13:24:25,530 Running migration: 0014_transactions_remove_flattened_columns
2022-05-21 13:24:25,551 Finished: 0014_transactions_remove_flattened_columns
2022-05-21 13:24:25,556 Running migration: 0001_discover_merge_table
2022-05-21 13:24:25,571 Finished: 0001_discover_merge_table
2022-05-21 13:24:25,575 Running migration: 0002_discover_add_deleted_tags_hash_map
2022-05-21 13:24:25,590 Finished: 0002_discover_add_deleted_tags_hash_map
2022-05-21 13:24:25,594 Running migration: 0003_discover_fix_user_column
2022-05-21 13:24:25,605 Finished: 0003_discover_fix_user_column
2022-05-21 13:24:25,609 Running migration: 0004_discover_fix_title_and_message
2022-05-21 13:24:25,632 Finished: 0004_discover_fix_title_and_message
2022-05-21 13:24:25,637 Running migration: 0005_discover_fix_transaction_name
2022-05-21 13:24:25,648 Finished: 0005_discover_fix_transaction_name
2022-05-21 13:24:25,652 Running migration: 0006_discover_add_trace_id
2022-05-21 13:24:25,663 Finished: 0006_discover_add_trace_id
2022-05-21 13:24:25,668 Running migration: 0007_discover_add_span_id
2022-05-21 13:24:25,678 Finished: 0007_discover_add_span_id
2022-05-21 13:24:25,683 Running migration: 0001_outcomes
2022-05-21 13:24:25,706 Finished: 0001_outcomes
2022-05-21 13:24:25,710 Running migration: 0002_outcomes_remove_size_and_bytes
2022-05-21 13:24:25,733 Finished: 0002_outcomes_remove_size_and_bytes
2022-05-21 13:24:25,738 Running migration: 0003_outcomes_add_category_and_quantity
2022-05-21 13:24:25,764 Finished: 0003_outcomes_add_category_and_quantity
2022-05-21 13:24:25,769 Running migration: 0004_outcomes_matview_additions
2022-05-21 13:24:25,779 Finished: 0004_outcomes_matview_additions
2022-05-21 13:24:25,785 Running migration: 0001_metrics_buckets
2022-05-21 13:24:25,795 Finished: 0001_metrics_buckets
2022-05-21 13:24:25,800 Running migration: 0002_metrics_sets
2022-05-21 13:24:25,837 Finished: 0002_metrics_sets
2022-05-21 13:24:25,841 Running migration: 0003_counters_to_buckets
2022-05-21 13:24:25,851 Finished: 0003_counters_to_buckets
2022-05-21 13:24:25,855 Running migration: 0004_metrics_counters
2022-05-21 13:24:25,891 Finished: 0004_metrics_counters
2022-05-21 13:24:25,895 Running migration: 0005_metrics_distributions_buckets
2022-05-21 13:24:25,905 Finished: 0005_metrics_distributions_buckets
2022-05-21 13:24:25,910 Running migration: 0006_metrics_distributions
2022-05-21 13:24:25,947 Finished: 0006_metrics_distributions
2022-05-21 13:24:25,952 Running migration: 0007_metrics_sets_granularity_10
2022-05-21 13:24:25,965 Finished: 0007_metrics_sets_granularity_10
2022-05-21 13:24:25,970 Running migration: 0008_metrics_counters_granularity_10
2022-05-21 13:24:25,979 Finished: 0008_metrics_counters_granularity_10
2022-05-21 13:24:25,983 Running migration: 0009_metrics_distributions_granularity_10
2022-05-21 13:24:25,992 Finished: 0009_metrics_distributions_granularity_10
2022-05-21 13:24:25,996 Running migration: 0010_metrics_sets_granularity_1h
2022-05-21 13:24:26,006 Finished: 0010_metrics_sets_granularity_1h
2022-05-21 13:24:26,010 Running migration: 0011_metrics_counters_granularity_1h
2022-05-21 13:24:26,019 Finished: 0011_metrics_counters_granularity_1h
2022-05-21 13:24:26,024 Running migration: 0012_metrics_distributions_granularity_1h
2022-05-21 13:24:26,034 Finished: 0012_metrics_distributions_granularity_1h
2022-05-21 13:24:26,038 Running migration: 0013_metrics_sets_granularity_1d
2022-05-21 13:24:26,047 Finished: 0013_metrics_sets_granularity_1d
2022-05-21 13:24:26,052 Running migration: 0014_metrics_counters_granularity_1d
2022-05-21 13:24:26,061 Finished: 0014_metrics_counters_granularity_1d
2022-05-21 13:24:26,065 Running migration: 0015_metrics_distributions_granularity_1d
2022-05-21 13:24:26,075 Finished: 0015_metrics_distributions_granularity_1d
2022-05-21 13:24:26,079 Running migration: 0016_metrics_sets_consolidated_granularity
2022-05-21 13:24:26,088 Finished: 0016_metrics_sets_consolidated_granularity
2022-05-21 13:24:26,092 Running migration: 0017_metrics_counters_consolidated_granularity
2022-05-21 13:24:26,106 Finished: 0017_metrics_counters_consolidated_granularity
2022-05-21 13:24:26,110 Running migration: 0018_metrics_distributions_consolidated_granularity
2022-05-21 13:24:26,119 Finished: 0018_metrics_distributions_consolidated_granularity
2022-05-21 13:24:26,123 Running migration: 0019_aggregate_tables_add_ttl
2022-05-21 13:24:26,147 Finished: 0019_aggregate_tables_add_ttl
2022-05-21 13:24:26,152 Running migration: 0020_polymorphic_buckets_table
2022-05-21 13:24:26,162 Finished: 0020_polymorphic_buckets_table
2022-05-21 13:24:26,167 Running migration: 0021_polymorphic_bucket_materialized_views
2022-05-21 13:24:26,187 Finished: 0021_polymorphic_bucket_materialized_views
2022-05-21 13:24:26,191 Running migration: 0022_repartition_polymorphic_table
2022-05-21 13:24:26,201 Finished: 0022_repartition_polymorphic_table
2022-05-21 13:24:26,205 Running migration: 0023_polymorphic_repartitioned_bucket_matview
2022-05-21 13:24:26,229 Finished: 0023_polymorphic_repartitioned_bucket_matview
2022-05-21 13:24:26,234 Running migration: 0024_metrics_distributions_add_histogram
2022-05-21 13:24:26,262 Finished: 0024_metrics_distributions_add_histogram
2022-05-21 13:24:26,267 Running migration: 0025_metrics_counters_aggregate_v2
2022-05-21 13:24:26,298 Finished: 0025_metrics_counters_aggregate_v2
2022-05-21 13:24:26,302 Running migration: 0026_metrics_counters_v2_writing_matview
2022-05-21 13:24:26,307 Finished: 0026_metrics_counters_v2_writing_matview
2022-05-21 13:24:26,311 Running migration: 0027_fix_migration_0026
2022-05-21 13:24:26,321 Finished: 0027_fix_migration_0026
2022-05-21 13:24:26,326 Running migration: 0028_metrics_sets_aggregate_v2
2022-05-21 13:24:26,356 Finished: 0028_metrics_sets_aggregate_v2
2022-05-21 13:24:26,361 Running migration: 0029_metrics_distributions_aggregate_v2
2022-05-21 13:24:26,392 Finished: 0029_metrics_distributions_aggregate_v2
2022-05-21 13:24:26,396 Running migration: 0030_metrics_distributions_v2_writing_mv
2022-05-21 13:24:26,407 Finished: 0030_metrics_distributions_v2_writing_mv
2022-05-21 13:24:26,411 Running migration: 0031_metrics_sets_v2_writing_mv
2022-05-21 13:24:26,420 Finished: 0031_metrics_sets_v2_writing_mv
2022-05-21 13:24:26,425 Running migration: 0032_redo_0030_and_0031_without_timestamps
2022-05-21 13:24:26,442 Finished: 0032_redo_0030_and_0031_without_timestamps
2022-05-21 13:24:26,447 Running migration: 0033_metrics_cleanup_old_views
2022-05-21 13:24:26,469 Finished: 0033_metrics_cleanup_old_views
2022-05-21 13:24:26,473 Running migration: 0034_metrics_cleanup_old_tables
2022-05-21 13:24:26,483 Finished: 0034_metrics_cleanup_old_tables
2022-05-21 13:24:26,489 Running migration: 0001_sessions
2022-05-21 13:24:26,512 Finished: 0001_sessions
2022-05-21 13:24:26,517 Running migration: 0002_sessions_aggregates
2022-05-21 13:24:26,585 Finished: 0002_sessions_aggregates
2022-05-21 13:24:26,590 Running migration: 0003_sessions_matview
2022-05-21 13:24:26,602 Finished: 0003_sessions_matview
Finished running migrations

▶ Creating additional Kafka topics ...
Container sentry-self-hosted-zookeeper-1  Running
Container sentry-self-hosted-zookeeper-1  Waiting
Container sentry-self-hosted-zookeeper-1  Healthy
Created topic ingest-attachments.

Container sentry-self-hosted-zookeeper-1  Running
Container sentry-self-hosted-zookeeper-1  Waiting
Container sentry-self-hosted-zookeeper-1  Healthy
Created topic ingest-transactions.

Container sentry-self-hosted-zookeeper-1  Running
Container sentry-self-hosted-zookeeper-1  Waiting
Container sentry-self-hosted-zookeeper-1  Healthy
Created topic ingest-events.


▶ Ensuring proper PostgreSQL version ...

▶ Setting up / migrating database ...
Container sentry-self-hosted-postgres-1  Creating
Container sentry-self-hosted-smtp-1  Creating
Container sentry-self-hosted-memcached-1  Creating
Container sentry-self-hosted-symbolicator-1  Creating
Container sentry-self-hosted-zookeeper-1  Running
Container sentry-self-hosted-clickhouse-1  Running
Container sentry-self-hosted-redis-1  Running
Container sentry-self-hosted-kafka-1  Running
Container sentry-self-hosted-snuba-api-1  Creating
Container sentry-self-hosted-snuba-sessions-consumer-1  Creating
Container sentry-self-hosted-snuba-replacer-1  Creating
Container sentry-self-hosted-snuba-consumer-1  Creating
Container sentry-self-hosted-snuba-transactions-consumer-1  Creating
Container sentry-self-hosted-snuba-outcomes-consumer-1  Creating
Container sentry-self-hosted-snuba-subscription-consumer-transactions-1  Creating
Container sentry-self-hosted-snuba-subscription-consumer-events-1  Creating
Container sentry-self-hosted-memcached-1  Created
Container sentry-self-hosted-snuba-subscription-consumer-transactions-1  Created
Container sentry-self-hosted-snuba-subscription-consumer-events-1  Created
Container sentry-self-hosted-snuba-replacer-1  Created
Container sentry-self-hosted-smtp-1  Created
Container sentry-self-hosted-snuba-api-1  Created
Container sentry-self-hosted-symbolicator-1  Created
Container sentry-self-hosted-snuba-outcomes-consumer-1  Created
Container sentry-self-hosted-postgres-1  Created
Container sentry-self-hosted-snuba-sessions-consumer-1  Created
Container sentry-self-hosted-snuba-transactions-consumer-1  Created
Container sentry-self-hosted-snuba-consumer-1  Created
Container sentry-self-hosted-smtp-1  Starting
Container sentry-self-hosted-memcached-1  Starting
Container sentry-self-hosted-postgres-1  Starting
Container sentry-self-hosted-symbolicator-1  Starting
Container sentry-self-hosted-zookeeper-1  Waiting
Container sentry-self-hosted-memcached-1  Started
Container sentry-self-hosted-zookeeper-1  Healthy
Container sentry-self-hosted-redis-1  Waiting
Container sentry-self-hosted-redis-1  Waiting
Container sentry-self-hosted-redis-1  Waiting
Container sentry-self-hosted-kafka-1  Waiting
Container sentry-self-hosted-clickhouse-1  Waiting
Container sentry-self-hosted-clickhouse-1  Waiting
Container sentry-self-hosted-clickhouse-1  Waiting
Container sentry-self-hosted-clickhouse-1  Waiting
Container sentry-self-hosted-clickhouse-1  Waiting
Container sentry-self-hosted-redis-1  Waiting
Container sentry-self-hosted-clickhouse-1  Waiting
Container sentry-self-hosted-clickhouse-1  Waiting
Container sentry-self-hosted-kafka-1  Waiting
Container sentry-self-hosted-kafka-1  Waiting
Container sentry-self-hosted-kafka-1  Waiting
Container sentry-self-hosted-kafka-1  Waiting
Container sentry-self-hosted-clickhouse-1  Waiting
Container sentry-self-hosted-kafka-1  Waiting
Container sentry-self-hosted-kafka-1  Waiting
Container sentry-self-hosted-redis-1  Waiting
Container sentry-self-hosted-kafka-1  Waiting
Container sentry-self-hosted-redis-1  Waiting
Container sentry-self-hosted-redis-1  Waiting
Container sentry-self-hosted-redis-1  Waiting
Container sentry-self-hosted-smtp-1  Started
Container sentry-self-hosted-symbolicator-1  Started
Container sentry-self-hosted-postgres-1  Started
Container sentry-self-hosted-redis-1  Healthy
Container sentry-self-hosted-redis-1  Healthy
Container sentry-self-hosted-kafka-1  Healthy
Container sentry-self-hosted-redis-1  Healthy
Container sentry-self-hosted-clickhouse-1  Healthy
Container sentry-self-hosted-clickhouse-1  Healthy
Container sentry-self-hosted-clickhouse-1  Healthy
Container sentry-self-hosted-clickhouse-1  Healthy
Container sentry-self-hosted-clickhouse-1  Healthy
Container sentry-self-hosted-clickhouse-1  Healthy
Container sentry-self-hosted-kafka-1  Healthy
Container sentry-self-hosted-clickhouse-1  Healthy
Container sentry-self-hosted-kafka-1  Healthy
Container sentry-self-hosted-redis-1  Healthy
Container sentry-self-hosted-clickhouse-1  Healthy
Container sentry-self-hosted-kafka-1  Healthy
Container sentry-self-hosted-kafka-1  Healthy
Container sentry-self-hosted-kafka-1  Healthy
Container sentry-self-hosted-redis-1  Healthy
Container sentry-self-hosted-redis-1  Healthy
Container sentry-self-hosted-kafka-1  Healthy
Container sentry-self-hosted-redis-1  Healthy
Container sentry-self-hosted-redis-1  Healthy
Container sentry-self-hosted-kafka-1  Healthy
Container sentry-self-hosted-snuba-consumer-1  Starting
Container sentry-self-hosted-snuba-subscription-consumer-transactions-1  Starting
Container sentry-self-hosted-snuba-sessions-consumer-1  Starting
Container sentry-self-hosted-snuba-transactions-consumer-1  Starting
Container sentry-self-hosted-snuba-api-1  Starting
Container sentry-self-hosted-snuba-outcomes-consumer-1  Starting
Container sentry-self-hosted-snuba-subscription-consumer-events-1  Starting
Container sentry-self-hosted-snuba-replacer-1  Starting
Container sentry-self-hosted-snuba-subscription-consumer-transactions-1  Started
Container sentry-self-hosted-snuba-transactions-consumer-1  Started
Container sentry-self-hosted-snuba-api-1  Started
Container sentry-self-hosted-snuba-consumer-1  Started
Container sentry-self-hosted-snuba-replacer-1  Started
Container sentry-self-hosted-snuba-outcomes-consumer-1  Started
Container sentry-self-hosted-snuba-sessions-consumer-1  Started
Container sentry-self-hosted-snuba-subscription-consumer-events-1  Started
Updating certificates in /etc/ssl/certs...
0 added, 0 removed; done.
Running hooks in /etc/ca-certificates/update.d...
done.
Installing additional dependencies...
WARNING: Running pip as the 'root' user can result in broken permissions and conflicting behaviour with the system package manager. It is recommended to use a virtual environment instead: https://pip.pypa.io/warnings/venv

13:25:15 [WARNING] sentry.utils.geo: Error opening GeoIP database: /geoip/GeoLite2-City.mmdb
13:25:15 [WARNING] sentry.utils.geo: Error opening GeoIP database in Rust: /geoip/GeoLite2-City.mmdb
13:25:17 [INFO] sentry.plugins.github: apps-not-configured
Running migrations for default
Operations to perform:
  Apply all migrations: admin, auth, contenttypes, nodestore, sentry, sessions, sites, social_auth
Running migrations:
  Applying sentry.0001_squashed_0200_release_indices... OK
  Applying contenttypes.0001_initial... OK
  Applying admin.0001_initial... OK
  Applying admin.0002_logentry_remove_auto_add... OK
  Applying admin.0003_logentry_add_action_flag_choices... OK
  Applying contenttypes.0002_remove_content_type_name... OK
  Applying auth.0001_initial... OK
  Applying auth.0002_alter_permission_name_max_length... OK
  Applying auth.0003_alter_user_email_max_length... OK
  Applying auth.0004_alter_user_username_opts... OK
  Applying auth.0005_alter_user_last_login_null... OK
  Applying auth.0006_require_contenttypes_0002... OK
  Applying auth.0007_alter_validators_add_error_messages... OK
  Applying auth.0008_alter_user_username_max_length... OK
  Applying auth.0009_alter_user_last_name_max_length... OK
  Applying auth.0010_alter_group_name_max_length... OK
  Applying auth.0011_update_proxy_permissions... OK
  Applying nodestore.0001_initial... OK
  Applying nodestore.0002_nodestore_no_dictfield... OK
  Applying sentry.0201_semver_package... OK
  Applying sentry.0202_org_slug_upper_idx... OK
  Applying sentry.0203_groupedmessage_status_index... OK
  Applying sentry.0204_use_project_team_for_team_key_transactions... OK
  Applying sentry.0205_semver_backfill... OK
  Applying sentry.0206_organization_require_email_verification_flag... OK
  Applying sentry.0207_release_adoption... OK
  Applying sentry.0208_add_team_scope... OK
  Applying sentry.0209_avatar_remove_file_fk... OK
  Applying sentry.0210_backfill_project_transaction_thresholds... OK
  Applying sentry.0211_add_artifact_count... OK
  Applying sentry.0212_appconnectbuilds... OK
  Applying sentry.0213_rule_project_status_owner_index... OK
  Applying sentry.0214_activity_project_type_datetime_idx... OK
  Applying sentry.0215_fix_state... OK
  Applying sentry.0216_cdc_setup_replication_index... OK
  Applying sentry.0217_debugfile_remove_project_fk... OK
  Applying sentry.0218_releasefile_remove_fks... OK
  Applying sentry.0219_exporteddatablob_remove_blob_fk... OK
  Applying sentry.0220_add_current_release_version_group_resolution... OK
  Applying sentry.0221_add_appconnect_upload_dates... OK
  Applying sentry.0222_add_datetime_index_to_auditlogentry... OK
  Applying sentry.0223_semver_backfill_2... OK
  Applying sentry.0224_has_sessions_flag... OK
  Applying sentry.0225_latest_appconnect_builds_check... OK
  Applying sentry.0226_add_visits... OK
  Applying sentry.0227_backfill_visits... OK
  Applying sentry.0228_update_auditlog_index_with_entry... OK
  Applying sentry.0229_drop_jiratenant... OK
  Applying sentry.0230_sentry_app_config_jsonfield... OK
  Applying sentry.0231_alert_rule_comparison_delta... OK
  Applying sentry.0232_backfill_missed_semver_releases... OK
  Applying sentry.0233_recreate_subscriptions_in_snuba... OK
  Applying sentry.0234_grouphistory... OK
  Applying sentry.0235_add_metricskeyindexer_table... OK
  Applying sentry.0236_remove_legacy_key_transactions... OK
  Applying sentry.0237_recreate_subscriptions_in_snuba... OK
  Applying sentry.0238_remove_scheduleddeletion_aborted... OK
  Applying sentry.0239_drop_scheduleddeletion_aborted... OK
  Applying sentry.0240_grouphistory_index... OK
  Applying sentry.0241_grouphistory_null_actor... OK
  Applying sentry.0242_delete_removed_plugin_data... OK
  Applying sentry.0243_delete_visualstudio_repo_data... OK
  Applying sentry.0244_organization_and_integration_foreign_keys... OK
  Applying sentry.0245_delete_itunes_credentials... OK
  Applying sentry.0246_incident_snapshots_remove_fks... OK
  Applying sentry.0247_add_color_column... OK
  Applying sentry.0248_add_popularity_column... OK
  Applying sentry.0249_add_avatar_type_back... OK
  Applying sentry.0250_backfill_popularity... OK
  Applying sentry.0251_sentryappavatar_sentryapp_not_unique... OK
  Applying sentry.0252_code_mapping_cascade_delete... OK
  Applying sentry.0253_add_widget_type... OK
  Applying sentry.0254_org_integration_grace_period_end... OK
  Applying sentry.0255_delete_code_mappings_with_no_integration... OK
  Applying sentry.0256_create_docintegration_table... OK
  Applying sentry.0257_add_target_id_and_type_to_integrationfeature... OK
  Applying sentry.0258_create_docintegrationavatar_table... OK
  Applying sentry.0259_delete_codeowners_and_code_mappings_with_no_integration... OK
  Applying sentry.0260_backfill_integrationfeature... OK
  Applying sentry.0261_prepare_remove_sentry_app_column... OK
  Applying sentry.0262_drop_sentry_app_from_integrationfeature... OK
  Applying sentry.0263_remove_not_null_integrationfeature... OK
  Applying sentry.0264_use_booleanfield_docintegration... OK
  Applying sentry.0265_add_userrole... OK
  Applying sentry.0266_add_dashboard_widget_detail_field... OK
  Applying sentry.0267_sentry_release_version_btree... OK
  Applying sentry.0268_rename_issue_widget_query_fields... OK
  Applying sentry.0269_alertrule_remove_unique_name... OK
  Applying sentry.0270_group_history_project_date_added_index... OK
  Applying sentry.0271_add_codeowners_auto_sync_setting... OK
  Applying sentry.0272_seq_scan_indexes... OK
  Applying sentry.0273_fix_grouplink_seqscans... OK
  Applying sentry.0274_add_dashboardwidgetquery_columns_aggregates... OK
  Applying sentry.0275_rule_fire_history... OK
  Applying sentry.0276_rulefirehistory_date_added_index... OK
  Applying sentry.0277_backfill_dashboard_widget_query_columns_aggregates... OK
  Applying sentry.0278_backfill_codeowners_auto_sync_setting... OK
  Applying sentry.0279_add_limit_dashboard_widget... OK
  Applying sentry.0280_extend_commit_author_email_length... OK
  Applying sentry.0281_add_new_indexer_table... OK
  Applying sentry.0282_add_field_aliases_dashboard_widget_query... OK
  Applying sentry.0283_extend_externalissue_key... OK
  Applying sentry.0284_metrics_indexer_alter_seq... OK
  Applying sentry.0285_add_organization_member_team_role... OK
  Applying sentry.0286_backfill_alertrule_organization... OK
  Applying sentry.0287_backfill_snubaquery_environment... OK
  Applying sentry.0288_fix_savedsearch_state... OK
  Applying sentry.0289_dashboardwidgetquery_convert_orderby_to_field... OK
  Applying sentry.0290_fix_project_has_releases... OK
  Applying sessions.0001_initial... OK
  Applying sites.0001_initial... OK
  Applying sites.0002_alter_domain_unique... OK
  Applying social_auth.0001_initial... OK
13:25:56 [WARNING] sentry: Cannot initiate onboarding for organization (1) due to missing owners
Created internal Sentry project (slug=internal, id=1)
Creating missing DSNs
Correcting Group.num_comments counter

Did not prompt for user creation due to non-interactive shell.
Run the following command to create one yourself (recommended):

  docker compose run --rm web createuser


▶ Migrating file storage ...
Unable to find image 'alpine:latest' locally
latest: Pulling from library/alpine
df9b9388f04a: Already exists
Digest: sha256:4edbd2beb5f78b1014028f4fbb99f3237d9561100b6881aabbf5acce2c4f9454
Status: Downloaded newer image for alpine:latest

▶ Setting up GeoIP integration ...
Setting up IP address geolocation ...
Installing (empty) IP address geolocation database ... done.
IP address geolocation is not configured for updates.
See https://develop.sentry.dev/self-hosted/geolocation/ for instructions.
Error setting up IP address geolocation.


-----------------------------------------------------------------

You're all done! Run the following command to get Sentry running:

  docker compose up -d

-----------------------------------------------------------------
````

#### Waring!!

You must run `docker/sentry/install.sh` on WSL2 rather than WSL1.
Otherwise, you should face to the following error.

````sh
$ ./install.sh --skip-user-prompt
▶ Parsing command line ...

▶ Initializing Docker Compose ...

▶ Setting up error handling ...

▶ Checking for latest commit ...
skipped

▶ Checking minimum requirements ...
An error occurred, caught SIGERR on line 8
Cleaning up...
````

### 2. Start Sentry

````sh
$ cd docker/sentry
$ docker compose up -d
````

Sentry takes time to finish launching.

If you run install.sh with `--skip-user-prompt`, you must run the following command to create user.

````sh
$ cd docker/sentry
$ docker compose run --rm web createuser
[+] Running 16/0
 ⠿ Container sentry-self-hosted-redis-1                                     Running                                0.0s
 ⠿ Container sentry-self-hosted-zookeeper-1                                 Running                                0.0s
 ⠿ Container sentry-self-hosted-smtp-1                                      Running                                0.0s
 ⠿ Container sentry-self-hosted-clickhouse-1                                Running                                0.0s
 ⠿ Container sentry-self-hosted-memcached-1                                 Running                                0.0s
 ⠿ Container sentry-self-hosted-postgres-1                                  Running                                0.0s
 ⠿ Container sentry-self-hosted-kafka-1                                     Running                                0.0s
 ⠿ Container sentry-self-hosted-symbolicator-1                              Running                                0.0s
 ⠿ Container sentry-self-hosted-snuba-replacer-1                            Running                                0.0s
 ⠿ Container sentry-self-hosted-snuba-subscription-consumer-events-1        Running                                0.0s
 ⠿ Container sentry-self-hosted-snuba-sessions-consumer-1                   Running                                0.0s
 ⠿ Container sentry-self-hosted-snuba-api-1                                 Running                                0.0s
 ⠿ Container sentry-self-hosted-snuba-consumer-1                            Running                                0.0s
 ⠿ Container sentry-self-hosted-snuba-outcomes-consumer-1                   Running                                0.0s
 ⠿ Container sentry-self-hosted-snuba-subscription-consumer-transactions-1  Running                                0.0s
 ⠿ Container sentry-self-hosted-snuba-transactions-consumer-1               Running                                0.0s
[+] Running 7/7
 ⠿ Container sentry-self-hosted-geoipupdate-1  Started                                                             0.3s
 ⠿ Container sentry-self-hosted-zookeeper-1    Healthy                                                             0.5s
 ⠿ Container sentry-self-hosted-kafka-1        Healthy                                                             1.6s
 ⠿ Container sentry-self-hosted-redis-1        Healthy                                                             1.6s
 ⠿ Container sentry-self-hosted-clickhouse-1   Healthy                                                             0.5s
 ⠿ Container sentry-self-hosted-postgres-1     Healthy                                                             0.5s
 ⠿ Container sentry-self-hosted-web-1          Healthy                                                             0.5s
Updating certificates in /etc/ssl/certs...
0 added, 0 removed; done.
Running hooks in /etc/ca-certificates/update.d...
done.
13:39:00 [INFO] sentry.plugins.github: apps-not-configured
Email: admin@gmail.com
Password:
Repeat for confirmation:
Should this user be a superuser? [y/N]: y
Added to organization: sentry
Should this user have Super Admin role? (This grants them all permissions available) [y/N]: y
User created: admin@gmail.com
````

### 3. Access to Sentry via browse

You can see Sentry's port number via `docker ps -a`.
Default port could be 9000.

So you can see

<img src="images/login.png?raw=true" title="Login"/>

And you will see after login

<img src="images/welcome.png?raw=true" title="Welcome"/>

<img src="images/dashboard.png?raw=true" title="Dashboard"/>

### 4. Create new C# project on Sentry

You can create new project from `Projects` menu from left sidebar and select `.NET` on platfrom list.
Then click `Create Project`.

<img src="images/selectproject.png?raw=true" title="Select Project Type"/>

Sentry tell you how to use Senty API from .NET application.

<img src="images/howto.png?raw=true" title="How To Usage"/>

## Result

A sample program is division program.
User can pass left and right operand.
So if pass 0 as right operand, program will crash and Sentry send report to local server.


````bat
> cd sources\Demo
> dotnet run -c Release -- 10 0
  Debug: Logging enabled with ConsoleDiagnosticLogger and min level: Debug
  Debug: Initializing Hub for Dsn: 'http://7faf7bce38604a47a2a25ff456572836@localhost:9000/2'.
  Debug: Using 'GzipBufferedRequestBodyHandler' body compression strategy with level Optimal.
  Debug: New scope pushed.
  Debug: Registering integration: 'AutoSessionTrackingIntegration'.
  Debug: Registering integration: 'AppDomainUnhandledExceptionIntegration'.
  Debug: Registering integration: 'AppDomainProcessExitIntegration'.
  Debug: Registering integration: 'TaskUnobservedTaskExceptionIntegration'.
  Debug: Registering integration: 'SentryDiagnosticListenerIntegration'.
  Debug: Failed to end session because there is none active.
   Info: Capturing event.
  Debug: Running processor on exception: Attempted to divide by zero.
  Debug: Creating SentryStackTrace. isCurrentStackTrace: False.
  Debug: Running main event processor on: Event 66d3ab4f2ab045b3b0123044e06f1538
   Info: Envelope queued up: '66d3ab4f2ab045b3b0123044e06f1538'
   Info: Disposing the Hub.
  Debug: Tracking depth: 1.
  Debug: Envelope 66d3ab4f2ab045b3b0123044e06f1538 handed off to transport. #1 in queue.
  Debug: Envelope '66d3ab4f2ab045b3b0123044e06f1538' sent successfully. Payload:
{"sdk":{"name":"sentry.dotnet","version":"3.17.1"},"event_id":"66d3ab4f2ab045b3b0123044e06f1538"}
{"type":"event","length":3227}
{"modules":{"System.Private.CoreLib":"6.0.0.0","Demo":"1.0.0.0","System.Runtime":"6.0.0.0","Sentry":"3.17.1.0","System.Net.Primitives":"6.0.0.0","System.IO.Compression":"6.0.0.0","System.Console":"6.0.0.0","System.Collections.Concurrent":"6.0.0.0","System.Threading":"6.0.0.0","System.Text.Encoding.Extensions":"6.0.0.0","System.Private.Uri":"6.0.0.0","System.Diagnostics.Process":"6.0.0.0","System.Net.Http":"6.0.0.0","System.ComponentModel.Primitives":"6.0.0.0","System.Diagnostics.Tracing":"6.0.0.0","Microsoft.Win32.Primitives":"6.0.0.0","System.Diagnostics.DiagnosticSource":"6.0.0.0","System.Runtime.InteropServices":"6.0.0.0","System.Net.Security":"6.0.0.0","System.Security.Cryptography.X509Certificates":"6.0.0.0","System.Collections":"6.0.0.0","System.Linq":"6.0.0.0","System.Diagnostics.StackTrace":"6.0.0.0","System.Reflection.Metadata":"6.0.0.0","System.Collections.Immutable":"6.0.0.0","System.Runtime.InteropServices.RuntimeInformation":"6.0.0.0","System.Memory":"6.0.0.0","System.Text.RegularExpressions":"6.0.0.0","System.Reflection.Emit.ILGeneration":"6.0.0.0","System.Reflection.Emit.Lightweight":"6.0.0.0","System.Reflection.Primitives":"6.0.0.0","System.Linq.Expressions":"6.0.0.0","System.Threading.Thread":"6.0.0.0","System.Threading.ThreadPool":"6.0.0.0"},"event_id":"66d3ab4f2ab045b3b0123044e06f1538","timestamp":"2022-05-21T14:05:17.2154715+00:00","platform":"csharp","release":"Demo@1.0.0","exception":{"values":[{"type":"System.DivideByZeroException","value":"Attempted to divide by zero.","module":"System.Private.CoreLib, Version=6.0.0.0, Culture=neutral, PublicKeyToken=7cec85d7bea7798e","thread_id":1,"stacktrace":{"frames":[{"filename":"D:\\Works\\OpenSource\\Demo\\Sentry\\01_Simple\\sources\\Demo\\Program.cs","function":"void Program.Main(string[] args)","lineno":23,"colno":17,"in_app":true,"package":"Demo, Version=1.0.0.0, Culture=neutral, PublicKeyToken=null","instruction_offset":37}]},"mechanism":{"type":"AppDomain.UnhandledException","handled":false}}]},"level":"error","request":{},"contexts":{"os":{"type":"os","raw_description":"Microsoft Windows 10.0.19044"},"runtime":{"type":"runtime","name":".NET","version":"6.0.5","raw_description":".NET 6.0.5"},"device":{"type":"device","timezone":"Tokyo Standard Time","timezone_display_name":"(UTC\u002B09:00) \u5927\u962A\u3001\u672D\u5E4C\u3001\u6771\u4EAC","boot_time":"2022-05-17T04:06:00.1351675+00:00"},"Current Culture":{"name":"ja-JP","display_name":"\u65E5\u672C\u8A9E (\u65E5\u672C)","calendar":"GregorianCalendar"},"Memory Info":{"allocated_bytes":484296,"high_memory_load_threshold_bytes":61668871372,"total_available_memory_bytes":68520968192,"finalization_pending_count":0,"compacted":false,"concurrent":false,"pause_durations":[0,0]},"Dynamic Code":{"Compiled":true,"Supported":true},"app":{"type":"app","app_start_time":"2022-05-21T14:05:16.6546488+00:00"},"ThreadPool Info":{"min_worker_threads":12,"min_completion_port_threads":12,"max_worker_threads":32767,"max_completion_port_threads":1000,"available_worker_threads":32767,"available_completion_port_threads":1000}},"user":{},"environment":"production","sdk":{"packages":[{"name":"nuget:sentry.dotnet","version":"3.17.1"}],"name":"sentry.dotnet","version":"3.17.1"}}

  Debug: Signaling flush completed.
  Debug: Successfully flushed all events up to call to FlushAsync.
Unhandled exception. System.DivideByZeroException: Attempted to divide by zero.
   at Demo.Program.Main(String[] args) in D:\Works\OpenSource\Demo\Sentry\01_Simple\sources\Demo\Program.cs:line 23
   Info: Disposing the Hub.
````

And you can find out that Senty is updated.

<img src="images/issues.png?raw=true" title="Issues"/>

<img src="images/details.png?raw=true" title="Details"/>