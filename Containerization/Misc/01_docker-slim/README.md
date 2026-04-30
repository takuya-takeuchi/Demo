# DockerSlim (SlimToolkit a.k.a Slim)

## Abstracts

* Slim docker image

## Dependencies

* [Slim](https://github.com/slimtoolkit/slim)
  * 1.41.8
  * Apache-2.0 license

##### Test Data

* [Female](https://sipi.usc.edu/database/download.php?vol=misc&img=4.1.01)
  * [4.1.01.tiff](./4.1.01.tiff)
* [Peppers](https://sipi.usc.edu/database/download.php?vol=misc&img=4.2.07)
  * [4.2.07.tiff](./4.2.07.tiff)

## How to install?

````bash
$ sudo -E mkdir -p /usr/local/bin
$ curl -sL https://raw.githubusercontent.com/slimtoolkit/slim/master/scripts/install-slim.sh | sudo -E bash -
$ slim --version
mint version linux/amd64|Aurora|1.41.8-1-g4b0d7e91|4b0d7e915ef33a98a2b32ccc10c8558086b27fe4|2025-12-12_05:28:51AM
````

## How to use?

#### 1. Build image

````bash
$ docker build -t docker-slim-exp-app .
````

#### 2. Check size

````bash
$ docker images | grep 24.04
ubuntu                                      24.04                                  0b1ebe5dd426   2 weeks ago       78.1MB
$ docker images | grep docker-slim-exp-app
docker-slim-exp-app                         latest                                 ec6614b6f9d1   46 seconds ago    164MB
````

#### 3. Apply Slim

````bash
$ slim build --continue-after 10 --http-probe=false -entrypoint "/app/Demo /app/4.2.07.tiff" --target docker-slim-exp-app
cmd=slim state=started
cmd=slim info=cmd.input.params image-build-engine='internal' target.type='image' target.image='docker-slim-exp-app' continue.mode='timeout' rt.as.user='true' keep.perms='true' tags='' 
cmd=slim state=image.inspection.start
cmd=slim info=image id='sha256:cec1ba1f31261dfdd78dd69c3b167265e5bac88c6ee3dadbb7885b6e45d5748a' size.bytes='163989939' size.human='164 MB' 
cmd=slim info=image.stack index='0' name='docker-slim-exp-app:latest' id='sha256:cec1ba1f31261dfdd78dd69c3b167265e5bac88c6ee3dadbb7885b6e45d5748a' 
cmd=slim state=image.inspection.done
cmd=slim state=container.inspection.start
cmd=slim info=sensor volume='mint-sensor.1.41.8-1-g4b0d7e91' location='/usr/local/bin/mint-sensor' filemode='-rwxr-xr-x' version='linux/amd64|Aurora|1.41.8-1-g4b0d7e91|4b0d7e915ef33a98a2b32ccc10c8558086b27fe4|2025-12-12_05:28:51AM' 
cmd=slim info=container id='75790773490fb7237215739b158b40d83f719c0f3a8da45b99f295c6fb2609b7' status='created' name='mintk_1117572_20260430102412' 
cmd=slim info=container status='running' name='mintk_1117572_20260430102412' id='75790773490fb7237215739b158b40d83f719c0f3a8da45b99f295c6fb2609b7' 
cmd=slim info=container message='obtained IP address' ip='172.17.0.2' 
cmd=slim info=cmd.startmonitor status='sent' 
cmd=slim info=event.startmonitor.done status='received' 
cmd=slim info=container name='mintk_1117572_20260430102412' id='75790773490fb7237215739b158b40d83f719c0f3a8da45b99f295c6fb2609b7' target.port.list='' target.port.info='' message='YOU CAN USE THESE PORTS TO INTERACT WITH THE CONTAINER' 
cmd=slim info=continue.after mode='timeout' message='no input required, execution will resume after the timeout' 
cmd=slim prompt='waiting for the target container (10 seconds)'
cmd=slim info=event message='done waiting for the target container' 
cmd=slim state=container.inspection.finishing
cmd=slim state=container.inspection.artifact.processing
cmd=slim state=container.inspection.done
cmd=slim state=building message="building optimized image" engine=internal 
cmd=slim state=completed
cmd=slim info=results status='MINIFIED' by='7.24X' size.original='164 MB' size.optimized='23 MB' 
cmd=slim info=results image-build-engine='internal' image.name='docker-slim-exp-app.slim' image.size='23 MB' image.id='sha256:ce8bbb25e49cb31b565dc8af5b02576d2f1f96340b53c996c03997dec0531121' image.digest='sha256:ea149ef79e0d2dba265130c05e897c04e1cbfcbc7482e630d17eb4b96a3eccbb' has.data='true' 
cmd=slim info=results artifacts.location='/tmp/mint-state/.mint-state/images/cec1ba1f31261dfdd78dd69c3b167265e5bac88c6ee3dadbb7885b6e45d5748a/artifacts' 
cmd=slim info=results artifacts.report='creport.json' 
cmd=slim info=results artifacts.dockerfile.reversed='Dockerfile.reversed' 
cmd=slim info=results artifacts.seccomp='docker-slim-exp-app-seccomp.json' 
cmd=slim info=results artifacts.apparmor='docker-slim-exp-app-apparmor-profile' 
cmd=slim state=done
cmd=slim info=commands message='use the xray command to learn more about the optimize image' 
cmd=slim info=report file='slim.report.json' 
cmd=slim info=version status='OUTDATED' local='1.41.8-1-g4b0d7e91' current='1.41.8' 
cmd=slim message='Your version of MinToolkit is out of date! Use `mint update` to get the latest version.'
app='mint' message='GitHub Discussions' info='https://github.com/mintoolkit/mint/discussions'
app='mint' message='Join the CNCF Slack channel to ask questions or to share your feedback' info='https://cloud-native.slack.com/archives/C059QP1RH1S'
app='mint' message='Join the Discord server to ask questions or to share your feedback' info='https://discord.gg/fAvq4ruKsG'
````

#### 4. Check size

````bash
$ docker images | grep docker-slim-exp-app.slim 
docker-slim-exp-app.slim                    latest                                 199c442b1327   10 seconds ago   22.7MB
````

#### 5. Run

````bash
$ docker run --rm --name docker-slim-exp-app.slim -it docker-slim-exp-app.slim /app/Demo /app/4.2.07.tiff
[Info] Starting loop
[Info] /app/4.2.07.tiff (w: 512, h: 512)
[Info] /app/4.2.07.tiff (w: 512, h: 512)
[Info] /app/4.2.07.tiff (w: 512, h: 512)
````

#### 6. Parameter tuning

App crashes if specify image file that doesn't exist.

````bash
$ docker run --rm --name docker-slim-exp-app.slim -it docker-slim-exp-app.slim /app/Demo /app/4.1.01.tiff
[Info] Starting loop
[ WARN:0@0.002] global loadsave.cpp:278 findDecoder imread_('/app/4.1.01.tiff'): can't open/read file: check file path/integrity
[Error] Failed to read image: /app/4.1.01.tiff
````

Slim remove unused image file when analysis docker image.
Therefore, we can give instructions to keep file.

````bash
$ slim build --continue-after 10 --http-probe=false -entrypoint "/app/Demo /app/4.2.07.tiff" --target docker-slim-exp-app --include-path /app/4.1.01.tiff

$ docker run --rm --name docker-slim-exp-app.slim -it docker-slim-exp-app.slim /app/Demo /app/4.1.01.tiff
[Info] Starting loop
[Info] /app/4.1.01.tiff (w: 256, h: 256)
[Info] /app/4.1.01.tiff (w: 256, h: 256)
````