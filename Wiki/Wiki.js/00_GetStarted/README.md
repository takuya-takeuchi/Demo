# Wiki.js Get Started

## Abstracts

* Start Wiki.js by docker-compose

## Dependencies

* [docker-ce](https://github.com/docker/docker-ce)
  * Apache-2.0 license
* [docker-ce-cli](https://github.com/docker/cli)
  * Apache-2.0 license
* [containerd.io](https://github.com/containerd/containerd)
  * Apache-2.0 license
* [Wiki.js](https://github.com/Requarks/wiki)
  * GNU Affero General Public License

## How to use?

Mount host machine directory to store wiki.js data.

### sqlite

Use sqlite as storage.

````sh
$ cd docker/sqlite
$ mkdir data
$ chmod 777 data
$ docker compose up -d
````

After running containers, you can access Wiki.js http://host-ip:3000.

<img src="./images/setup.png" />