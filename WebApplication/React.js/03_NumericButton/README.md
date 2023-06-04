# NumericButton

## Abstracts

* Use `useState` hook
* Use css grid layout to align button and text
* Deploy app to nginx by docker

## Requirements

* React
  * 18.2.0

## How to usage?

````cmd
$ npm install
````

Then, run app on local server

````cmd
$ npm start
````

<img src="./images/image.gif" />

## How to bundle?

````cmd
$ npm run build 
````

## How to deploy?

Here is usage of nginx by docker.
You must bundle before deploy.

````cmd
$ docker build -t react-demo .
$ docker run --rm -d -p 8080:80 react-demo
````