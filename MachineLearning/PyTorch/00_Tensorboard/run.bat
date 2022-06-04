docker build -t tensorboard .
mkdir $PWD\logs
docker run -it --detach --rm -p 10000:6006 -v $PWD/logs:/logs tensorboard