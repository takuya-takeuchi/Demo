docker build -t tensorboard .
mkdir %CD%\logs
docker run -it --detach --rm -p 10000:6006 -v %CD%\logs:/logs tensorboard