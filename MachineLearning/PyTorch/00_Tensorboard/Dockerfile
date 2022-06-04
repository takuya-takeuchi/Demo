FROM python:3.8
RUN pip install tensorflow
WORKDIR /logs
ENTRYPOINT ["tensorboard", "--logdir", "/logs", "--host", "0.0.0.0"]   