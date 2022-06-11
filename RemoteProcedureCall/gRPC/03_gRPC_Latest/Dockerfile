FROM ubuntu:18.04
LABEL maintainer "Takuya Takeuchi <takuya.takeuchi.dev@gmail.com>"

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    git \
    cmake \
 && apt-get clean && rm -rf /var/lib/apt/lists/*