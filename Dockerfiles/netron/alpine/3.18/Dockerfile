FROM alpine:3.18
LABEL maintainer "Takuya Takeuchi <takuya.takeuchi.dev@gmail.com>"

ARG COMPONENT_VERSION

RUN apk update && apk add --no-cache --repository http://dl-cdn.alpinelinux.org/alpine/v3.18/main \
    ca-certificates \
 && rm -rf /var/cache/apk/*
RUN /usr/sbin/update-ca-certificates

# 4. custom tools
RUN apk --update --no-cache --repository http://dl-cdn.alpinelinux.org/alpine/edge/main/ add \
    python3 \
    py3-pip \
    su-exec \
 && python3 -m pip install --upgrade pip setuptools --break-system-packages \
 && rm -r /root/.cache

RUN python3 -m pip install --break-system-packages \
    netron==${COMPONENT_VERSION}

ADD entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]