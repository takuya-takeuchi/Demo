# Refer: https://github.com/ursetto/vscode-sysroot/blob/main/Dockerfile
FROM ubuntu:latest AS crosstool

RUN apt-get update
RUN apt-get install -y gcc g++ gperf bison flex texinfo help2man make libncurses5-dev \
python3-dev autoconf automake libtool libtool-bin gawk wget bzip2 xz-utils unzip \
patch rsync meson ninja-build

# Install crosstool-ng
ENV PKG=crosstool-ng-1.27.0
RUN wget https://github.com/crosstool-ng/crosstool-ng/releases/download/$PKG/$PKG.tar.bz2
RUN tar -xjf $PKG.tar.bz2
RUN cd $PKG && ./configure --prefix=/$PKG/out && make && make install
ENV PATH=$PATH:/$PKG/out/bin
ENV CT_ALLOW_BUILD_AS_ROOT_SURE=y
WORKDIR /src

FROM crosstool AS sysroot

COPY x86_64-gcc-8.5.0-glibc-2.28.config /src/.config
RUN ct-ng build
RUN wget -O - https://github.com/NixOS/patchelf/releases/download/0.18.0/patchelf-0.18.0-x86_64.tar.gz | tar zxv -C x86_64-linux-gnu/x86_64-linux-gnu/sysroot/usr ./bin/patchelf
RUN tar zcf vscode-sysroot-x86_64-linux-gnu.tgz -C x86_64-linux-gnu/x86_64-linux-gnu --exclude '*.a' sysroot