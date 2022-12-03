FROM ubuntu:16.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    apt-transport-https \
    build-essential \
    autoconf \
    automake \
    gosu \
    sudo \
    wget \
    curl \
    git \
    unzip \
    bash-completion

# Install 32bit lib
RUN apt-get -y install lib32stdc++6 lib32z1

# Environment variables
ENV JAVA_HOME /usr/lib/jvm/java-8-openjdk-amd64

# Install Java8
RUN apt-get install -y software-properties-common curl \
 && add-apt-repository -y ppa:openjdk-r/ppa
RUN apt-get update && apt-get install -y \
    openjdk-8-jdk \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

ENV CMAKE_VERSION 3.17.2
RUN wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-Linux-x86_64.sh && chmod +x cmake-${CMAKE_VERSION}-Linux-x86_64.sh
RUN echo y | ./cmake-${CMAKE_VERSION}-Linux-x86_64.sh
RUN mv cmake-${CMAKE_VERSION}-Linux-x86_64 /opt && echo y | ln -s /opt/cmake-${CMAKE_VERSION}-Linux-x86_64/bin/* /usr/bin

# Environment variables
# OpenCV requires to use 25.2.5.
# https://github.com/opencv/opencv/issues/8460
ENV ANDROID_COMPILE_SDK="24"
ENV ANDROID_BUILD_TOOLS="26.0.1"
ENV ANDROID_SDK_TOOLS_REV="r25.2.5"
ENV ANDROID_CMAKE_REV="3.6.411459"
ENV ANDROID_CMAKE_REV_3_10="3.10.2.49884404"
ENV ANDROID_HOME=/usr/local/android-sdk-linux
ENV ANDROID_NDK=/usr/local/android-sdk-linux/ndk-bundle

ENV PATH $ANDROID_HOME/platform-tools:$ANDROID_NDK:$ANDROID_HOME/tools/bin:$PATH

ARG SDK_TOOL_VERSION=tools_${ANDROID_SDK_TOOLS_REV}-linux

# update Android NDK version
WORKDIR $ANDROID_HOME
RUN wget https://dl.google.com/android/repository/android-ndk-r16b-linux-x86_64.zip
RUN unzip android-ndk-r16b-linux-x86_64.zip
RUN rm android-ndk-r16b-linux-x86_64.zip \
 && mv android-ndk-r16b ndk-bundle

# install android sdk tools
WORKDIR $ANDROID_HOME
RUN wget "https://dl.google.com/android/repository/${SDK_TOOL_VERSION}.zip" \
 && unzip -d $ANDROID_HOME $SDK_TOOL_VERSION.zip \
 && rm -rf $SDK_TOOL_VERSION.zip \
 && mkdir -p $HOME/.android \
 && touch ~/.android/repositories.cfg

# install tools
# RUN yes | sdkmanager --licenses > /dev/null \ # from 25.3.0,  support --licenses
# RUN yes | sdkmanager 'tools' \
#  && yes | sdkmanager 'platform-tools' \
#  && yes | sdkmanager 'build-tools;'$ANDROID_BUILD_TOOLS \
#  && yes | sdkmanager 'platforms;android-'$ANDROID_COMPILE_SDK \
#  && yes | sdkmanager 'platforms;android-28' \
#  && yes | sdkmanager 'extras;android;m2repository' \
#  && yes | sdkmanager 'extras;google;google_play_services' \
#  && yes | sdkmanager 'extras;google;m2repository'
RUN yes | sdkmanager 'build-tools;'$ANDROID_BUILD_TOOLS

