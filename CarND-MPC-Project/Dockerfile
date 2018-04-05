# Dockerfile containing software for Control C++ quizzes
FROM ubuntu:xenial

WORKDIR /quizzes

RUN apt-get update && apt-get install -y \
    build-essential \
    gcc \
    g++ \
    gfortran \
    cmake \
    pkg-config \
    unzip \
    git \
    wget \
    cppad \
    python-matplotlib \
    python2.7-dev \
    libuv1-dev \
    libssl-dev \
    make

RUN git clone https://github.com/uWebSockets/uWebSockets
RUN cd uWebSockets && git checkout e94b6e1 && mkdir build && cd build \
    && cmake .. && make && make install
# RUN cd ../..
RUN ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
RUN rm -r uWebSockets

ADD install_ipopt.sh .

RUN wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip
RUN bash install_ipopt.sh Ipopt-3.12.7

RUN rm -rf /var/lib/apt/lists/*
