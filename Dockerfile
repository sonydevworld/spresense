FROM phusion/baseimage:0.11

# Use baseimage-docker's init system.
CMD ["/sbin/my_init"]

RUN apt-get update
RUN apt-get install -y git gperf libncurses5-dev flex bison genromfs vim-common gcc-arm-none-eabi wget bzip2 gcc make python3
RUN apt-get install -y pkg-config autoconf automake cmake --install-recommends

RUN git clone https://bitbucket.org/nuttx/tools.git
RUN cd tools/kconfig-frontends/ && ./configure --disable-shared && make && make install
RUN ldconfig

# Clean up APT when done.
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
