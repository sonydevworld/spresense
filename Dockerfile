FROM phusion/baseimage:0.10.1

# Use baseimage-docker's init system.
CMD ["/sbin/my_init"]


RUN apt-get update
RUN apt-get install -y gperf libncurses5-dev flex bison genromfs vim-common gcc-arm-none-eabi wget bzip2 gcc make python3

RUN wget http://ymorin.is-a-geek.org/download/kconfig-frontends/kconfig-frontends-3.10.0.0.tar.bz2
RUN tar -xvf kconfig-frontends-3.10.0.0.tar.bz2
RUN cd kconfig-frontends-3.10.0.0 && ./configure && make && make install && ldconfig

# Clean up APT when done.
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
