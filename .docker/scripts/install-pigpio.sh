#!/usr/bin/env bash

cd /tmp && \
wget https://github.com/joan2937/pigpio/archive/master.zip && \
unzip -qq master.zip && \
cd pigpio-master && \
sed -i 's,ldconfig,,' Makefile && \
make && \
sudo make install && \
rm -rf /tmp/*
