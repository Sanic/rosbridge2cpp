#!/bin/bash
wget https://github.com/mongodb/libbson/releases/download/1.5.3/libbson-1.5.3.tar.gz
tar xzf libbson-1.5.3.tar.gz
pushd libbson-1.5.3;
./configure --prefix=$HOME/deps/libbson && make -j$(grep -c ^processor /proc/cpuinfo) && make install;
popd;
