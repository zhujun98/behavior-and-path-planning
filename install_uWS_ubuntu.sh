#!/bin/bash

sudo apt-get install libuv1-dev libssl-dev libz-dev
git clone --single-branch -b v0.14.8 https://github.com/uWebSockets/uWebSockets

pushd uWebSockets
make
sudo make install

sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so

popd
sudo rm -r uWebSockets