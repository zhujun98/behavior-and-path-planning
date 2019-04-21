#!/bin/bash

set -x
set -e

TP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"/../third_party

BOOST_VERSION=1.67.0
BOOST_VERSION_UNDERSCORE=1_67_0

# Download and compile boost if it isn't already present.
if [ ! -d $TP_DIR/boost ]; then
    BOOST_DOWNLOAD=boost_${BOOST_VERSION_UNDERSCORE}.tar.gz
    wget https://dl.bintray.com/boostorg/release/${BOOST_VERSION}/source/${BOOST_DOWNLOAD}

    BOOST_BUILD_DIR=${TP_DIR}/build/
    BOOST_INSTALL_DIR=${TP_DIR}/boost/
    if [ ! -d ${BOOST_BUILD_DIR} ]; then
        mkdir ${BOOST_BUILD_DIR}
    fi
    tar -xzf ${BOOST_DOWNLOAD} -C ${BOOST_BUILD_DIR}
    rm -rf ${BOOST_DOWNLOAD}

    # Compile boost.
    pushd ${BOOST_BUILD_DIR}
        pushd boost_${BOOST_VERSION_UNDERSCORE}
            ./bootstrap.sh
            ./b2 -j 4 variant=release --prefix=${BOOST_INSTALL_DIR} --with-log install > /dev/null
        popd
    popd
fi
