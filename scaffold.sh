#!/bin/bash

sha=0
previous_sha=0
pwd=`pwd`

update_sha()
{
    sha=`ls -lR . | sha1sum`
}

build () {
    cd $pwd
    make
    cd tests
    make run
    cd $pwd
    echo "[Scaffold: Standby]"
}

changed () {
    echo "[Scaffold: Files changed - Building]"
    build
    previous_sha=$sha
}

compare () {
    update_sha
    if [[ $sha != $previous_sha ]] ; then changed; fi
}

run () {
    while true; do

        compare

        read -s -t 3 && (
            echo "[Scaffold: Forced Update]"
            build
        )

    done
}

echo "[Scaffold: Init]"
echo "[Scaffold: Standby]"
run
