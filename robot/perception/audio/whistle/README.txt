Steps towards running on the robot.

1. Download pyalsaaudio.
https://launchpad.net/ubuntu/+archive/primary/+files/pyalsaaudio_0.7.orig.tar.gz


2.  Compile pyalsaaudio manually,
    to work around robots not having internet access.

    Something like:

cd ${RUNSWIFT_CHECKOUT_DIR}/robot/perception/audio/whistle/pyalsaaudio-0.7/
export CC=${CTC_DIR}/sysroot/usr/i686-pc-linux-gnu/gcc-bin/4.5.3/gcc;
export LD_LIBRARY_PATH=${CTC_DIR}/sysroot/usr/lib;
${CTC_DIR}/sysroot/usr/i686-pc-linux-gnu/gcc-bin/4.5.3/gcc alsaaudio.c -I ${CTC_DIR}/sysroot/usr/include/python2.7 --sysroot=${CTC_DIR}/sysroot -shared -o alsaaudio.so -lasound


3. Copy to behaviours on the robot.
