#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/DynamixelSDK/ros"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/yunfei/Projects/robotAutonomy/CameraCalibration/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/yunfei/Projects/robotAutonomy/CameraCalibration/install/lib/python2.7/dist-packages:/home/yunfei/Projects/robotAutonomy/CameraCalibration/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/yunfei/Projects/robotAutonomy/CameraCalibration/build" \
    "/home/yunfei/bin/python" \
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/DynamixelSDK/ros/setup.py" \
    build --build-base "/home/yunfei/Projects/robotAutonomy/CameraCalibration/build/cmu-16662-robot-ctrl-master/external/DynamixelSDK/ros" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/yunfei/Projects/robotAutonomy/CameraCalibration/install" --install-scripts="/home/yunfei/Projects/robotAutonomy/CameraCalibration/install/bin"
