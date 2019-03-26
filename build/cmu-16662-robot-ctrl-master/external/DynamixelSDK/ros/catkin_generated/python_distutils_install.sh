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

echo_and_run cd "/home/parallels/Autonomy_ws/hw4/src/cmu-16662-robot-ctrl-master/external/DynamixelSDK/ros"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/parallels/Autonomy_ws/hw4/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/parallels/Autonomy_ws/hw4/install/lib/python2.7/dist-packages:/home/parallels/Autonomy_ws/hw4/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/parallels/Autonomy_ws/hw4/build" \
    "/usr/bin/python" \
    "/home/parallels/Autonomy_ws/hw4/src/cmu-16662-robot-ctrl-master/external/DynamixelSDK/ros/setup.py" \
    build --build-base "/home/parallels/Autonomy_ws/hw4/build/cmu-16662-robot-ctrl-master/external/DynamixelSDK/ros" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/parallels/Autonomy_ws/hw4/install" --install-scripts="/home/parallels/Autonomy_ws/hw4/install/bin"
