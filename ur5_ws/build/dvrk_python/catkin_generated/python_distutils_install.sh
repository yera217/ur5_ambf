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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/yera/ur5_ambf/ur5_ws/src/dvrk-ros/dvrk_python"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/yera/ur5_ambf/ur5_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/yera/ur5_ambf/ur5_ws/install/lib/python2.7/dist-packages:/home/yera/ur5_ambf/ur5_ws/build/dvrk_python/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/yera/ur5_ambf/ur5_ws/build/dvrk_python" \
    "/usr/bin/python2" \
    "/home/yera/ur5_ambf/ur5_ws/src/dvrk-ros/dvrk_python/setup.py" \
     \
    build --build-base "/home/yera/ur5_ambf/ur5_ws/build/dvrk_python" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/yera/ur5_ambf/ur5_ws/install" --install-scripts="/home/yera/ur5_ambf/ur5_ws/install/bin"
