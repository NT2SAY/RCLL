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

echo_and_run cd "/home/rai/raibocup/raibo_ws/src/rosserial/rosserial_python"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/rai/raibocup/raibo_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/rai/raibocup/raibo_ws/install/lib/python3/dist-packages:/home/rai/raibocup/raibo_ws/build/rosserial_python/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/rai/raibocup/raibo_ws/build/rosserial_python" \
    "/usr/bin/python3" \
    "/home/rai/raibocup/raibo_ws/src/rosserial/rosserial_python/setup.py" \
     \
    build --build-base "/home/rai/raibocup/raibo_ws/build/rosserial_python" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/rai/raibocup/raibo_ws/install" --install-scripts="/home/rai/raibocup/raibo_ws/install/bin"