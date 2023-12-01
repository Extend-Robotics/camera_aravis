#! /bin/bash

# ------------------------------------ #
# This script generates debian package #
# Assumptions:
# - ROS environment was sourced
# - ROS melodic is on Ubuntu bionic
# - ROS noetic is on Ubuntu focal
#
# By default result will be in
# - /tmp/packages
#
# The script
# - installs dependencies for packaging
# - copies source to temporary location
# - installs package dependencies (rosdep)
# - builds debian package
# - copies result to $BINARIES_PATH
# -------------------------------------#

set -e

# Config

BUILD_PATH=/tmp/catkin_pkg
BINARIES_PATH=/tmp/packages

# Start from directory with this script
cd "$(dirname "$0")"
# Move to package directory
cd ..

# Get package name
PACKAGE_PATH=$(basename $(pwd))

# Move to directory containing package
cd ..

echo "debian packaging $PACKAGE_PATH"

if [ "$ROS_DISTRO" == "melodic" ]
then
OS_VERSION=bionic
elif [ "$ROS_DISTRO" == "noetic" ]
then
OS_VERSION=focal
fi

# packaging dependencies

if [ "$ROS_DISTRO" == "melodic" ]
then
  sudo apt-get install -y python-bloom fakeroot
elif [ "$ROS_DISTRO" == "noetic" ]
then
  sudo apt-get install -y python3-bloom fakeroot debhelper
fi

### Copy source

mkdir -p "$BUILD_PATH"/src
cp -r "$PACKAGE_PATH" "$BUILD_PATH"/src/
cd "$BUILD_PATH"/src

rosdep install --from-paths . -r -y --rosdistro="$ROS_DISTRO" && \
cd "$PACKAGE_PATH"

### package
bloom-generate rosdebian --os-name ubuntu --os-version "$OS_VERSION" --ros-distro "$ROS_DISTRO"

if [ $(whoami) == "root" ]
then
  debian/rules binary
else
  fakeroot debian/rules binary
fi

# copy result

mkdir -p "$BINARIES_PATH"
cp ../*.deb "$BINARIES_PATH"

# Cleanup

rm -rf "$BUILD_PATH"

# Final message

echo "the resulting package was copied to"
echo "$BINARIES_PATH"

