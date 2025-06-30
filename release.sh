#!/bin/bash

set -euo pipefail

BUILDOUT_DIR="buildout/"
ROS2_TARBALL="ros2.tar.gz"
DEPS_ONLY=0
USE_EXISTING=0
ARG="${1-}"

if [ -z "$ARG" ]; then
    echo -e "Usage: $0 -d | <template_file>\n\nFlags:\n"\
            " -d: Create ros2-build-tools package only"
    exit 1
fi

if [ "$ARG" = "-d" ]; then
    DEPS_ONLY=1
elif [ ! -f "$ARG" ]; then
    echo "Failed to open $ARG"
    exit 1
fi

if [ -f "$ROS2_TARBALL" ]; then
    read -p "Previously built $ROS2_TARBALL found. If you choose not to \
use it, it will be deleted. Do you want to use it? [Y/n] " -n 1 -r
    
    if [[ "$REPLY" != [nN] ]]; then
        USE_EXISTING=1
        echo -e "\nUsing previously built $ROS2_TARBALL"
    else
        rm $ROS2_TARBALL
        echo -e "\nRemoving $ROS2_TARBALL"
    fi
fi

if [ -d "$BUILDOUT_DIR" ]; then
    read -p "Build directory $BUILDOUT_DIR found. \
Do you want to remove it? [y/N] " -n 1 -r

    if [[ "$REPLY" != [yY] ]]; then
        echo -e "\nExiting as requested..."
        exit 1
    fi

    rm -rf $BUILDOUT_DIR
    echo -e "\nRemoving $BUILDOUT_DIR"
fi

mkdir -p $BUILDOUT_DIR

echo "Building ros2-build-tools package"
equivs-build ros2-build-tools

mv ros2-build-tools_*_*.deb $BUILDOUT_DIR
mv ros2-debian_*_*.{buildinfo,changes} $BUILDOUT_DIR

if [ $DEPS_ONLY -eq 1 ]; then
    echo "Done"
    exit 0
fi

echo "Building ros2-dist package"

if [ $USE_EXISTING -eq 0 ]; then
    ./build.sh "$1"
fi

install_path=$(yq -e -r '.install_path' "$ARG")

if [ -z "$install_path" ]; then
    echo "install_path not set, exiting"
    exit 1
fi

echo "Install path: $install_path"

install_path=$(realpath -m "$install_path")
install_path=${install_path#/}
install_path=$(dirname "tmp/ros2-dist/$install_path")/

echo "Extracting tarball to: $install_path"

rm -rf tmp/
mkdir -p tmp/ros2-dist/DEBIAN/
mkdir -p tmp/ros2-dist/usr/share/doc/ros2-dist/
mkdir -p -- "$install_path"

tar -C "$install_path" -xzf "$ROS2_TARBALL"
gzip -c changelog > tmp/ros2-dist/usr/share/doc/ros2-dist/changelog.gz

( cd "tmp/ros2-dist/"
    find . -type f ! -path './DEBIAN/*' -printf '%P\0' | sort -z |
        xargs -0 md5sum > DEBIAN/md5sums
)

ver=$(dpkg-parsechangelog -l changelog --show-field Version)
echo "Version: $ver"

arch=$(dpkg --print-architecture)
echo "Architecture: $arch"

maint=$(dpkg-parsechangelog -l changelog --show-field Maintainer)
echo "Maintainer: $maint"

deps=$(yq -e -r '.rt_deps' "$ARG")
deps=${deps// /, }
echo "Depends: $deps"

cat > "tmp/ros2-dist/DEBIAN/control" << EOF
Package: ros2-dist
Source: ros2-debian
Version: $ver
Architecture: $arch
Maintainer: $maint
Depends: $deps
Section: misc
Priority: optional
Homepage: https://github.com/markuspetermann/ros2-debian
Description: ROS 2 distribution
 Custom built ROS 2 distribution using ros2-debian with the ${ARG##*/} template
EOF

fakeroot dpkg-deb -b tmp/ros2-dist/ $BUILDOUT_DIR/ros2-dist_${ver}_${arch}.deb

echo "Done"
