# ROS 2 on debian bookworm

This repository provides simple tooling to help define, build, package, and run ROS 2 distributions on Debian Bookworm. The underlying concepts are explained in more detail [here](./DETAILS.md).

## Quickstart

```bash
# Make sure you have yq and systemd-nspawn installed
$ sudo apt install systemd-nspawn yq

# Clone this repository and build the basic ros2-base distribution
$ git clone https://github.com/markuspetermann/ros2-debian.git && cd ros2-debian/
$ ./release.sh templates/ros2-base.yaml
```

This will build two Debian packages, which can be found in the `buildout/` folder. The `build.sh` script requires superuser privileges to set up and run the container; however, it will leave your host system untouched.

## release.sh

The `release.sh` script creates two different Debian packages:

`ros2-build-tools_*.deb` - A metapackage that depends on a minimal set of Debian packages from the official Debian repository required for working with ROS 2 packages. This does not require a template and is useful for developing ROS 2 packages.

`ros2-dist_*.deb` - The custom-built ROS 2 distribution based on the given template. This installs ROS 2 to the specified location and makes it available as an immutable installation to all users.

```bash
Usage: ./release.sh -d | <template_file>

Flags:
  -d: Create ros2-build-tools package only
```

If `release.sh` is invoked with a template as an argument and detects an existing tarball (ros2.tar.gz), it will ask whether to use the existing tarball or rebuild it. This is helpful when only tweaking runtime dependencies or similar settings, as compiling a ROS 2 distribution can be a lengthy process.

## build.sh

The `build.sh` script builds a custom ROS 2 distribution from a template. It takes a template as input and outputs a tarball (ros2.tar.gz) containing a merged ROS 2 installation. The tarball can then be used to manually distribute ROS 2 or be converted to an installable Debian package using `release.sh`. To manually distribute the tarball, simply copy and extract it on the target system. The tarball must be extracted to the exact same location where it was initially built, as some parts of ROS 2 include absolute paths.

```bash
Usage: ./build.sh <template_file>
```

## Templates

The template file specifies the ROS 2 packages to include in the distribution as well as the build environment to use for building the distribution.

```yaml
build_mode: <native|container>  # native builds everything on the current system; container uses systemd-nspawn (recommended)
container_path: <path>  # path for the nspawn container, e.g. /var/lib/machines/bookworm-ros2-build
install_path: <path>  # path for the merged ROS 2 installation, e.g. /opt/ros2
ros_distro: jazzy  # currently only tested with jazzy
ros_packages: <packages>  # packages to be included in the distribution, e.g. ros_base or "desktop_full navigation2 nav2_bringup slam_toolbox joint_state_broadcaster_gui twist_mux"
ros_repos:
  - repo: <url>  # Git URL of additional packages to include in the distribution
    branch: <branch>  # branch or tag to be pulled
skip_keys: <keys>  # packages to be ignored by colcon when checking dependencies; useful when build dependencies have different names on Debian than what colcon expects, e.g., "fastcdr python3-vcstool rti-connext-dds-6.0.1 urdfdom_headers libogre-dev xtensor"
build_deps: >  # list of build dependencies to be installed from the official Debian repository
  <deps>
rt_deps: >  # list of runtime dependencies to be included in the ros2-dist_*.deb package
  <deps>
```

The templates included in the `templates/` folder can serve as starting points for building fully customized distributions.
