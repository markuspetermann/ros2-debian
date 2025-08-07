# Build ROS 2 on debian bookworm

This brief documentation outlines one of many ways to build, package, and deploy ROS 2 (Jazzy) on Debian Bookworm. As Debian Bookworm is only supported at a Tier 3 level [1], one must compile the desired set of ROS 2 packages from source. There are various approaches to achieve this. In this note, I focus on obtaining a __clean, lightweight, and deployable set of core packages, resulting in a well-defined and immutable environment suitable for both production and development.__ For production, the aim is to have a lightweight (small in size) package for deployment. For development, the aim is to keep the overlay sourced on top of the immutable core packages as thin as possible to minimize variance across different machines and developers.

This involves the following steps:

* Set up a VM, container, or chroot to build the ROS 2 core workspace (optional, but useful due to the myriad of dependencies needed)
* Install essential ROS 2 build tooling from the official Debian repository
* Use `rosinstall_generator` or a handcrafted list to fetch the sources of the desired ROS 2 packages
* Resolve all build dependencies, both Debian and ROS 2 packages
* Build and install the ROS 2 packages to a generic, user-independent directory
* Create a package containing the install directory, i.e., the resulting immutable ROS 2 core workspace

## Optional: Use systemd-nspawn to create a build environment

As ROS 2 requires numerous dependencies to build, it may be desirable to run the build process in a separate Debian environment. Here is a brief example of how to use systemd-nspawn to set up and use a container

```bash
# Setup the container
$ sudo debootstrap stable /var/lib/machines/bookworm-ros2 http://deb.debian.org/debian/
# Switch into the container
$ sudo systemd-nspawn -D /var/lib/machines/bookworm-ros2
```

Within the container, create a build user

```bash
$ apt install build-essential ca-certificates sudo
$ adduser builder
$ adduser builder sudo
$ su builder
```

## Install essential ROS 2 tooling from Debian repositories

A few tools are needed or at least helpful to download the desired set of package sources, examine their build dependencies, and start the build process. This list mostly resembles the packages included in the ros-dev-tools package [2]. The aim is to keep this list as short as possible

```bash
$ sudo apt install cmake colcon git python3-colcon-argcomplete python3-colcon-bash python3-colcon-cd python3-colcon-cmake python3-colcon-core python3-colcon-defaults python3-colcon-devtools python3-colcon-library-path python3-colcon-metadata python3-colcon-notification python3-colcon-output python3-colcon-package-information python3-colcon-package-selection python3-colcon-parallel-executor python3-colcon-python-setup-py python3-colcon-recursive-crawl python3-colcon-ros python3-colcon-test-result python3-colcon-zsh python3-rosdep2 python3-rosinstall-generator python3-setuptools vcstool wget
```

Lastly, run

```bash
$ rosdep update
```

## Clone package sources

Create a workspace folder and a `src/` directory within it

```bash
$ mkdir -p ~/ros2_ws/src/ && cd ros2_ws/
```

Next, clone the desired source repositories to the `src/` directory either by using `rosinstall_generator` and `vcs`

```bash
$ rosinstall_generator desktop_full navigation2 --deps --rosdistro jazzy | vcs import src/
```

or by manually cloning the packages git repository to the `src/` folder

```bash
$ cd src/ && git clone ...
```

One may also combine the two methods, e.g., pull in ros_desktop_full and nav2 via `rosinstall_generator` and then clone custom packages alongside.

## Examine further build dependencies

The collected packages will likely require additional build dependencies. There are two types of build dependencies that need different handling. To get a list of the required and unsatisfied dependencies, use

```bash
$ rosdep install --rosdistro jazzy --from-paths src --ignore-src -y --skip-keys \
  "fastcdr python3-vcstool rti-connext-dds-6.0.1 urdfdom_headers xtensor" --simulate
```

The `-skip-keys` are a combination of the recommendations from the ROS 2 build documentation [3] and additional keys for packages that are named differently and consequently not detected by `rosdep`, such as xtensor, which is provided by libxtensor-dev in Debian.

The above command returns a list of dependencies that `rosdep` would try to install. While this would work for packages that are available in the Debian repositories and are named correctly, it would fail for all ROS 2 packages that are only available as Debian packages on Tier 1 platforms. The ROS 2 packages can be easily identified by the `ros2-` prefix.

First, most dependencies that are available from the official Debian repository, e.g., `curl`, can simply be installed by copying the package names and using `apt install`. To reduce the number of installed packages, add the `--no-install-recommends` flag

```bash
$ sudo apt install --no-install-recommends <list of packages>
```

For some packages such as `xtensor`, this will fail as Debian does not provide a package named `xtensor`. In these cases, use `apt search` to find the package that provides the required build dependencies—in this example, it is `libxtensor-dev`—and install it in the same way. It may be useful to add packages that cannot be resolved automatically to `-skip-keys`. `rosdep` provides a list of dependency name to Debian name mappings in `/usr/share/python3-rosdep2/debian.yaml`; however, it appears to be out of date.

Second, if there are any packages prefixed with `ros2-`, manually search for and clone the source of the respective ROS 2 packages as described above. This should only occur when adding packages manually, as `rosinstall_generator`, when called with the `--deps` flag, handles dependencies on other ROS 2 packages.

## Build and install the ROS 2 packages

Create a generic, user-independent installation directory

```bash
$ sudo mkdir -p /opt/ros2
# Temporarily grant write permissions to the build user
$ sudo chown builder:builder /opt/ros2
```

Start the build process from within the workspace directory

```bash
$ cd ~/ros2_ws/
$ colcon build --install-base /opt/ros2 --merge-install --cmake-args -DCMAKE_CXX_FLAGS="-Wno-error=null-dereference -Wno-error=restrict"
```

Use `--merge-install` to reduce the length of the resulting `PATH` and similar environment variables when sourcing the final installation. The `CMAKE_CXX_FLAGS` convert some errors to warnings; otherwise, the build for the navigation2 package will fail.

## Create a deployable package of the built core workspace

A simple method for creating a manually deployable package of the resulting workspace is to create a tar archive

```bash
$ sudo chown -R root:root /opt/ros2
$ tar czf ros2.tar.gz -C /opt/ ros2/
```

## Using the core workspace

While some ROS 2 packages work when the installation folder is relocated after build, others such as Gazebo do not. Therefore, ensure that you always extract the archive to the same location where it was initially built, e.g., `/opt/ros2`.

Then, source the core workspace

```bash
$ . /opt/ros2/setup.bash
```

For development, create and source an overlay workspace to develop additional ROS 2 packages as described in the official documentation.

When using the core workspace in an environment other than the build environment, some runtime dependencies will most certainly be missing. This usually results in ROS nodes and applications being unable to start. In most cases, the error messages clearly indicate which packages are missing. Here is a list of necessary runtime dependencies I have collected over time; however, this may vary significantly among different setups

```bash
# Runtime dependencies for ROS 2 core packages
$ sudo apt install --no-install-recommends liblttng-ust1 libspdlog1.10

# Runtime dependencies for demo_nodes_cpp
$ sudo apt install --no-install-recommends libconsole-bridge1.0

# Runtime dependencies for RViz2
$ sudo apt install --no-install-recommends libzzip-0-13 liblttng-ust1 libspdlog1.10 liborocos-kdl1.5 python3-lark

# Runtime dependencies for Gazebo Harmonic 
$ sudo apt install --no-install-recommends libtinyxml2-9 liburdfdom-model3.0 libassimp5 qml-module-qtquick-controls qml-module-qtquick-controls2 qml-module-qtquick-dialogs libgdal32 libbullet3.24 libode8 libfcl0.7 libgflags2.2
```

[1] [https://www.ros.org/reps/rep-2000.html](https://www.ros.org/reps/rep-2000.html)
[2] [https://www.ros.org/reps/rep-2001.html](https://www.ros.org/reps/rep-2001.html)
[3] [https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html)
