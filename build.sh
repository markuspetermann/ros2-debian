#!/bin/bash

set -euo pipefail

# Reset sudo so nothing happens without explicit confirmation
sudo -k

tplfile="${1-}"

if [ -z "$tplfile" ]; then
    echo "Usage: $0 <template_file>"
    exit 1
fi

TOOLS=(systemd-nspawn yq)
MISSING=0

RED="\033[0;31m"
GREEN="\033[0;32m"
ENDCOL="\033[0m"

echo "Checking required tools..."

for TOOL in ${TOOLS[*]}; do
    if ! command -v $TOOL &> /dev/null; then
        echo -e "$RED$TOOL not found$ENDCOL"
        MISSING=1
    else
        echo -e "$GREEN$TOOL found$ENDCOL"
    fi
done

if [ $MISSING -ne 0 ]; then
    echo "Warning: Some required tools are missing, this might lead to errors" \
         "during the build process"
    
    read -p "Do you want to continue? [y/N] " -n 1 -r

    if [[ "$REPLY" != [yY] ]]; then
        echo -e "\nExiting as requested..."
        exit 1
    fi

    echo -e "\nContinuing build process..."
fi

if [ ! -f "$tplfile" ]; then
    echo "Failed to open $tplfile"
    exit 1
fi

BUILD_MODES=("native" "container")
build_mode=$(yq -e -r '.build_mode' "$tplfile")

if [[ ! " ${BUILD_MODES[*]} " =~ " ${build_mode} " ]]; then
    echo "build_mode must be one of: ${BUILD_MODES[*]}"
    exit 1
fi

cmd_prefix=""
cmd_prefix_user=""
container_path=""
ws_path=""


# Prepare native build environment
if [ "$build_mode" = "native" ]; then
    ws_path=$(yq -e -r '.workspace_path' "$tplfile")

    if [ -z "$ws_path" ]; then
        echo "workspace_path not set, exiting"
        exit 1
    fi

    redbold=$'\e[31;1m'
    reset=$'\e[0m'

    echo "Checking workspace path ${ws_path}"

    if [ -d "${ws_path}" ]; then
        read -p "Directory $redbold$ws_path$reset exists. \
It's recommended to use an empty directory. \
Do you want to remove it? [y/N] " -n 1 -r
        
        if [[ "$REPLY" = [yY] ]]; then
            rm -rf ${ws_path}
        fi
    fi

    cmd_prefix="sudo"
    cmd_prefix_user=""
fi


# Prepare systemd-nspawn build environment
if [ "$build_mode" = "container" ]; then
    if ! command -v systemd-nspawn &> /dev/null; then
        echo >&2 "systemd-nspawn is required but not installed"
        exit 1
    fi

    container_path=$(yq -e -r '.container_path' "$tplfile")

    redbold=$'\e[31;1m'
    reset=$'\e[0m'

    echo "Checking container path ${container_path}"

    if sudo [ -d "${container_path}" ]; then
        read -p "Directory $redbold$container_path$reset exists. If you choose to \
not remove it your existing container will be used for building. \
Do you want to remove it? [y/N] " -n 1 -r
        
        if [[ "$REPLY" = [yY] ]]; then
            sudo rm -rf ${container_path}
        fi
    fi

    cmd_prefix="sudo systemd-nspawn -D ${container_path}"
    cmd_prefix_user="sudo systemd-nspawn -D ${container_path} sudo -u builder"
    ws_path="~/ros2_ws"

    sudo mkdir -p ${container_path}
    sudo debootstrap stable ${container_path} http://deb.debian.org/debian/

    ${cmd_prefix} bash -c 'sed -i "1s/$/ $(hostname)/" /etc/hosts'
    ${cmd_prefix} apt update
    ${cmd_prefix} apt install -y build-essential ca-certificates sudo
    ${cmd_prefix} adduser --disabled-password --gecos "" builder
    ${cmd_prefix} adduser builder sudo
    ${cmd_prefix} sh -c 'echo "builder:builder" | sudo chpasswd'
fi


# Install ROS2 build tools, essentially the packeges from ros2-build-tools
${cmd_prefix} apt install -y \
    cmake colcon git python3-colcon-argcomplete python3-colcon-bash \
    python3-colcon-cd python3-colcon-cmake python3-colcon-core \
    python3-colcon-defaults python3-colcon-devtools \
    python3-colcon-library-path python3-colcon-metadata \
    python3-colcon-notification python3-colcon-output \
    python3-colcon-package-information python3-colcon-package-selection \
    python3-colcon-parallel-executor python3-colcon-python-setup-py \
    python3-colcon-recursive-crawl python3-colcon-ros \
    python3-colcon-test-result python3-colcon-zsh python3-rosdep2 \
    python3-rosinstall-generator python3-setuptools vcstool wget

${cmd_prefix_user} rosdep update
${cmd_prefix_user} bash -c "mkdir -p ${ws_path}/src"

second_stage_deps=$(yq -e -r '.build_deps' "$tplfile")

if [ ! -z "$second_stage_deps" ]; then
    ${cmd_prefix} apt install -y --no-install-recommends $second_stage_deps
fi

ros_distro=$(yq -e -r '.ros_distro' "$tplfile")

if [ -z "$ros_distro" ]; then
    echo "ros_distro not set, exiting"
    exit 1
fi

ros_packages=$(yq -e -r '.ros_packages' "$tplfile")

if [ ! -z "$ros_packages" ]; then
    ${cmd_prefix_user} bash -c "rosinstall_generator $ros_packages --deps \
        --rosdistro $ros_distro | vcs import --shallow -w8 ${ws_path}/src"
fi

repos=$(yq -e -r '.ros_repos[] | .repo, .branch' "$tplfile" 2>/dev/null || \
    true)

echo "$repos" | while read -r repo; read -r branch; do
    ${cmd_prefix_user} bash -c "cd ${ws_path}/src && \
        git clone --depth 1 --branch \"$branch\" \"$repo\""
done

# FIXME: Apply patches, system/container and ws_path/src

skip_keys=$(yq -e -r '.skip_keys' "$tplfile")
[[ $skip_keys == "null" ]] && skip_keys=""

rv=$(${cmd_prefix_user} \
    bash -c "cd ${ws_path} && rosdep install --rosdistro $ros_distro \
    --from-paths src --ignore-src -y --skip-keys \"$skip_keys\" --simulate")

# Check for packages that rosdep can't resolve
if [ $? -ne 0 ]; then
    echo -e "rosdep failed to resolve at least one package, this can be"\
        "solved by finding a manual solution and adding the package to"\
        "skip_keys\n"\
        "Original message:\n$rv"
    exit 1
fi

echo "$rv"

missing_pkgs=()

while IFS= read -r line; do
    if [[ "$line" == "  sudo -H apt-get install -y "* ]]; then
        missing_pkgs+=("${line#  sudo -H apt-get install -y }")
    fi
done <<< "$rv"

if [ ${#missing_pkgs[@]} -gt 0 ]; then
    echo "The following packages are missing and either need to be installed"\
        "or ignored by adding them to skip_keys: "
    
    for pkg in "${missing_pkgs[@]}"; do
        echo "- $pkg" | tr -d '\r'    
    done

    exit 1
fi

install_path=$(yq -e -r '.install_path' "$tplfile")

if [ -z "$install_path" ]; then
    echo "install_path not set, exiting"
    exit 1
fi

if sudo [ -d "${container_path}/${install_path}" ]; then
    read -p "Directory $redbold$install_path$reset exists. \
Do you want to remove it? [y/N] " -n 1 -r
    
    if [[ "$REPLY" = [yY] ]]; then
        ${cmd_prefix} rm -rf ${install_path}
    fi
fi

${cmd_prefix} mkdir -p ${install_path}

if [ "$build_mode" = "native" ]; then
    ${cmd_prefix} chown -R $UID:$GROUPS ${install_path}
elif [ "$build_mode" = "container" ]; then
    ${cmd_prefix} chown -R builder:builder ${install_path}
fi

echo "Starting build process..."

rv=$(${cmd_prefix_user} \
    bash -c "cd ${ws_path} && colcon build --install-base ${install_path} \
        --merge-install --cmake-args \
        -DCMAKE_CXX_FLAGS=\"-Wno-error=null-dereference -Wno-error=restrict\"")  # FIXME: custom cmake-args

if [ $? -ne 0 ]; then
    echo "Failed to build packages"
    exit 1
fi

echo "Build successful, creating tarball..."

output_path=$(dirname "$container_path/$install_path")/
ros_path=$(basename "$container_path/$install_path")/

${cmd_prefix} chown -R root:root ${install_path}
sudo tar czf ros2.tar.gz -C ${output_path} ${ros_path}
sudo chown $UID:$GROUPS ros2.tar.gz
