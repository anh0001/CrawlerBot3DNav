#!/bin/bash

# Set the SCRIPT_DIR to the current script directory if not already set
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

LOG_DIR="$SCRIPT_DIR/logs"
LOG_FILE="$LOG_DIR/run_command_$(date '+%Y-%m-%d_%H-%M-%S').log"

mkdir -p "$LOG_DIR"
exec > >(tee -a "$LOG_FILE") 2>&1

log_message() {
    local message="$1"
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $message"
}

check_and_create_directory() {
    local dir="$1"
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
    fi
}

check_permissions() {
    local dir="$1"
    local permission="$2"
    if [ ! "$permission" "$dir" ]; then
        log_message "Error: Cannot $permission from/to $dir. Please check permissions."
        exit 1
    fi
}

build_and_install() {
    local name="$1"
    local build_cmd="$2"
    local install_cmd="$3"
    local src_dir="$4"
    
    log_message "Building and installing $name..."
    
    pushd "$src_dir" >/dev/null
    mkdir -p build
    pushd build >/dev/null
    
    if ! eval "$build_cmd"; then
        log_message "Error: Failed to build $name."
        popd >/dev/null
        popd >/dev/null
        return 1
    fi
    
    if ! eval "$install_cmd"; then
        log_message "Error: Failed to install $name."
        popd >/dev/null
        popd >/dev/null
        return 1
    fi
    
    popd >/dev/null
    popd >/dev/null
    
    log_message "$name built and installed successfully."
}

setup_ros_environment() {
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        ros_distribution="noetic"
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        ros_distribution="melodic"
    else
        log_message "Error: No supported ROS distribution found."
        return 1
    fi

    log_message "Setting up ROS environment for $ros_distribution..."
    
    local ros_setup_script="/opt/ros/$ros_distribution/setup.bash"
    if [[ ! -f "$ros_setup_script" ]]; then
        log_message "Error: ROS setup script for $ros_distribution does not exist."
        return 1
    fi

    source "$ros_setup_script"
    export ROS_EDITION="ROS1"
    log_message "ROS environment set up successfully for $ros_distribution."
}

install_realsense_sdk() {
    log_message "Checking for Intel RealSense SDK installation..."
    
    if dpkg -l | grep librealsense2; then
        log_message "Intel RealSense SDK already installed."
        return 0
    fi

    log_message "Installing Intel RealSense SDK..."
    
    if ! sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE; then
        log_message "Error: Failed to add Intel RealSense SDK key."
        return 1
    fi
    
    if ! sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u; then
        log_message "Error: Failed to add Intel RealSense repository."
        return 1
    fi
    
    if ! sudo apt update; then
        log_message "Error: Failed to update package list."
        return 1
    fi
    
    if ! sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg; then
        log_message "Error: Failed to install Intel RealSense SDK packages."
        return 1
    fi
    
    log_message "Intel RealSense SDK installed successfully."
}

setup() {
    log_message "Starting setup..."
    
    setup_ros_environment || return 1
    install_realsense_sdk || return 1
    
    build_and_install "Livox-SDK2" "cmake .. && make -j" "sudo make install" "src/Livox-SDK2" || return 1
    
    log_message "Building livox_ros_driver2..."
    pushd "src/livox_ros_driver2" >/dev/null
    if ! ./build.sh ROS1; then
        log_message "Error: Failed to build livox_ros_driver2."
        popd >/dev/null
        return 1
    fi
    popd >/dev/null
    
    log_message "Building ROS packages..."
    source devel/setup.bash
    if ! catkin_make; then
        log_message "Error: Failed to build ROS packages."
        return 1
    fi
    
    log_message "Setup completed successfully."
}

clear_logs() {
    log_message "Clearing all logs in $LOG_DIR"
    rm -rf "$LOG_DIR"/*
    log_message "Logs cleared."
}

clear_all() {    
    clear_logs
    log_message "Deleting devel and build directories..."
    rm -rf devel build
    log_message "Deleted devel and build directories."
}

case "$1" in
    setup) setup ;;
    clear-logs) clear_logs ;;
    clear-all) clear_all ;;
    *)
        log_message "Usage: $0 [setup|clear-logs|clear-all]"
        ;;
esac

if [ "${BASH_SOURCE[0]}" != "${0}" ]; then
    _commands_completions() {
        local cur="${COMP_WORDS[COMP_CWORD]}"
        local commands="setup clear-logs clear-all"
        COMPREPLY=( $(compgen -W "${commands}" -- ${cur}) )
    }
    complete -F _commands_completions cmds.sh
fi
