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

process_rosbag() {
    local src_bag="$1"
    local base_filename=$(basename "$src_bag")
    local dst_bag="$rosbag_processed_dir/fastlio_$base_filename"
    local log_file="$LOG_DIR/roslaunch_$(basename "$src_bag" .bag).log"

    log_message "Processing: $src_bag -> $dst_bag"
    
    if ! roslaunch fast_lio bag_mapping_mid360.launch src_bag:="$src_bag" dst_bag:="$dst_bag" &> "$log_file"; then
        log_message "Error processing $src_bag. Check the log file for details: $log_file"
    else
        log_message "Successfully processed $src_bag. Output written to $dst_bag"
    fi
}

process_npz() {
    local src_bag="$1"
    local base_filename=$(basename "$src_bag" .bag)
    local output_dir="$SCRIPT_DIR/out"
    local log_file="$LOG_DIR/npz_$(basename "$src_bag" .bag).log"

    check_and_create_directory "$output_dir"
    
    log_message "Converting: $src_bag to npz format"
    
    if ! python3 scripts/pointcloud2_to_npz.py "$src_bag" -t /cloud_registered -o "$output_dir" &> "$log_file"; then
        log_message "Error converting $src_bag to npz. Check the log file for details: $log_file"
    else
        log_message "Successfully converted $src_bag to npz format. Output written to $output_dir"
    fi
}

visualize_pointcloud() {
    local npz_file="$1"
    if [ ! -f "$npz_file" ]; then
        log_message "Error: Specified file $npz_file does not exist."
        exit 1
    fi

    log_message "Visualizing: $npz_file"

    if ! python3 scripts/npz_pointcloud_viewer_open3d.py "$npz_file"; then
        log_message "Error visualizing $npz_file. Please check the file and try again."
    else
        log_message "Successfully visualized $npz_file."
    fi
}

visualize_all() {
    local npz_dir="$SCRIPT_DIR/out"
    log_message "Visualizing all npz files in $npz_dir"

    for npz_file in "$npz_dir"/*.npz; do
        if [ -f "$npz_file" ]; then
            visualize_pointcloud "$npz_file"
        else
            log_message "No npz files found in $npz_dir."
        fi
    done
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

setup() {
    log_message "Starting setup..."
    
    setup_ros_environment || return 1
    
    build_and_install "Livox-SDK2" "cmake .. && make -j" "sudo make install" "src/Livox-SDK2" || return 1
    build_and_install "livox_ros_driver2" "./build.sh ROS1" "" "src/livox_ros_driver2" || return 1
    
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
    run-fastlio-mapping) run_fastlio ;;
    map-to-npz) run_convert_npz ;;
    combine-maps-to-npz) combine_maps_to_npz ;;
    visualize-pc)
        if [ -z "$2" ]; then
            log_message "Error: Please specify the npz file to visualize."
            log_message "Usage: $0 visualize-pc <file.npz>"
            exit 1
        fi
        visualize_pointcloud "$2"
        ;;
    visualize-all) visualize_all ;;
    clear-logs) clear_logs ;;
    clear-all) clear_all ;;
    *)
        log_message "Usage: $0 [setup|run-fastlio-mapping|map-to-npz|combine-maps-to-npz|visualize-pc|visualize-all|clear-logs|clear-all]"
        ;;
esac

if [ "${BASH_SOURCE[0]}" != "${0}" ]; then
    _commands_completions() {
        local cur="${COMP_WORDS[COMP_CWORD]}"
        local commands="setup run-fastlio-mapping map-to-npz combine-maps-to-npz visualize-pc visualize-all clear-logs clear-all"
        COMPREPLY=( $(compgen -W "${commands}" -- ${cur}) )
    }
    complete -F _commands_completions cmds.sh
fi
