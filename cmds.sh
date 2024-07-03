#!/bin/bash

# Set the SCRIPT_DIR to the current script directory if not already set
SCRIPT_DIR="${SCRIPT_DIR:-$(dirname "$(readlink -f "$0")")}"

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

build_packages() {
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        ros_distribution="noetic"
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        ros_distribution="melodic"
    else
        echo "Error: No supported ROS distribution found."
        return 1
    fi

    echo "Building ROS packages for $ros_distribution..."
    export ROS_EDITION="ROS1"

    local ros_setup_script="/opt/ros/$ros_distribution/setup.bash"
    if [[ ! -f "$ros_setup_script" ]]; then
        echo "Error: ROS setup script for $ros_distribution does not exist."
        return 1
    fi

    source "$ros_setup_script"

    # Build and install Livox-SDK2
    pushd src/Livox-SDK2 >/dev/null
    mkdir -p build
    pushd build >/dev/null
    if ! cmake .. && make -j; then
        echo "Error: Failed to build Livox-SDK2."
        popd >/dev/null
        popd >/dev/null
        return 1
    fi
    if ! sudo make install; then
        echo "Error: Failed to install Livox-SDK2."
        popd >/dev/null
        popd >/dev/null
        return 1
    fi
    popd >/dev/null
    popd >/dev/null

    # # Build and install Livox-ROS-Driver2
    # pushd src/livox_ros_driver2 >/dev/null
    # if ! ./build.sh ROS1; then
    #     echo "Error: Failed to build livox_ros_driver2."
    #     popd >/dev/null
    #     return 1
    # fi
    # popd >/dev/null

    # if [ "$#" -eq 1 ]; then
    #     if ! catkin_make --only-pkg-with-deps "$1"; then
    #         echo "Error: Failed to build the specified ROS package."
    #         return 1
    #     fi
    # else
    #     if ! catkin_make; then
    #         echo "Error: Failed to build the workspace."
    #         return 1
    #     fi
    # fi

    # echo "Build completed successfully."
}

clear_logs() {
    log_message "Clearing all logs in $LOG_DIR"
    rm -rf "$LOG_DIR"/*
    log_message "Logs cleared."
}

case "$1" in
    build-packages) build_packages ;;
    run-fastlio-mapping) run_fastlio ;;
    map-to-npz) run_convert_npz ;;
    combine-maps-to-npz) combine_maps_to_npz ;;
    visualize-pc)
        if [ -z "$2" ]; then
            echo "Error: Please specify the npz file to visualize."
            echo "Usage: $0 visualize-pc <file.npz>"
            exit 1
        fi
        visualize_pointcloud "$2"
        ;;
    visualize-all) visualize_all ;;
    clear-logs) clear_logs ;;
    *)
        echo "Usage: $0 [build-packages|run-fastlio-mapping|map-to-npz|combine-maps-to-npz|visualize-pc|visualize-all|clear-logs]"
        ;;
esac

if [ "${BASH_SOURCE[0]}" != "${0}" ]; then
    _commands_completions() {
        local cur="${COMP_WORDS[COMP_CWORD]}"
        local commands="build-packages run-fastlio-mapping map-to-npz combine-maps-to-npz visualize-pc visualize-all clear-logs"
        COMPREPLY=( $(compgen -W "${commands}" -- ${cur}) )
    }
    complete -F _commands_completions cmds.sh
fi
