#!/bin/bash

# Function to update a subtree
update_subtree() {
  local prefix=$1
  local remote_url=$2
  local branch=$3

  echo "Updating subtree at $prefix from $remote_url ($branch)"

  # Fetch the latest changes from the remote repository
  git fetch $remote_url $branch

  # Merge the fetched changes into the subtree
  git subtree pull --prefix=$prefix $remote_url $branch --squash
}

# Update livox_ros_driver subtree
update_subtree "src/livox_ros_driver" "https://github.com/Livox-SDK/livox_ros_driver.git" "master"

# Update realsense-ros subtree
update_subtree "src/realsense-ros" "https://github.com/IntelRealSense/realsense-ros.git" "ros1-legacy"

echo "Subtrees updated successfully."
