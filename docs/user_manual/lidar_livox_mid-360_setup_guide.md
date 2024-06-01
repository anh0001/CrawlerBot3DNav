# Lidar Livox Mid-360 Subtree Creation Guide

This guide will walk you through the process of creating subtrees for Lidar Livox Mid-360.

## Step 1: Create a Subtree of Livox SDK2

First, we need to create a subtree of the Livox SDK2. Run the following command:

```bash
git subtree add --prefix=src/Livox-SDK2 https://github.com/Livox-SDK/Livox-SDK2.git master --squash
```

After running the command, follow the installation instructions in the readme file.

## Create subtree of github livox_ros_driver2

To create a subtree of Lidar Livox Mid-360, use the following command:

```bash
git subtree add --prefix=src/livox_ros_driver2 https://github.com/Livox-SDK/livox_ros_driver2.git master --squash
```

Run this command
```bash
src/livox_ros_driver2/build.sh ROS1
```

# Notes
## To delete a subtree that you have added to your Git repository, you need to follow a few steps. Here’s how you can do it:

1. Remove the subtree directory: First, delete the subtree directory from your working directory. In your case, it would be `src/livox_ros_driver`.

2. Commit the deletion: Commit the changes to remove the directory from the repository.
   ```bash
   git rm -r src/livox_ros_driver2
   git commit -m "Remove livox_ros_driver2 subtree"
   ```

3. Remove the subtree reference from Git history: Use the `filter-branch` command to remove all references to the subtree from your Git history. This is a bit more advanced and should be used with caution as it rewrites the history of your repository.
   ```bash
   git filter-branch -f --tree-filter 'rm -rf src/livox_ros_driver2' HEAD
   ```

4. Force push the changes (optional): If you are working on a shared repository, you might need to force push the changes to the remote repository.
   ```bash
   git push origin main --force
   ```

### Note:
Rewriting history can cause problems if others are working on the same repository. Make sure to inform your team and coordinate accordingly.