# Create a Subtree of Lidar Livox Mid-360

To create a subtree of Lidar Livox Mid-360, use the following command:

```bash
git subtree add --prefix=src/livox_ros_driver https://github.com/Livox-SDK/livox_ros_driver.git master --squash
```

# To delete a subtree that you have added to your Git repository, you need to follow a few steps. Here’s how you can do it:

1. Remove the subtree directory: First, delete the subtree directory from your working directory. In your case, it would be `src/livox_ros_driver`.

2. Commit the deletion: Commit the changes to remove the directory from the repository.
   ```bash
   git rm -r src/livox_ros_driver
   git commit -m "Remove livox_ros_driver subtree"
   ```

3. Remove the subtree reference from Git history: Use the `filter-branch` command to remove all references to the subtree from your Git history. This is a bit more advanced and should be used with caution as it rewrites the history of your repository.
   ```bash
   git filter-branch --tree-filter 'rm -rf src/livox_ros_driver' HEAD
   ```

4. Force push the changes (optional): If you are working on a shared repository, you might need to force push the changes to the remote repository.
   ```bash
   git push origin master --force
   ```

### Note:
Rewriting history can cause problems if others are working on the same repository. Make sure to inform your team and coordinate accordingly.