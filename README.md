# Bag Utility
A ROS bag utility package to continuously bag and save ROS messages to easily collect large datasets for ML applications.

# Getting Started
1) Configure topics in ```bag_utility/params/params.yaml```
2) Launch
   ```shell
   roslaunch bag_utility save_bags.launch

   # Optional params
   - name:="filename_postfix"
   - path:="/path/to/save/bags"
   ```
