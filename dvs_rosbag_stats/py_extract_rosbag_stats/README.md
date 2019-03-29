# How to use
This program will extract cool stats about all rosbag files from a given folder.
To call it, execute:
```./extract_rosbag_stats.py /path/to/bag_files topic_name```

E.g., if your bag files are in /home/user/Downloads and the recorded topic is ```/camera/image_raw```, run:
```./extract_rosbag_stats.py /home/user/Downloads /camera/image_raw```

The script will create a .txt file, containing all the cool stats.

