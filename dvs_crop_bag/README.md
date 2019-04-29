# DVS crop bag

Crops bag by number of messages.

### Usage:
#### Single .bag file:

        rosrun dvs_crop_bag dvs_crop_bag path_to_input.bag <num_msgs> <num_skip>
        
num_skip is an optional parameters that specifies the number of messages to skip at the beginning.
        
#### Process folder (used to process an entire folder of .bag files):

        python process_folder.py --folder <bag/folder/> --num_msgs <num_msgs>
        
## Installation

Assumes you have already set up your workspace (see [here](https://github.com/cedric-scheerlinck/dvs_image_reconstruction) for more details).

Build

    catkin build dvs_crop_bag
    source ~/catkin_ws/devel/setup.bash 
    
Saves output to ```path_to_input.bag.filtered```.

