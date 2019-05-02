# DVS sort events

Sort events in a bag by timestamp.

### Usage:
#### Single .bag file:

    rosrun dvs_sort_events dvs_sort_events path_to_input.bag
        
#### Process folder (used to process a folder of .bag files):

    python process_folder.py --folder <bag/folder/>
        
## Installation

Assumes you have already set up your workspace (see [here](https://github.com/cedric-scheerlinck/dvs_image_reconstruction) for more details).

Build

    catkin build dvs_sort_events
    source ~/catkin_ws/devel/setup.bash 
    
Saves output to ```path_to_input.bag.sorted```.

