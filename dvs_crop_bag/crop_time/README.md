# DVS crop time

Crops bag by timestamp.

### Usage:
#### Single .bag file:

        rosrun dvs_crop_time dvs_crop_time path_to_input.bag --t1s=<t1s> --t1e=<t1e> --t2s=<t2s> --t2e=<t2e>
        
t1s timestamp 1 start
t1e timestamp 1 end
t2s timestamp 2 start
t2e timestamp 2 end

specify the start and end boundaries for each crop 'segment'. The time intervals not included within a start and end segment will be removed, and the timestamps shifted to make the bag 'continuous'.

        
#### Process folder (used to process an entire folder of .bag files):

        python process_folder.py --folder <bag/folder/> --t1s t1s --t1e t1e 
        
## Installation

Assumes you have already set up your workspace (see [here](https://github.com/cedric-scheerlinck/dvs_image_reconstruction) for more details).

Build

    catkin build dvs_crop_time
    source ~/catkin_ws/devel/setup.bash 
    
Saves output to ```path_to_input.bag.filtered```.

