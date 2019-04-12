# DVS Frame Filter

Removes bad frames that do not have correct dimensions (height, width).
If height or width are not provided or set to zero, it will automatically detect the correct height and width based on mode of a sample of event and image messages.

### Usage:
#### Single .bag file:

        rosrun dvs_frame_filter dvs_frame_filter path_to_input.bag <height> <width>
        
height and width are optional.
        
#### Process folder (used to process an entire folder of .bag files):

        python process_folder.py --folder <bag/folder/> --height <height> --width <width>
        
--height, --width are optional.
If --height or --width are not provided or set to zero, it will automatically detect height and width for each bag.

## Installation

Assumes you have already set up your workspace (see [here](https://github.com/cedric-scheerlinck/dvs_image_reconstruction) for more details).

Build

    catkin build dvs_frame_filter
    source ~/catkin_ws/devel/setup.bash 
    
Saves output to ```<bag_name>.bag.filtered```.

