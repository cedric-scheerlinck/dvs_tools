# DVS sort events

Sort events in a bag by timestamp.

### Usage:
#### Single .bag file:

    rosrun dvs_sort_events sort path_to_input.bag --max_events=<max_events> --max_duration_ms=<max_time>
    
max_events is the maximum number of events per message.  
max_duration_ms is the maximum duration of a message in milliseconds.  

If both are set, messages are written with whichever condition is met first.
If neither are set, defaults to 30ms time-windows.
        
#### Process folder (used to process a folder of .bag files):

    python process_folder.py --folder <bag/folder/> --max_events <max_events> --max_duration_ms <max_time>
        
## Installation

Assumes you have already set up your workspace (see [here](https://github.com/cedric-scheerlinck/dvs_image_reconstruction) for more details).

Build

    catkin build dvs_sort_events
    source ~/catkin_ws/devel/setup.bash 
    
Saves output to ```path_to_input.bag.sorted```.

