# Human_Quad  
positioning/positioning_pub:<br/>
  Publishes position data from Marvelmind to /mavros/vision_pose/pose. Careful of angle of marvelmind with repect to North. Should set the constant variable in the file to the right value. Eventually that should be set in the settings file...<br/><br/>
  
trajectories:<br/>
  Contains tests for the four basic trajectories, as well as the reference frame transform functions and path generation functions necessary.<br/><br/>

setup.py:<br/>
  Run with sudo python, will scan network and ask for Ground Control Station IP adress from a list of IP, then apply the relevant information to the /opt/ros/mavros/.../px4.launch file<br/><br/>
  
tmux.sh:<br/>
shell script to auto-start tmux for integration work when flying the drone.<br/><br/>

config.ini:<br/>
  For now contains only the map to drone offset, set by the set_offset.py script. More to come.<br/><br/>
  
set_offset.py:<br/>
  Sets the config file fields for map rotation relative to north.<br/><br/>
  
safety_functions.py:<br/>
  For now just contains a function to check the battery level preflight, and a launch position check.<br/><br/>
  
checkpoint_reached_test.py:<br/>
  Should be deleted. Used for dev work without the drone.
