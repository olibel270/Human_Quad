# Human_Quad  
For now, three files.<br/>
setup.py:<br/>
  Run with sudo python, will scan network and ask for Ground Control Station IP adress from a list of IP, then apply the relevant information to the /opt/ros/mavros/.../px4.launch file<br/><br/>
set_origin.py.backup:<br/>
  Version of set_origin.py that used to work with arducopter, gets rid of the "Requesting Home Position" message, and sets the global origin of the drone for accurate magnetic declination of compass.<br/><br/>
positioning_pub:<br/>
  publishes position data from pozyx to /mavros/vision_pose/pose. Careful of angle of pozyx with repect to North. Couldn't test it, Pozyx Fried...
