# Human_Quad  
setup.py:<br/>
  Run with sudo python, will scan network and ask for Ground Control Station IP adress from a list of IP, then apply the relevant information to the /opt/ros/mavros/.../px4.launch file<br/><br/>
Marvelmind/positioning_pub:<br/>
  Publishes position data from Marvelmind to /mavros/vision_pose/pose. Careful of angle of marvelmind with repect to North. Should set the constant variable in the file to the right value. Eventually that should be set in a settings file...
