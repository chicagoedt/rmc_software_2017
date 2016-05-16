from __future__ import print_function
import subprocess

filep1 = '''<?xml version="1.0" encoding="utf-8"?>
<launch>

  <param name="useAruco" type="int" value="1"/>

  <param name="values_for_average" type="int" value="20"/>         <!-- This param is used to determine how many of the previous data are saved and averaged -->
  <param name="threshold_for_average" type="double" value="1.2"/>   <!-- Used to determine how much a value needs to be above average to be a 'hit' -->
  <param name="num_to_dock" type="int" value="3"/>             <!-- How many consecutive hits are needed to determine dock state -->
  <param name="arucoDistance" type="double" value="'''

filep2= '''"/>

  <node pkg="state_machine" type="mission_validator" name="mission_validator" output="screen"/>

  <node pkg="state_machine" type="state_machine_node" name="state_machine" output="screen" >
        <rosparam command="load" file="$(find rmc_config)/yaml/waypoints.yaml" />
  </node>

</launch>
'''

############################################################################

print("checking rostopic list for /ar_single_board/pose topic")
list_of_topics = subprocess.Popen("timeout 5s rostopic list", stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True);

output = '';
err = '';
output, err = list_of_topics.communicate();
output = output.rstrip();
err = err.rstrip();

if (err[:5] == 'ERROR'):
	print("An error has occured:", err, sep='\n', end='\n')
	exit(-1);

lookup = "/ar_single_board/pose"
if (lookup not in output):
	print("The topic", lookup, "does not exist.\nExiting", sep=' ', end='\n')
	exit(-1)

#############################################################################

print("checking for errors");

val = subprocess.Popen("timeout 3s rosrun tf tf_echo /base_link /ar_board_marker | head -n 2 | sed -n '2p'", stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True);
output = '';
err = '';
output, err = val.communicate();
output = output.rstrip();

if (err[:6] == '[ERROR'):
	print("An error has occured:", err, sep='\n', end='\n')
	exit(-1);
if (err[:9] == 'Exception'):
	print("An error has occured:", err, sep='\n', end='\n')
	exit(-1);

#############################################################################

print("No errors detected. Processing...")
print(output[2:])

while (output[:1] != ":"):
	output = output[1:]
output = output[4:].rstrip();
finout = '';
while (output[:1] != ","):
	finout+=output[:1];
	output = output[1:];
final = filep1+finout+filep2;

print("Writing to ../launch/state_machine.launch")
target = open("../launch/state_machine.launch", 'w')
target.write(final);
target.close();
