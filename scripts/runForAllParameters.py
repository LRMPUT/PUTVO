from subprocess import call
import subprocess
import sys
import glob
import os

folderName = "SSRR";


freiburg1 = ['rgbd_dataset_freiburg1_desk', 'rgbd_dataset_freiburg1_room'];
freiburg2 = ['rgbd_dataset_freiburg2_pioneer_360', 'rgbd_dataset_freiburg2_pioneer_slam', 'rgbd_dataset_freiburg2_pioneer_slam2', 'rgbd_dataset_freiburg2_pioneer_slam3'];
imperialCollege = ['living_room_2', 'office_traj0'];

datasetNames = [];
datasetNames.extend(freiburg2);

#preparationCommands = ['python2 ../scripts/prepareDatasetFreiburg.py rgbd_dataset_freiburg1_desk', 'python2 ../scripts/prepareDatasetFreiburg.py rgbd_dataset_freiburg1_room',  'python2 ../scripts/prepareDatasetImperialCollegeLondon.py living_room_2', 'python2 ../scripts/prepareDatasetImperialCollegeLondon.py office_traj0'];
#preparationCommands = ['python2 ../scripts/prepareDatasetFreiburg.py rgbd_dataset_freiburg2_pioneer_360'];

for index in range(len(datasetNames)):

	datasetName = datasetNames[index];

	# Preparation scripts
	if "freiburg" in datasetName:
		call('python2 ../scripts/prepareDatasetFreiburg.py ' + datasetName, shell=True);
	elif "Przejazd" in datasetName:
		call('python2 ../scripts/prepareDatasetPUT.py ' + datasetName, shell=True);
	else :
		call('python2 ../scripts/prepareDatasetImperialCollegeLondon.py ' + datasetName, shell=True);

	for file in glob.glob("../Configs/*.cfg"):
		base = os.path.basename(file);
		filename = os.path.splitext(base)[0]
	
		# Copy config
		print("cp " + str(file) + " ../parameters.cfg");
		call("cp " + str(file) + " ../parameters.cfg", shell=True);

		# Running evaluation
		call("python2 ../scripts/runProgram.py " + datasetName, shell=True); 

		if not os.path.exists("../"+folderName+"/" + datasetName):
	    		os.makedirs("../"+folderName+"/" + datasetName);

		if not os.path.exists("../"+folderName+"/" + datasetName + "/" + filename):
	    		os.makedirs("../"+folderName+"/" + datasetName + "/" + filename);
	
		# Move results
		print("mv ../results/* ../"+folderName + "/" + datasetName + "/" + filename + "/");
		call("mv ../results/* ../"+folderName + "/" + datasetName + "/" + filename + "/", shell=True);

		call("cp " + str(file) + " ../"+folderName + "/" + datasetName + "/" + filename + "/" + filename + ".cfg", shell=True);

call("python2 ../scripts/runPresentResults.py " + folderName, shell=True);