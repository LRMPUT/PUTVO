from subprocess import call
import subprocess
import sys
import glob
import os

folderName = "HugeComparison";
verbose = 0;

freiburg1 = ['rgbd_dataset_freiburg1_desk', 'rgbd_dataset_freiburg1_desk2', 'rgbd_dataset_freiburg1_room'];
freiburg2 = ['rgbd_dataset_freiburg2_pioneer_360', 'rgbd_dataset_freiburg2_pioneer_slam', 'rgbd_dataset_freiburg2_pioneer_slam2', 'rgbd_dataset_freiburg2_pioneer_slam3'];
freiburg3 = ['rgbd_dataset_freiburg3_long_office_household'];
imperialCollege = ['living_room_2', 'office_traj0'];
ssrr = ['outdoorhand1', 'outdoorhand2', 'outdoorhand3', 'outdoorhand4', 'messor1', 'messor2', 'messor3'];
outdoor2 = ['outdoorhand2'];
ssrr2 = ['messor4', 'messor5'];
ssrr3 = ['messor1', 'messor2', 'messor3', 'messor4', 'messor5'];

ssrrKinVsXtion = [ 'exp1_out_asus', 'exp1_out_kinect2','exp2_out_asus', 'exp2_out_kinect2', 'exp3_out_asus', 'exp3_out_kinect2' ];
ssrrKinVsXtion2 = [ 'exp_in_4_asus', 'exp_in_4_kinect2', 'exp_out_6_kinect2' ];

datasetNames = ['rgbd_dataset_freiburg3_long_office_household'];
datasetNames.extend(freiburg1);
datasetNames.extend(imperialCollege);

for index in range(len(datasetNames)):

	datasetName = datasetNames[index];
	datasetType = datasetName;

	# Preparation scripts
	if "freiburg" in datasetName:
		call('python2 ../scripts/prepareDatasetFreiburg.py ' + datasetName, shell=True);
	elif "Przejazd" in datasetName:
		call('python2 ../scripts/prepareDatasetPUT.py ' + datasetName, shell=True);
	elif "messor" in datasetName or "outdoorhand" in datasetName:
		print(datasetName);
		call('python2 ../scripts/prepareDatasetPUT_SSRR.py ' + datasetName, shell=True);
		datasetType = "SSRR";
	elif "_out_" in datasetName or "_in_" in datasetName:
		if verbose == 1:		
			print('python2 ../scripts/prepareDatasetPUT_Kinect2.py ' + datasetName);
		call('python2 ../scripts/prepareDatasetPUT_Kinect2.py ' + datasetName, shell=True);
		datasetType = "SSRR";
	else :
		call('python2 ../scripts/prepareDatasetImperialCollegeLondon.py ' + datasetName, shell=True);
		datasetType = "Imperial";

	for file in glob.glob("../Configs/*.cfg"):
		base = os.path.basename(file);
		filename = os.path.splitext(base)[0]
	
		# Copy config
		if verbose == 1:
			print("cp " + str(file) + " ../parameters.cfg");
		call("cp " + str(file) + " ../parameters.cfg", shell=True);

		# Running evaluation
		call("python2 ../scripts/runProgram.py " + datasetType, shell=True); 

		if not os.path.exists("../"+folderName):
			if verbose == 1:
				print("mkdir ../"+folderName);
	    		os.makedirs("../"+folderName);


		if not os.path.exists("../"+folderName+"/" + datasetName):
			if verbose == 1:
				print("mkdir ../"+folderName+"/" + datasetName);
	    		os.makedirs("../"+folderName+"/" + datasetName);

		if not os.path.exists("../"+folderName+"/" + datasetName + "/" + filename):
			if verbose == 1:
				print("mkdir ../"+folderName+"/" + datasetName + "/" + filename);	    		
			os.makedirs("../"+folderName+"/" + datasetName + "/" + filename);
	
		# Move results
		print("mv ../results/* ../"+folderName + "/" + datasetName + "/" + filename + "/");
		call("mv ../results/* ../"+folderName + "/" + datasetName + "/" + filename + "/", shell=True);

		call("cp " + str(file) + " ../"+folderName + "/" + datasetName + "/" + filename + "/" + filename + ".cfg", shell=True);

call("python2 ../scripts/runPresentResults.py " + folderName, shell=True);
