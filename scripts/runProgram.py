from subprocess import call
import subprocess
import sys

# How to run:
# 	python2 ../scripts/prepareDataset.py arg
# where arg is the name of the used set, e.x.:
#	python2 ../scripts/prepareDataset.py rgbd_dataset_freiburg1_desk
dataset=sys.argv[1];
scale = 1000;
if "fr" in dataset:
	scale = 5000;

# Cleaning
call("rm *.rel", shell=True);
call("rm *.abs", shell=True);
call("rm *.pcd", shell=True);

# Dataset image counter
p = subprocess.Popen("cat matchedIndices | wc -l", stdout=subprocess.PIPE, shell=True);
numberOfImages, err = p.communicate();


# Running program
#call("./kinect " + numberOfImages.rstrip() +" " + str(scale), shell=True);
call("./kinect " + "100" + " " + str(scale), shell=True);

# Running evaluation
call("python2 ../scripts/evaluate_ate.py groundtruth.txt ../results/result --verbose --scale 1 --plot ../results/ate.png > ate.res", shell=True); 
call("python2 ../scripts/evaluate_rpe.py groundtruth.txt ../results/result --interpolate --verbose --delta_unit 'f' --fixed_delta --save rpe_error --plot ../results/rpe.png > rpe.res", shell=True);
call("mv ate.res ../results/ate.res", shell=True);
call("mv rpe.res ../results/rpe.res", shell=True);
call("mv rpe_error ../results/", shell=True);
call("mv export_*.txt ../results/", shell=True);

# Move to result section
call("rm ../results/rpeatepartial/*", shell=True);
call("mv *.rel ../results/rpeatepartial", shell=True);
call("mv *.abs ../results/rpeatepartial", shell=True);

# Clouds
call("rm ../results/pcd/*", shell=True);
call("mv *.pcd ../results/pcd", shell=True);

# g2o
call("python2 ../scripts/g2o.py", shell=True);
