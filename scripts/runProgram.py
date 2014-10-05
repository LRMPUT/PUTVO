from subprocess import call
import subprocess
import sys

# How to run:
# 	python2 ../scripts/prepareDataset.py arg
# where arg is the name of the used set, e.x.:
#	python2 ../scripts/prepareDataset.py rgbd_dataset_freiburg1_desk
dataset=sys.argv[1];
scale = 5000;
if "our" in dataset or "SSRR" in dataset:
	scale = 1000;

# Cleaning
call("rm *.rel", shell=True);
call("rm *.abs", shell=True);
call("rm *.pcd", shell=True);
call("rm ../results/*", shell=True);

# Dataset image counter
p = subprocess.Popen("cat matched | wc -l", stdout=subprocess.PIPE, shell=True);
numberOfImages, err = p.communicate();


# Running program
#call("./kinect " + numberOfImages.rstrip() +" " + str(scale), shell=True);
call("./kinect " + "150" + " " + str(scale), shell=True);


# Running evaluation
if "SSRR" in dataset:
	call("python2 ../scripts/evaluate_ate.py ../results/result ../results/result --verbose --scale 1 --save_associations ate_association.res --plot ../results/ate.png > ate.res", shell=True); 
else:
	call("python2 ../scripts/evaluate_ate.py groundtruth.txt ../results/result --verbose --scale 1 --save_associations ate_association.res --plot ../results/ate.png > ate.res", shell=True); 
	call("python2 ../scripts/evaluate_rpe.py groundtruth.txt ../results/result --interpolate --verbose --delta_unit 'f' --fixed_delta --plot ../results/rpe.png > rpe.res ", shell=True);
call("mv ate_association.res ../results/ate_association.res", shell=True);
call("mv ate.res ../results/ate.res", shell=True);
call("mv rpe.res ../results/rpe.res", shell=True);
call("mv export_trans.txt ../results/transErrors", shell=True);
call("mv export_rot.txt ../results/rotErrors", shell=True);

# Move to result section
call("mkdir ../results/rpeatepartial", shell=True);
call("rm ../results/rpeatepartial/*", shell=True);
call("mv *.rel ../results/rpeatepartial", shell=True);
call("mv *.abs ../results/rpeatepartial", shell=True);
call("mv timeAndLC ../results/timeAndLC", shell=True);

# Clouds
call("rm ../results/pcd/*", shell=True);
call("mv *.pcd ../results/pcd", shell=True);

# g2o
call("python2 ../scripts/g2o.py " + dataset, shell=True);
