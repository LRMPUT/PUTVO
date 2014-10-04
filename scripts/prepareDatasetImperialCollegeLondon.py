from subprocess import call
import subprocess
import sys

# How to run:
# 	python2 ../scripts/prepareDataset.py arg
# where arg is the name of the used set, e.x.:
#	python2 ../scripts/prepareDataset.py rgbd_dataset_freiburg1_desk
dataset=sys.argv[1];

# Cleaning
call("rm *.png", shell=True);
call("rm matched", shell=True);
call("rm matchedIndices", shell=True);

# Dataset image counter
p = subprocess.Popen("ls ../../Datasets/"+dataset+"/*.png | wc -l", stdout=subprocess.PIPE, shell=True);
numberOfImages, err = p.communicate();
print(numberOfImages)

# Copying ground truth
call("cp ../../Datasets/"+dataset+"/*.gt.freiburg ./groundtruth.txt", shell=True);

# Copying camera parameters
call("cp ../cameraImperialCollege.cfg ../camera.cfg", shell=True); 

# Copy images with new names
for i in range(1,int(numberOfImages)+1):
	   
	p1 = subprocess.Popen("ls ../../Datasets/"+dataset+"/", stdout=subprocess.PIPE, shell=True);
	p2 = subprocess.Popen("grep .png", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
	p3 = subprocess.Popen("sort --version-sort", stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);	
	p4 = subprocess.Popen("head -n " + str(i), stdin=p3.stdout, stdout=subprocess.PIPE, shell=True);	
	p5 = subprocess.Popen("tail -1", stdin=p4.stdout, stdout=subprocess.PIPE, shell=True);	
	rgbName, err = p5.communicate();

	p1 = subprocess.Popen("ls ../../Datasets/"+dataset+"/", stdout=subprocess.PIPE, shell=True);
	p2 = subprocess.Popen("grep .depth", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
	p3 = subprocess.Popen("sort --version-sort", stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);	
	p4 = subprocess.Popen("head -n " + str(i), stdin=p3.stdout, stdout=subprocess.PIPE, shell=True);	
	p5 = subprocess.Popen("tail -1", stdin=p4.stdout, stdout=subprocess.PIPE, shell=True);	
	dName, err = p5.communicate();

	# Debug printing 
	print(str(i)+"/"+ (numberOfImages.rstrip())+"\t" + rgbName.rstrip() + "\t" + dName.rstrip());
	
	# Copying images
	call("cp ../../Datasets/"+dataset+"/"+str(rgbName.rstrip())+" ./rgb_%0.5d.png" % (i), shell=True);
	call("cp ../../Datasets/"+dataset+"/"+str(dName.rstrip())+" ./depth_%0.4d.depth" % (i), shell=True);
	
	# Converting depth images
	if "living" in dataset:
		call("./kinect 0 depth_%0.4d.depth" % (i) + " depth_%0.5d.png" % (i), shell=True);
	else:
		call("./kinect 1 depth_%0.4d.depth" % (i) + " depth_%0.5d.png" % (i), shell=True);
	call("rm *.depth", shell=True);

	# Saving initial position from gt
	if i==1:
		p1 = subprocess.Popen("cat groundtruth.txt", stdout=subprocess.PIPE, shell=True);
		p2 = subprocess.Popen("head -n 1", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
		p3 = subprocess.Popen("cut -d' ' -f 2-8 > initialPosition", stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);

	p1 = subprocess.Popen("cat groundtruth.txt", stdout=subprocess.PIPE, shell=True);
	p2 = subprocess.Popen("head -n " + str(i), stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
	p3 = subprocess.Popen("tail -1", stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);
	p4 = subprocess.Popen("cut -d' ' -f 1 >> matchedIndices", stdin=p3.stdout, stdout=subprocess.PIPE, shell=True);

