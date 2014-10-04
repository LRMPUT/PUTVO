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
p = subprocess.Popen("ls ../../Datasets/"+dataset+"/color* | wc -l", stdout=subprocess.PIPE, shell=True);
numberOfImages, err = p.communicate();
print(numberOfImages)

# initial
if "kinect2" in dataset:
	call("echo '0 0 0 -0.00627547 0.00192514 -0.00385029 0.999925' > initialPosition", shell=True);
else:
	call("echo '0 0 0 0 0 0 1' > initialPosition", shell=True);

# Copying camera parameters
if "kinect2" in dataset:
	call("cp ../cameraKinect2.cfg ../camera.cfg", shell=True);
else:
	call("cp ../cameraXtion.cfg ../camera.cfg", shell=True);

myFile = open('matchedIndices', 'w')
# Copy images with new names

rgbNames = subprocess.check_output("ls ../../Datasets/"+dataset+"/color/ | sort --version-sort" , shell = True);
rgbNames = rgbNames.rstrip().rsplit('\n');

dNames = subprocess.check_output("ls ../../Datasets/"+dataset+"/depth*/ | sort --version-sort" , shell = True);
dNames = dNames.rstrip().rsplit('\n');

for i in range(1,int(numberOfImages)+1):
	   
	dName = dNames[i-1];
	rgbName = rgbNames[i-1];
	
	# Debug printing 
	print(str(i)+"/"+ (numberOfImages.rstrip())+"\t" + rgbName.rstrip() + "\t" + dName.rstrip());
	
	# Copying images
	if ".bmp" in str(rgbName.rstrip()):
		call("convert ../../Datasets/"+dataset+"/color/"+str(rgbName.rstrip())+" ./rgb_%0.5d.png" % (i), shell=True);
	else:
		call("cp ../../Datasets/"+dataset+"/color/"+str(rgbName.rstrip())+" ./rgb_%0.5d.png" % (i), shell=True);
	call("cp ../../Datasets/"+dataset+"/depth*/"+str(dName.rstrip())+ " ./depth_%0.5d.png" % (i), shell=True);

	# Saving matches indices
	myFile.write(str(i) + "\t" + str(i) + "\n")
myFile.close()		
