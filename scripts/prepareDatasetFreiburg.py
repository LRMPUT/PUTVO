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

# Finding matches
call("python2 ../scripts/associate.py ../../Datasets/"+dataset+"/rgb.txt ../../Datasets/"+dataset+"/depth.txt > matched", shell=True);
myFile = open('matchedIndices', 'w')

# Dataset image counter
imagesCount=call("cat matched | wc -l", shell=True);
p = subprocess.Popen("cat matched | wc -l", stdout=subprocess.PIPE, shell=True);
numberOfImages, err = p.communicate();

# Copying ground truth
call("cp ../../Datasets/"+dataset+"/groundtruth*.txt ./groundtruth.txt", shell=True);

# Copying camera parameters
if "frieburg1" in dataset:
	call("cp ../cameraFr1.cfg ../camera.cfg", shell=True); 
else:
	call("cp ../cameraFr2.cfg ../camera.cfg", shell=True); 

# Copy images with new names
for i in range(1,int(numberOfImages)+1):
	   
	p1 = subprocess.Popen("cat matched", stdout=subprocess.PIPE, shell=True);
	p2 = subprocess.Popen("cut -d' ' -f 1", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
	p3 = subprocess.Popen("head -n " + str(i), stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);	
	p4 = subprocess.Popen("tail -1", stdin=p3.stdout, stdout=subprocess.PIPE, shell=True);	
	rgbName, err = p4.communicate();

	p1 = subprocess.Popen("cat matched", stdout=subprocess.PIPE, shell=True);
	p2 = subprocess.Popen("cut -d' ' -f 3", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
	p3 = subprocess.Popen("head -n " + str(i), stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);	
	p4 = subprocess.Popen("tail -1", stdin=p3.stdout, stdout=subprocess.PIPE, shell=True);	
	dName, err = p4.communicate();

	# Debug printing 
	print(str(i)+"/"+ (numberOfImages.rstrip())+"\t" + rgbName.rstrip() + "\t" + dName.rstrip());
	
	# Copying images
	call("cp ../../Datasets/"+dataset+"/rgb/"+str(rgbName.rstrip())+".png ./rgb_%0.4d.png" % (i), shell=True);
	call("cp ../../Datasets/"+dataset+"/depth/"+str(dName.rstrip())+".png ./depth_%0.4d.png" % (i), shell=True);
	
	# Saving matches indices
	myFile.write(rgbName.rstrip() + "\t" + dName.rstrip() + "\n")

	# Saving initial position from gt
	if i==1:
		p1 = subprocess.Popen("cat matched", stdout=subprocess.PIPE, shell=True);
		p2 = subprocess.Popen("head -n 1 > test", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
		call("python2 ../scripts/associate.py groundtruth.txt test > init", shell=True);
		p1 = subprocess.Popen("cat init", stdout=subprocess.PIPE, shell=True);
		p2 = subprocess.Popen("cut -d' ' -f 2-8 > initialPosition", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
		call("rm test init", shell=True);

myFile.close()


