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
p = subprocess.Popen("ls ../../Datasets/"+dataset+"/img/rgb_* | wc -l", stdout=subprocess.PIPE, shell=True);
numberOfImages, err = p.communicate();
print(numberOfImages)

# copying ground truth
# znacznik
p = subprocess.Popen("ls ../../Datasets/"+dataset+"/*znac*", stdout=subprocess.PIPE, shell=True);	
znac, err = p.communicate();
f = open(znac.rstrip());
znacznik = [];
for line in f:
	znacznik = [x.strip() for x in line.split(';')];
f.close();
# gt
myGT = open('groundtruth.txt', 'w')
matchedInd = open('matchedIndices', 'w');
p = subprocess.Popen("ls ../../Datasets/"+dataset+"/*traj*", stdout=subprocess.PIPE, shell=True);	
traj, err = p.communicate();
f = open(traj.rstrip());
i = 1;
for line in f:
	gtline = [x.strip() for x in line.split(';')];
	print("Ground truth recalculation: " + str(i) + "/" + numberOfImages.rstrip());
	p5 = subprocess.Popen("./kinect " + gtline[1] + " " + gtline[2] + " " + gtline[3] + " " + gtline[4] + " " + gtline[5] + " " + gtline[6] + " " + znacznik[0] + " " + znacznik[1] + " " + znacznik[2] + " " + znacznik[3] + " " + znacznik[4] + " " + znacznik[5], stdout=subprocess.PIPE, shell=True);	
	traj, err = p5.communicate();
	myGT.write(gtline[0] + " " + traj);
	matchedInd.write(gtline[0] + "\n");
	i=i+1;
matchedInd.close();
myGT.close();


# Copying camera parameters
call("cp ../cameraOur.cfg ../camera.cfg", shell=True);

myFile = open('matched', 'w')
# Copy images with new names
for i in range(1,int(numberOfImages)+1):
	   
	p1 = subprocess.Popen("ls ../../Datasets/"+dataset+"/img/", stdout=subprocess.PIPE, shell=True);
	p2 = subprocess.Popen("grep rgb", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
	p3 = subprocess.Popen("sort --version-sort", stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);	
	p4 = subprocess.Popen("head -n " + str(i), stdin=p3.stdout, stdout=subprocess.PIPE, shell=True);	
	p5 = subprocess.Popen("tail -1", stdin=p4.stdout, stdout=subprocess.PIPE, shell=True);	
	rgbName, err = p5.communicate();

	p1 = subprocess.Popen("ls ../../Datasets/"+dataset+"/img/", stdout=subprocess.PIPE, shell=True);
	p2 = subprocess.Popen("grep depth", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
	p3 = subprocess.Popen("sort --version-sort", stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);	
	p4 = subprocess.Popen("head -n " + str(i), stdin=p3.stdout, stdout=subprocess.PIPE, shell=True);	
	p5 = subprocess.Popen("tail -1", stdin=p4.stdout, stdout=subprocess.PIPE, shell=True);	
	dName, err = p5.communicate();

	# Debug printing 
	print(str(i)+"/"+ (numberOfImages.rstrip())+"\t" + rgbName.rstrip() + "\t" + dName.rstrip());
	
	# Copying images
	call("convert ../../Datasets/"+dataset+"/img/"+str(rgbName.rstrip())+" ./rgb_%0.4d.png" % (i), shell=True);
	call("cp ../../Datasets/"+dataset+"/img/"+str(dName.rstrip())+ " ./depth_%0.4d.png" % (i), shell=True);

	# Saving matches indices
	myFile.write(str(i) + "\t" + str(i) + "\n")
	
	# Saving initial position from gt
	if i==1:
		p1 = subprocess.Popen("cat groundtruth.txt", stdout=subprocess.PIPE, shell=True);
		p2 = subprocess.Popen("head -n 1", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
		p3 = subprocess.Popen("cut -d' ' -f 2-8 > initialPosition", stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);
myFile.close()		
