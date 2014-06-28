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
p = subprocess.Popen("ls ../../Datasets/"+dataset+"/image* | wc -l", stdout=subprocess.PIPE, shell=True);
numberOfImages, err = p.communicate();
print(numberOfImages)

# initial
if "messor" not in dataset:
	call("echo '0 0 0 0 0 0 1' > initialPosition", shell=True);
else: 
	call("echo '0 0 0 0 0 0.156434 0.987688' > initialPosition", shell=True);

# Copying camera parameters
call("cp ../cameraXtion.cfg ../camera.cfg", shell=True);

myFile = open('matchedIndices', 'w')
# Copy images with new names

rgbNames = subprocess.check_output("ls ../../Datasets/"+dataset+"/ | grep image | sort --version-sort" , shell = True);
rgbNames = rgbNames.rstrip().rsplit('\n');

dNames = subprocess.check_output("ls ../../Datasets/"+dataset+"/ | grep depth | sort --version-sort" , shell = True);
dNames = dNames.rstrip().rsplit('\n');

for i in range(1,int(numberOfImages)+1):
	   
	dName = dNames[i-1];
	rgbName = rgbNames[i-1];
	#p1 = subprocess.Popen("ls ../../Datasets/"+dataset+"/", stdout=subprocess.PIPE, close_fds=True, shell=True);
	#p2 = subprocess.Popen("grep image", stdin=p1.stdout, stdout=subprocess.PIPE, close_fds=True, shell=True);
	#p3 = subprocess.Popen("sort --version-sort", stdin=p2.stdout, stdout=subprocess.PIPE, close_fds=True, shell=True);	
	#p4 = subprocess.Popen("head -n " + str(i), stdin=p3.stdout, stdout=subprocess.PIPE, close_fds=True, shell=True);	
	#p5 = subprocess.Popen("tail -1", stdin=p4.stdout, stdout=subprocess.PIPE, close_fds=True, shell=True);	
	#rgbName, err = p5.communicate();
	#rgbName = subprocess.check_output("ls ../../Datasets/"+dataset+"/ | grep image | sort --version-sort | head -n " + str(i) + " | tail -1" , shell = True);

	#p1 = subprocess.Popen("ls ../../Datasets/"+dataset+"/", stdout=subprocess.PIPE, close_fds=True, shell=True);
	#p2 = subprocess.Popen("grep depth", stdin=p1.stdout, stdout=subprocess.PIPE, close_fds=True, shell=True);
	#p3 = subprocess.Popen("sort --version-sort", stdin=p2.stdout, stdout=subprocess.PIPE, close_fds=True, shell=True);	
	#p4 = subprocess.Popen("head -n " + str(i), stdin=p3.stdout, stdout=subprocess.PIPE, close_fds=True, shell=True);	
	#p5 = subprocess.Popen("tail -1", stdin=p4.stdout, stdout=subprocess.PIPE, close_fds=True, shell=True);	
	#dName, err = p5.communicate();
	#dName = subprocess.check_output("ls ../../Datasets/"+dataset+"/ | grep depth | sort --version-sort | head -n " + str(i) + " | tail -1" , shell = True);

	# Debug printing 
	print(str(i)+"/"+ (numberOfImages.rstrip())+"\t" + rgbName.rstrip() + "\t" + dName.rstrip());
	
	# Copying images
	call("cp ../../Datasets/"+dataset+"/"+str(rgbName.rstrip())+" ./rgb_%0.5d.png" % (i), shell=True);
	call("cp ../../Datasets/"+dataset+"/"+str(dName.rstrip())+ " ./depth_%0.5d.png" % (i), shell=True);

	# Saving matches indices
	myFile.write(str(i) + "\t" + str(i) + "\n")
myFile.close()		
