from subprocess import call
import subprocess
import sys

call("cp ../cameraOur.cfg  ../camera.cfg", shell=True);

myFile = open('matchedIndices', 'w')
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
	myFile.write(i + "\t" + str(i) + "\n")
	
	# Saving initial position from gt
	if i==1:
		p1 = subprocess.Popen("cat groundtruth.txt", stdout=subprocess.PIPE, shell=True);
		p2 = subprocess.Popen("head -n 1", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
		p3 = subprocess.Popen("cut -d' ' -f 2-8 > initialPosition", stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);
myFile.close()		
