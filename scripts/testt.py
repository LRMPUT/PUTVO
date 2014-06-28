from subprocess import call
import subprocess
import sys
dataset=sys.argv[1];

rgbNames = subprocess.check_output("ls ../../Datasets/"+dataset+"/ | grep image | sort --version-sort" , shell = True);
rgbNames = rgbNames.rstrip().rsplit('\n');

dNames = subprocess.check_output("ls ../../Datasets/"+dataset+"/ | grep depth | sort --version-sort" , shell = True);
dNames = dNames.rstrip().rsplit('\n');
print(len(rgbNames))
print(len(dNames))

