from subprocess import call
import subprocess
import sys


indicesIn = open("../MScResults/matchedIndices");
datain = open("../MScResults/TrajectoryTest100.log");
dataout = open("../MScResults/TrajectoryTest100.eval", 'w')
i = 1;

for line in datain:
	gtline = [x.strip() for x in line.split(' ')];

	groundTruthIndex = indicesIn.readline().rstrip();
	xxx = [x.strip() for x in groundTruthIndex.split('\t')];	
				
	dataout.write(xxx[0]);			
	for x in gtline[1:]:
		dataout.write(" "+ x);
	dataout.write("\n");
dataout.close();
datain.close();


# Running evaluation
call("python2 ../scripts/evaluate_ate.py ../MScResults/groundtruth.txt ../MScResults/TrajectoryTest100.eval --verbose --scale 0.5 --save_associations ate_association.res --plot ../MScResults/ate.png > ate.res", shell=True); 
call("python2 ../scripts/evaluate_rpe.py ../MScResults/groundtruth.txt ../MScResults/TrajectoryTest100.eval --interpolate --scale 0.5 --verbose --delta_unit 'f' --fixed_delta --plot ../MScResults/rpe.png  > rpe.res ", shell=True);
call("mv ate_association.res ../results/ate_association.res", shell=True);
call("mv ate.res ../MScResults/ate.res", shell=True);
call("mv rpe.res ../MScResults/rpe.res", shell=True);
call("mv export_trans.txt ../MScResults/transErrors", shell=True);
call("mv export_rot.txt ../MScResults/rotErrors", shell=True);
