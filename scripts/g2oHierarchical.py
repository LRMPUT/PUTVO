from subprocess import call
import subprocess
import sys

dataset=sys.argv[1];

 -guess -hi 10 -li 10 -o ../results/graphOpt.g2o ../results/graphFile.g2o

call("../../g2o_trackXYZ/g2o/bin/g2o_hierarchical -i 0 -solver gn_pcg -o ../results/g2o_out_0.g2o ../results/graphFile.g2o", shell=True); # removed "-v"
call("../../g2o_trackXYZ/g2o/bin/g2o_hierarchical -i 70 -guess -solver gn_pcg -o ../results/g2o_out_70.g2o ../results/graphFile.g2o", shell=True);

indicesIn = open("../results/g2oIndices");
datain = open("../results/g2o_out_70.g2o");
dataout = open("../results/g2o_out_70_errorTest.g2o", 'w')
i = 1;

for line in datain:
	
	gtline = [x.strip() for x in line.split(' ')];
	if gtline[0] == "VERTEX_SE3:QUAT":
		
		if int(gtline[1]) < 5000:
			groundTruthIndex = indicesIn.readline().rstrip();
			#dataout.write("1305031" + str(float(gtline[1])/100)+ " ");
			#dataout.write(str(float(gtline[1]))+ " ");			
			
			dataout.write(groundTruthIndex);			
			for x in gtline[2:]:
				dataout.write(" "+ x);
			dataout.write("\n");
dataout.close();
datain.close();

#call("rm ../results/g2oIndices", shell=True);

if "SSRR" in dataset:
	call("python2 ../scripts/evaluate_ate.py ../results/g2o_out_70_errorTest.g2o ../results/g2o_out_70_errorTest.g2o --verbose --scale 1 --plot ../results/g2o_ate.png > ../results/g2o_ate.res ", shell=True); 
else:
	call("python2 ../scripts/evaluate_rpe.py groundtruth.txt ../results/g2o_out_70_errorTest.g2o --interpolate --verbose --delta_unit 'f' --fixed_delta --plot ../results/g2o_rpe.png > ../results/g2o_rpe.res", shell=True);
	call("python2 ../scripts/evaluate_ate.py groundtruth.txt ../results/g2o_out_70_errorTest.g2o --verbose --scale 1 --plot ../results/g2o_ate.png > ../results/g2o_ate.res ", shell=True); 

call("mv export_trans.txt ../results/g2o_transErrors", shell=True);
call("mv export_rot.txt ../results/g2o_rotErrors", shell=True);
