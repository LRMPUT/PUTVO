from subprocess import call
import subprocess
import sys

call("./../../PUTSLAM2/PUTSLAM/3rdParty/g2o/bin/g2o -i 60 -guess -v -solver gn_pcg -o ../results/tracking2.g2o ../results/tracking.g2o", shell=True);

datain = open("../results/tracking2.g2o");
dataout = open("../results/tracking3.g2o", 'w')
i = 1;
for line in datain:
	gtline = [x.strip() for x in line.split(' ')];
	if gtline[0] == "VERTEX_SE3:QUAT":
		if int(gtline[1]) < 5000:
			for x in gtline[1:]:
				dataout.write(x + " ");
			dataout.write("\n");
dataout.close();
datain.close();


call("python2 ../scripts/evaluate_rpe.py groundtruth.txt ../results/tracking3.g2o --interpolate --verbose --delta_unit 'f' --fixed_delta --save rpe_error --plot ../results/rpe_g2o.png > ../results/g2o_rpe.res", shell=True);
call("python2 ../scripts/evaluate_ate.py groundtruth.txt ../results/tracking3.g2o --verbose --scale 1 --plot ../results/ate_g2o.png ", shell=True); #> ../results/g2o_ate.res
