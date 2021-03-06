from subprocess import call
import subprocess
import sys
import glob
import os
from math import ceil

call("rm test", shell=True); 
call("touch test", shell=True); 

call("rm table", shell=True); 
call("touch table", shell=True); 

prevDatasetName = "";
for root, dirs, files in os.walk("../SSRR/"):
	rpeTra = "";
	rpeRot = "";
	ateTra = "";
	time = "";
	
	for file in files:
		if 'g2o_rpe.res' in file:
			call("echo \"" + root + "\" >> test" , shell=True); 
			call("cat " + root +"/" + file + " | grep translational_error.rmse |  sed 's/translational_error.rmse/RPE.rmse/'>> test" , shell=True); 	
			call("cat " + root +"/" + file + " | grep rotational_error.rmse |  sed 's/rotational_error.rmse/RPE.rmse/'>> test" , shell=True);


			p1 = subprocess.Popen("cat " + root +"/" + file, stdout=subprocess.PIPE, shell=True);
			p2 = subprocess.Popen("grep translational_error.rmse", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
			p3 = subprocess.Popen("sed 's/translational_error.rmse //'", stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);
			p4 = subprocess.Popen("sed 's/ m//'", stdin=p3.stdout, stdout=subprocess.PIPE, shell=True);	
			p5 = subprocess.Popen("head -c -1", stdin=p4.stdout, stdout=subprocess.PIPE, shell=True);		
			rpeTra, err = p5.communicate();
			rpeTra = str(round( float(rpeTra.rstrip()), 3));

			
		     	p1 = subprocess.Popen("cat " + root +"/" + file, stdout=subprocess.PIPE, shell=True);
			p2 = subprocess.Popen("grep rotational_error.rmse", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
			p3 = subprocess.Popen("sed 's/rotational_error.rmse //'", stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);	
			p4 = subprocess.Popen("sed 's/ deg//'", stdin=p3.stdout, stdout=subprocess.PIPE, shell=True);	
			p5 = subprocess.Popen("head -c -1", stdin=p4.stdout, stdout=subprocess.PIPE, shell=True);	
			rpeRot, err = p5.communicate();
			rpeRot = str(round( float(rpeRot.rstrip()), 3));

		if 'timeAndLC' in file: 
		     	call("echo \"Time: \" | head -c -1 >> test" , shell=True);
		    	call("cat " + root +"/" + file + " | head -n 1 >> test" , shell=True);	
		
		     	p1 = subprocess.Popen("cat " + root +"/" + file, stdout=subprocess.PIPE, shell=True);
			p2 = subprocess.Popen("head -n 1", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
			p3 = subprocess.Popen("head -c -1", stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);
			time, err = p3.communicate();	
			

		if 'g2o_ate.res' in file: 
		     	call("cat " + root +"/" + file + " | grep absolute_translational_error.rmse |  sed 's/absolute_translational_error.rmse/ATE.rmse/' >> test" , shell=True); 

		     	p1 = subprocess.Popen("cat " + root +"/" + file, stdout=subprocess.PIPE, shell=True);
			p2 = subprocess.Popen("grep absolute_translational_error.rmse", stdin=p1.stdout, stdout=subprocess.PIPE, shell=True);
			p3 = subprocess.Popen("sed 's/absolute_translational_error.rmse //'", stdin=p2.stdout, stdout=subprocess.PIPE, shell=True);	
			p4 = subprocess.Popen("sed 's/ m//'", stdin=p3.stdout, stdout=subprocess.PIPE, shell=True);	
			p5 = subprocess.Popen("head -c -1", stdin=p4.stdout, stdout=subprocess.PIPE, shell=True);	
			ateTra, err = p5.communicate();
			ateTra = str(round( float(ateTra), 3));
			
			tmp = root.split("/");
			if tmp[2] not in prevDatasetName:
				call("echo \"\n\t" + tmp[2] + "\"  >> table" , shell=True); 

			datasetSize = 1;
			if tmp[2] in "rgbd_dataset_freiburg1_desk":
				datasetSize = 573;
			if tmp[2] in "rgbd_dataset_freiburg1_room":
				datasetSize = 1352;
			if tmp[2] in "living_room_2":
				datasetSize = 882;
			if tmp[2] in "office_traj0":
				datasetSize = 1510;
			if tmp[2] in "living_room_0":
				datasetSize = 1510;
			time = str(round( datasetSize / float(time) , 2));


			if tmp[3] == "MatchingFastBrief":
				tmp[3] = "Matching FAST-BRIEF";
			if tmp[3] == "MatchingFastBriefBA":
				tmp[3] = "Matching+BA F-B";
			if tmp[3] == "MatchingFastBriefBALC":
				tmp[3] = "Matching+BA F-B with LC";

			#elif tmp[3] in "Tracking":

			elif tmp[3] == "TrackingBA":
				tmp[3] = "Tracking+BA";

			elif tmp[3] == "MatchingORB":
				tmp[3] = "Matching ORB";
			elif tmp[3] == "MatchingORBBA":		
				tmp[3] = "Matching+BA ORB";
			elif tmp[3] == "MatchingORBBALC":
				tmp[3] = "Matching+BA ORB with LC";

			elif tmp[3] == "MatchingSurf":
				tmp[3] = "Matching SURF";
			elif tmp[3] == "MatchingSurfBA":
				tmp[3] = "Matching+BA SURF"
			elif tmp[3] == "MatchingSurfBALC":
				tmp[3] = "Matching+BA SURF with LC";

			elif tmp[3] == "TrackingLC_FASTBRIEF":
				tmp[3] = "Tracking with LC F-B";
			elif tmp[3] == "TrackingLC_ORB":
				tmp[3] = "Tracking with LC ORB";
			elif tmp[3] == "TrackingLC_SURF":
				tmp[3] = "Tracking with LC SURF";

			elif tmp[3] == "TrackingBALC_FASTBRIEF":
				tmp[3] = "Tracking+BA with LC F-B"; 
			elif tmp[3] == "TrackingBALC_ORB":
				tmp[3] = "Tracking+BA with LC ORB";
			elif tmp[3] == "TrackingBALC_SURF":
				tmp[3] = "Tracking+BA with LC SURF";	
 

			call("echo \"" + tmp[3] + " & " + ateTra + " & " + rpeTra + " & " + rpeRot + " & " + time + " \" >> table" , shell=True); 
			prevDatasetName = tmp[2];



call("mv test ../ACCV/results", shell=True); 
call("mv table ../ACCV/table", shell=True); 
