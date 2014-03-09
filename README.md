PUTVO
=====

PUT Visual Odometry for RGBD data presented, e.g., at ECMR 2013

-> Dependencies

Eigen
OpenCv
PCL
OpenNI
g2o (can be used, but tracking can provide results without it)

-> How to compile

mkdir build
cmake ..
make (-j4 to compile parallerly on 4 cores)

-> How to run

My structure of directories:
 /home/Projects/PUTVO/build
 /home/Projects/Datasets/

To prepare datasets to run, you can use python script saved in scripts/prepareDatasetFreiburg.py or scripts/prepareDatasetOur.py, e.g., by calling:
python2 ../scripts/prepareDatasetFreiburg.py freiburg1_desk
from the build directory in case of a freiburg1_desk dataset put in directory /home/Projects/Datasets/freiburg1_desk/. 
The datasets has to be put there manually - no automatic data download is performed.

Program is run by the python script runProgram.py, e.g.:
python2 ../scripts/runProgram.py our 
or
python2 ../scripts/runProgram.py freiburg
[the names are necessary to run with proper depth scaling of datasets]

Program running parameters can be set in .cfg file parameters.cfg, which is located in main project directory.

The results are automatically copied into the result directory. The *.rpe results are relative pose error results (erros between cosecutive frames). The *.ate are the absolute trajectory errors.

The g2o is run by fast workaround - the result's from tracking are stored in file, the g2o optimizer is called on this file, and then data is parsed to fit the freiburg format of evaluation.
I run the g2o from the directory /home/Projects/PUTSLAM/PUTSLAM/3rdParty/g2o/bin

