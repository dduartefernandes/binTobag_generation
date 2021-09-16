# KittibinTobag_generation
This code provides the following features:
* ros integration with pointpillars model in c++ 
* convert kitti bin files for a sequence of frames by publishing it on ros topic passed as argument

cmd fields:
./bin_gen [0] [1] [2] [3], where:
        [0] refers to the desired sampling publishing rate;
        [1] topic where data will be pubished;
        [2] txt file listing the frames for publishing;
        [3] location of the .bin files

cmd example:
./bin_gen 1 /p12_bag eval.txt /home/p12/final_image_dpu/p12_server/Vitis/Vitis-AI/pointpillars/data/KITTI/training/velodyne"

