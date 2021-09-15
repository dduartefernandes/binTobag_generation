#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <chrono>
#include <thread>

//#define SINGLE_FRAME

using namespace std::chrono_literals;

class kitti4ROS
{ 
  private:
    sensor_msgs::PointCloud2 msg;
    ros::NodeHandle n;
    ros::Publisher pub_;
    //ros::Subscriber sub_;
    std::string path_; 
    const uint64_t ros_rate_, number_frames;
    void publish_msg(uint64_t nPoints);
    int count_digit(int number);
  public:
    static uint64_t NFrame_Seq;  
    kitti4ROS(int argc, char** argv, uint64_t value, uint64_t value_):ros_rate_(value), number_frames(value_){   
      pub_ = n.advertise<sensor_msgs::PointCloud2> ("/p12_bag", 1);
      //sub_ = n.subscribe("/p12_detections", 1, &kitti4ROS::cloud_cb, this);
      path_= argv[2];
    }
    static void update_frameNumber(void){
      NFrame_Seq++;
    }

    void cloud_cb (const jsk_recognition_msgs::BoundingBoxArray& BBarray)
    {
      ROS_INFO("Message Received\n");
    }


    void convert_data(void){
      static uint64_t countNPoints=0;
      #ifndef SINGLE_FRAME
        std::ifstream input2(get_sequence_path().c_str(), std::ios::in | std::ios::binary);
      #else
        std::ifstream input2(path_.c_str(), std::ios::in | std::ios::binary);
        std::cout<< "[" << NFrame_Seq << "]: "<< path_ << std::endl;
      #endif
      if(!input2.good() || NFrame_Seq>number_frames){
          ROS_INFO("End of sequence!\n");
          abort();
      }else{
        while (!input2.eof())
        {   
          update_msg(input2.get());
          countNPoints++;
        }
        input2.clear();
        input2.close();
        publish_msg(countNPoints);
        countNPoints=0;
      }
    }  

    static uint64_t get_frameNumber(void)
    {
      return NFrame_Seq;
    }
    std::string get_sequence_path(void)
    {
      std::string string_; 
      std::string aux;
      std::stringstream ss;
      ss << path_;
      ss >> string_;
       
      string_.resize(string_.size()-count_digit(NFrame_Seq)-4);
      ss.clear();
      ss <<NFrame_Seq;
      ss >> aux;
      string_.append(aux); 
      ss.clear();
      string_.append(".bin"); 
      std::cout<< "[" << NFrame_Seq << "]: "<< string_ << std::endl;
      return string_;
    }

    void update_msg(char c)
    {
      msg.data.push_back(c);
    }
};

int kitti4ROS::count_digit(int number)
{
  int count = 0;
  while(number != 0) {
    number = number / 10;
    count++;
  }
  return count;
}

void kitti4ROS::publish_msg(uint64_t nPoints)
{
  sensor_msgs::PointField field;
  msg.data.pop_back();
  msg.header.seq= kitti4ROS::NFrame_Seq;
  msg.header.frame_id="velo_link"; 
  msg.header.stamp.nsec=0;
  msg.header.stamp.sec=0;
  msg.point_step=16;
  msg.height=1;
  msg.row_step=nPoints;
  msg.width=nPoints/16;
  //std::cout << "Bytes " << countNPoints << ", width " << msg.width << std::endl;
  field.name="x";    
  field.offset=0;
  field.datatype=7;
  field.count=1;

  msg.fields.push_back(field);
  
  field.name="y";    
  field.offset=4;
  msg.fields.push_back(field);

  field.name="z";    
  field.offset=8;
  msg.fields.push_back(field);

  field.name="intensity";    
  field.offset=12;
  msg.fields.push_back(field);

  msg.is_bigendian=false;
  msg.is_dense=true;
  
  pub_.publish(msg);
  msg.data.clear();
}

uint64_t kitti4ROS::NFrame_Seq=0;

int main(int argc, char** argv)
{ 
  
  /*if(const char* env_p = std::getenv("PWD"))
      std::cout << "Your PATH is: " << env_p << '\n';
  */
 if (argc!=4)
 {
   std::cout<<"Command example ./bin_gen 1 /home/p12/final_image_dpu/p12_server/bags/KITTI_DATASET_ROOT/training/velodyne/000000.bin 1000" << std::endl;
   abort();
   /* code */
 }
 
  std::stringstream str;  
  str << argv[1]; 
  uint64_t value_rate, nframes;
  str >> value_rate;
  str.clear();
  str<<argv[3];
  str>>nframes;
  
  ros::init(argc, argv, "bin_converter");
  kitti4ROS BIN(argc, argv, value_rate, nframes);
  std::this_thread::sleep_for(5s);
  while (ros::ok())
  { 
    sensor_msgs::PointField field;
    ros::Rate loop_rate(value_rate);
    ros::spinOnce();
    loop_rate.sleep();
    BIN.update_frameNumber();
    BIN.convert_data();
    
  }
  return 0;
}