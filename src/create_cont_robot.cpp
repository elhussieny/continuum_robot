#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <string.h>

using namespace std;
#define NoOfSegments 1
#define NoOfDisks 10
#define Length 5
 ofstream myfile;
int main( int argc, char** argv )
{
  ros::init(argc, argv, "creat_cont_robot");
  std::string path= ros::package::getPath("continuum_robot");  path =path+ "/urdf/robot_model.urdf";
  //printf("PATH: %s \n",path.c_str());
  remove(path.c_str());
  myfile.open (path.c_str(), std::fstream::app);
  myfile << "<?xml version=\"1.0\"?>" <<endl;
  myfile << "<robot name=\"continuum_robot\">"<<endl;
  myfile << "<link name=\"base_link\"/>"<<endl;

  for(int disk=0;disk<NoOfDisks;disk++)
  {
	  myfile << "<link name=\"S0L"<<disk<<"\">"<<endl;
	  myfile << "<visual>"<<endl;
	  myfile << "<geometry>"<<endl;
	  myfile << "<cylinder length=\"0.1\" radius=\"0.5\"/>"<<endl;
	  myfile << "<origin rpy=\"0 0 0\" xyz=\"0 0 "<<(disk/(NoOfDisks-1))*Length<<"\"/>"<<endl;
	  myfile << "</geometry>"<<endl;
	  myfile <<"</visual>"<<endl;
	   myfile <<"</link>"<<endl;
	   myfile <<endl;
	   myfile<<"<joint name=\"S0J"<<disk<<"\" type=\"floating\">"<<endl;
	   myfile<<"<parent link=\"base_link\"/>"<<endl;
      myfile<<"<child link=\"S0L"<<disk<<"\"/>"<<endl;
	      myfile<<"</joint>"<<endl;
	      myfile<<endl;

  }
  myfile <<"</robot>"<<endl;
  myfile.close();


}
