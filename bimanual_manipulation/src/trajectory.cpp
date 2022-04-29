#include "trajectory.h"

void Trajectory::readTraj(){
  std::string line;

  std::ifstream txt(ros::package::getPath("bimanual_manipulation")+this->file_name_full);
  // check if text file is well opened
  if(!txt.is_open()){
    std::cout << "FILE NOT FOUND\n";
  }

  // first line indicates the size
  /*
  std::getline(txt, line); // drawing size
  std::vector<std::string> tempSplit = split(line, ' ');
  double width = stod(tempSplit[0]);
  double height = stod(tempSplit[1]);
  setDrawingSize(width/height);
*/
  double x, y, z;
  point_t pt, orientation;
  Stroke stroke;
  geometry_msgs::Pose drawing_pose;

  while(std::getline(txt, line)) {
    if(line == "End") { // stroke finished
      this->strokes.push_back(stroke);
      //stroke.clear();
      Stroke().swap(stroke);
    }
    else { // start reading strokes
      point_t().swap(pt);

      tempSplit = split(line, ' ');
      y = (-stod(tempSplit[0])+0.5) * this->ratio * this->target_size;
      z = (-stod(tempSplit[1])+0.5) * this->target_size + this->drawing_pose.position.z;
      pt.push_back(y); pt.push_back(z);

      // std::cout<< "1) Get Quaternion \n";


      tie(x, orientation) = getXAndQuaternion(pt);

      drawing_pose.position.x = x;
      drawing_pose.position.y = y;
      drawing_pose.position.z = z;
      drawing_pose.orientation.x = orientation[0];
      drawing_pose.orientation.y = orientation[1];
      drawing_pose.orientation.z = orientation[2];
      drawing_pose.orientation.w = orientation[3];
      stroke.push_back(drawing_pose);

      // std::cout<< "8) DONE! Got Drawing Pose into stroke\n\n";
    }
  }
}