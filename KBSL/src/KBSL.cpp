/**
* Assignment 3 Robust Plane Fitting SOLUTIONS
*/
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "KBSL/TransformPointSrv.h"
#include "KBSL/FitMinimalPlaneSrv.h"
#include "KBSL/FindInliersSrv.h"
#include "KBSL/FitBestPlaneSrv.h"
#include "KBSL/PlaneParametersMsg.h"
// #include "AudioAPI.h"

#include <SFML/Audio.hpp>
#include <math.h>

using Eigen::Matrix3f;
using Eigen::Vector3f;
using geometry_msgs::Point32;
using namespace std;

#define PI 3.14159265


class Sound {

public:
  Sound();
  void scan_to_sound(vector<float>, vector<float>);
  void scan_to_sound_2(vector<float>, vector<float>);
  void set_volume(float);

private:
  sf::SoundBuffer buffer;
  sf::Sound sound;
  void set_pitch(float);
  void set_position(float, float, float);
  void set_play();
  void set_pause();

};
  // initializes the sound object
  // a sin wave is stored in the sound buffer
  // the sound is on a loop
  // volume is set to 0 
  Sound::Sound() {
    const unsigned SAMPLES     = 44100;
    const unsigned SAMPLE_RATE = 44100;
    const unsigned AMPLITUDE   = 30000;
    const double TWO_PI        = 6.28318;
    const double increment     = 440./44100;
    sf::Int16 raw[SAMPLES];
    double x = 0;
    for (unsigned i = 0; i < SAMPLES; i++) {
      raw[i] = AMPLITUDE * sin(x*TWO_PI);
      x += increment;
    }  
    if (!buffer.loadFromSamples(raw, SAMPLES, 1, SAMPLE_RATE)) {
      // error
    }
    sound.setBuffer(buffer);
    sound.setLoop(true);
    sound.setVolume(0.f);
    sound.play();
    sf::Listener::setPosition(0.f, 0.f, 0.f);
    sf::Listener::setDirection(1.f, 0.f, 0.f);
    sf::Listener::setGlobalVolume(0.f);
  }
  // changes the volume of the sound object
  // takes a float as parameter
  // volume ranges from 0-100
  void Sound::set_volume(float volume) {
    sound.setVolume(volume);
    sf::Listener::setGlobalVolume(volume);
  }
  // changes the pitch of the sound object
  // takes a float as parameter
  // 1.0 will bring it back to the original sound
  void Sound::set_pitch(float pitch) {
    sound.setPitch(pitch);
  }
  // sets the position of the sound object
  // takes three floats as parameters
  // +x is in front of the listener
  // +y is above the listener
  // +z is to the right of the listener
  void Sound::set_position(float x, float y, float z) {
    sound.setPosition(x, y, z);
  }
  // resumes playing the sound object
  void Sound::set_play() {
    sound.play();
  }
  // pauses the sound object
  void Sound::set_pause() {
    sound.pause();
  }
  // angle -0.7, +0.7 in rad
  // distance in meters
  void Sound::scan_to_sound(vector<float> angle, vector<float> distance) {  
    int max_index = 0;
    float max_distance = 0;
    for (int i = 0; i < (int)distance.size(); i++) {
      if (distance.at(i) > max_distance) {
        max_index = i;
        max_distance = distance.at(i);
      }
    }
    if (max_index == 0) {
      sound.setPosition(0.f, 0.f, -10.f);
      sound.setVolume(100.f);
      sf::Listener::setGlobalVolume(100.f);
    }
    else if(max_index == 1) {
      sound.setPosition(0.f, 0.f, -5.f);
      sound.setVolume(50.f);
      sf::Listener::setGlobalVolume(50.f);
    }
    else if(max_index == 2) {
      sound.setPosition(0.f, 0.f, 0.f);
      sound.setVolume(0.f);
      sf::Listener::setGlobalVolume(0.f);
    }
    else if(max_index == 3) {
      sound.setPosition(0.f, 0.f, 5.f);
      sound.setVolume(50.f);
      sf::Listener::setGlobalVolume(50.f);
    }
    else if(max_index == 4) {
      sound.setPosition(0.f, 0.f, 10.f);
      sound.setVolume(100.f);
      sf::Listener::setGlobalVolume(100.f);
    }
    else {
      //error
      sf::Listener::setPosition(0.f, 0.f, 0.f);
      sound.setVolume(0.f);
      sf::Listener::setGlobalVolume(0.f);
    }
  }

  void Sound::scan_to_sound_2(vector<float> angle, vector<float> distance) {  

    for (int i = 0; i < (int)distance.size(); i++) {

      if (distance.at(i) > 10) distance.at(i) = 10;
      if (distance.at(i) < 1) distance.at(i) = 1;

      if (i == 0) {
        sound.setPosition(0.f, 0.f, -10.f);
        sound.setVolume(100.f - distance.at(i) * 8);
        sf::Listener::setGlobalVolume(100.f - distance.at(i) * 8);
        sleep(0.5);
      }
      else if(i == 1) {
        sound.setPosition(0.f, 0.f, -5.f);
        sound.setVolume(100.f - distance.at(i) * 8);
        sf::Listener::setGlobalVolume(100.f - distance.at(i) * 8);
        sleep(0.5);
      }
      else if(i == 2) {
        sound.setPosition(0.f, 0.f, 0.f);
        sound.setVolume(100.f - distance.at(i) * 8);
        sf::Listener::setGlobalVolume(100.f - distance.at(i) * 8);
        sleep(0.5);
      }
      else if(i == 3) {
        sound.setPosition(0.f, 0.f, 5.f);
        sound.setVolume(100.f - distance.at(i) * 8);
        sf::Listener::setGlobalVolume(100.f - distance.at(i) * 8);
        sleep(0.5);
      }
      else if(i == 4) {
        sound.setPosition(0.f, 0.f, 10.f);
        sound.setVolume(100.f - distance.at(i) * 8);
        sf::Listener::setGlobalVolume(100.f - distance.at(i) * 8);
        sleep(0.5);
      }
      

    }

    // sound.setVolume(0.f);
    // sf::Listener::setGlobalVolume(0.f);
    // sleep(0.2);



  }






/*-----------------------------------------End Sound----------------------------------------------*/

/*-----------------------------------------Begin Obs----------------------------------------------*/










//USER PARAMETERS
double kinect_angle = 20.0;
double kinect_height = 1.3;
double user_width = 0.2;
double user_height = 1.7;

float max_sensor_dist = 100;
float min_sensor_dist = 1;

Vector3f floor_norm(1.0,0,0);
float norm_tol = 1.0;
float max_ground = -.9*kinect_height;


sensor_msgs::PointCloud full_point_cloud;
// Message for published filtered 3D point clouds.
sensor_msgs::PointCloud ground_point_cloud;
sensor_msgs::PointCloud obstacles_point_cloud;

float x_min = max_sensor_dist;
float x_max = -1*max_sensor_dist;
float y_min = max_sensor_dist;
float y_max = -1*max_sensor_dist;

//DownSampled PointCloud ----- Convert These to Sound
vector<float> angles;
vector<float> angle_distances;

// Publisher for 3D plane filtered point clouds.
ros::Publisher filtered_point_cloud_publisher_;
ros::Publisher filtered_point_cloud_publisher_2;
ros::Publisher filtered_point_cloud_publisher_3;

// RANSAC parameters.
static const int kIterations = 55;

static const float kMinInlierFraction = 0.7;
static const float kEpsilonDetect = 0.03;
static const float kEpsilonCollect = 0.08;

// static const float kMinInlierFraction = 0.7;
// static const float kEpsilon = 0.05;

struct Rect{
  Rect(float a, Vector3f near, Vector3f far){
    ang = a;
    nearPnt = near;
    farPnt = far;
  }
  float ang;
  Vector3f nearPnt;
  Vector3f farPnt;
};

bool npInRange(Vector3f n, Vector3f p){
  //TODO
  //if angle between n and floor_norm > norm_tol return false
  //else if p.z < z_max ground return false
  //else return true
  return true;
}

// Helper function to convert ROS Point32 to Eigen Vectors.
Vector3f ConvertPointToVector(const Point32& point) {
  return Vector3f(point.x, point.y, point.z);
}

// Helper function to convert Eigen Vectors to ROS Point32.
Point32 ConvertVectorToPoint(const Vector3f& vector) {
  Point32 point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

void FitMinimalPlane(const Vector3f& P1, const Vector3f& P2, const Vector3f& P3,
                     Vector3f* n, Vector3f* P0) {
  *P0 = P1;
  const Vector3f P21 = P2 - P1;
  const Vector3f P31 = P3 - P1;
  *n = (P21.cross(P31)).normalized();
}

void FindInliers(const Vector3f& n, const Vector3f& P0, float epsilon,
    const vector<Vector3f>& point_cloud, vector<Vector3f>* inliers) {

  inliers->clear();
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    if (fabs((point_cloud[i] - P0).dot(n)) < epsilon) {
      inliers->push_back(point_cloud[i]);
    }
  }
}

void FindInliersFloorObs(const Vector3f& n, const Vector3f& P0, float epsilon,
    const vector<Vector3f>& point_cloud, vector<Vector3f>* inliers, vector<Vector3f>* outliers) {

  x_min = max_sensor_dist;
  x_max = -1* max_sensor_dist;
  y_min = max_sensor_dist;
  y_max = -1* max_sensor_dist;

    

  inliers->clear();
  for (size_t i = 0; i < point_cloud.size(); ++i) {

    if(point_cloud[i].x() < x_min) x_min = point_cloud[i].x();
    if(point_cloud[i].x() > x_max) x_max = point_cloud[i].x();
    if(point_cloud[i].y() < y_min and point_cloud[i].y() > min_sensor_dist) y_min = point_cloud[i].y();
    if(point_cloud[i].y() > y_max) y_max = point_cloud[i].y();

    if (fabs((point_cloud[i] - P0).dot(n)) < epsilon) {
      inliers->push_back(point_cloud[i]);
    }
    else{
      outliers->push_back(point_cloud[i]);
    }
  }
}

Vector3f PickRandomPoint(const vector<Vector3f>& point_cloud) {
  if(point_cloud.size() == 0){
    return Vector3f();
  }
  else{
    const int i = rand() % (point_cloud.size());
    return point_cloud[i];
  }
}

Vector3f GetSmallestEigenVector(const Matrix3f& M) {
  // Initialize the eigenvalue solver.
  Eigen::EigenSolver<Matrix3f> eigen_solver;
  // Ask the solver to compute the eigen values and vectors.
  eigen_solver.compute(M);
  // Get all the eigenvalues. Note the result is complex-valued, and we are
  // asking for only the real component by calling ".real()" over the result.
  Vector3f eigen_values = eigen_solver.eigenvalues().real();
  // Get all the eigenvectors. Note the result is complex-valued, and we are
  // asking for only the real component by calling ".real()" over the result.
  Matrix3f eigen_vectors = eigen_solver.eigenvectors().real();

  float smallest_eigenvalue = eigen_values(0);
  Vector3f smallest_eigenvector = eigen_vectors.col(0);
  for (int i = 1; i < 3; ++i) {
    if (eigen_values(i) < smallest_eigenvalue) {
      smallest_eigenvalue = eigen_values(i);
      smallest_eigenvector = eigen_vectors.col(i);
    }
  }
  return (smallest_eigenvector.normalized());
}

void FitBestPlane(const vector<Vector3f>& point_cloud, Vector3f* n_ptr,
                  Vector3f* P0_ptr) {
  // Alias of n.
  Vector3f& n = *n_ptr;
  // Alias of P0.
  Vector3f& P0 = *P0_ptr;

  // Find P0.
  P0.setZero();
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    P0 += point_cloud[i];
  }
  P0 = P0 / static_cast<float>(point_cloud.size());

  // Build M matrix.
  Matrix3f M;
  M.setZero();
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    M = M + (point_cloud[i] - P0) * (point_cloud[i] - P0).transpose();
  }
  n = GetSmallestEigenVector(M);
}

void RANSAC_MOD(const vector<Vector3f>& point_cloud, Vector3f* n_ptr,
            Vector3f* P0_ptr) {

  vector<Vector3f> near_floor_pnts;
  // Copy over the input point cloud - filtering out points above y-max
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    if(point_cloud[i].z() < max_ground){
      near_floor_pnts.push_back(point_cloud[i]);
    }
  }

  // Alias of n.
  Vector3f& n = *n_ptr;
  // Alias of P0.
  Vector3f& P0 = *P0_ptr;

  // Run RANSAC.
  vector<Vector3f> inliers;
  float inlier_fraction = 0;
  int i = 0;
  do {
    ++i;
    const Vector3f P1 = PickRandomPoint(near_floor_pnts);
    const Vector3f P2 = PickRandomPoint(near_floor_pnts);
    const Vector3f P3 = PickRandomPoint(near_floor_pnts);
    FitMinimalPlane(P1, P2, P3, &n, &P0);
    FindInliers(n, P0, kEpsilonDetect, near_floor_pnts, &inliers);
    inlier_fraction = static_cast<float>(inliers.size()) /
        static_cast<float>(near_floor_pnts.size());
  } while (i < kIterations && inlier_fraction < kMinInlierFraction && npInRange(*n_ptr, *P0_ptr));
  FitBestPlane(inliers, n_ptr, P0_ptr);
}

void PointCloudCallback(const sensor_msgs::PointCloud2& point_cloud_2_msg) {
  //TODO perform rotation
  //Perform transformation

  
  double x_rot = 90.0 + kinect_angle;

  sensor_msgs::PointCloud point_cloud_msg;

  bool b = sensor_msgs::convertPointCloud2ToPointCloud(point_cloud_2_msg, point_cloud_msg);

  if(not b) ROS_INFO("Error on PointCloud2 to PointCloud Conversion");

  // filtered_point_cloud_publisher_.publish(point_cloud_msg);

	point_cloud_msg.header = point_cloud_2_msg.header;

  sensor_msgs::PointCloud trans_pnt_cloud;
  trans_pnt_cloud.header = point_cloud_msg.header;


  Matrix3f R;
  //Y-Rotation

    // R(0, 0) = cos(x_rot*PI/180.0);
    // R(0, 1) = 0;
    // R(0, 2) = -1*sin(x_rot*PI/180.0);
    // R(1, 0) = 0;
    // R(1, 1) = 1;
    // R(1, 2) = 0;
    // R(2, 0) = sin(x_rot*PI/180.0);
    // R(2, 1) = 0;
    // R(2, 2) = cos(x_rot*PI/180.0);

    R(0, 0) = 1;//cos(x_rot*PI/180.0);
    R(0, 1) = 0;
    R(0, 2) = 0;//-1*sin(x_rot*PI/180.0);
    R(1, 0) = 0;
    R(1, 1) = cos(x_rot*PI/180.0);
    R(1, 2) = sin(x_rot*PI/180.0);
    R(2, 0) = 0;//sin(x_rot*PI/180.0);
    R(2, 1) = -1*sin(x_rot*PI/180.0);
    R(2, 2) = cos(x_rot*PI/180.0);

  // R(0, 0) = 1.0;
  // R(1, 1) = 1.0;
  // R(2, 2) = 1.0;


  const Vector3f T(0.0, 0.0, 0.0);
  // const Vector3f T(0.0, 0.0, kinect_height);

  for(int i = 0; i<(int)point_cloud_msg.points.size(); i++){
    Vector3f P(point_cloud_msg.points[i].x, point_cloud_msg.points[i].y, point_cloud_msg.points[i].z);
    // Compute P_prime from P, R, T.
    Vector3f P_prime = R * P + T;

    Point32 new_point;
    new_point.x = P_prime.x();
    new_point.y = P_prime.y();
    new_point.z = P_prime.z();

    trans_pnt_cloud.points.push_back(new_point);
  }


  full_point_cloud = trans_pnt_cloud;
  // filtered_point_cloud_publisher_2.publish(full_point_cloud);

  // filtered_point_cloud_publisher_.publish(full_point_cloud);

}

void updateClouds(){
  
  ground_point_cloud.header = full_point_cloud.header;
  obstacles_point_cloud.header = full_point_cloud.header;

  // Create a Vector3f point cloud, of the same size as the input point cloud.
  vector<Vector3f> point_cloud(full_point_cloud.points.size());

  // Copy over the input point cloud.
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    point_cloud[i] = ConvertPointToVector(full_point_cloud.points[i]);
  }

  vector<Vector3f> filtered_point_cloud_ground;
  vector<Vector3f> filtered_point_cloud_obstacles;
  Vector3f n;
  Vector3f P0;
  // Extract filtered point cloud.
  RANSAC_MOD(point_cloud, &n, &P0);
  FindInliersFloorObs(n, P0, kEpsilonCollect, point_cloud, &filtered_point_cloud_ground, &filtered_point_cloud_obstacles);

  // Copy over the output for ground points.
  ground_point_cloud.points.resize(filtered_point_cloud_ground.size());
  for (size_t i = 0; i < filtered_point_cloud_ground.size(); ++i) {
    ground_point_cloud.points[i] =
        ConvertVectorToPoint(filtered_point_cloud_ground[i]);
  }

  //Copy over the output for obstacle points
  obstacles_point_cloud.points.resize(filtered_point_cloud_obstacles.size());
  for (size_t i = 0; i < filtered_point_cloud_obstacles.size(); ++i) {
    obstacles_point_cloud.points[i] =
        ConvertVectorToPoint(filtered_point_cloud_obstacles[i]);
  }

  //TEST WHAT THE POINT CLOUD CONTAIN
  filtered_point_cloud_publisher_.publish(ground_point_cloud);
  filtered_point_cloud_publisher_2.publish(obstacles_point_cloud);

}



void obsAvoid(){

  Vector3f origin = Vector3f(0,0,0);

  int numPaths = 5;

  float min_ang = -24.0*(PI/180.0);
  float max_ang = 24.0*(PI/180.0);

  float ang_diff = max_ang - min_ang;
  if(ang_diff < 0) ang_diff *= -1;
  float ang_unit = ang_diff/(float)numPaths;

  Vector3f near = Vector3f(0, origin.y() - (user_width/(float)2), origin.z() - kinect_height);
  Vector3f far = Vector3f(0, origin.y() + (user_width/(float)2), origin.z() + (user_height - kinect_height));

  vector<float> ang_vec;
  vector<float> dist_vec;

  for(int i = 0; i<numPaths; i++){
    ang_vec.push_back(min_ang + ((float)i*ang_unit) + (0.5* ang_unit));
    dist_vec.push_back(max_sensor_dist);
  }

  float rot_init = min_ang - (ang_unit*0.5);

  sensor_msgs::PointCloud temp_cloud = obstacles_point_cloud;

  //Obstancle Avoidance
  for (int i = 0; i<(int)temp_cloud.points.size(); i++){
    Vector3f currPnt = ConvertPointToVector(temp_cloud.points[i]);
    currPnt = Vector3f(currPnt.y(), currPnt.x(), currPnt.z());

    // ROS_INFO("OG-\n x:%f y:%f z:%f", currPnt.x(), currPnt.y(), currPnt.z());

    float x = currPnt.x() * cos(rot_init) + currPnt.y() * sin(rot_init);
    float y = -1*currPnt.x() * sin(rot_init) + currPnt.y() * cos(rot_init);
    currPnt = Vector3f(x, y, currPnt.z()); //R0 * currPnt;

    for(int j = 0; j<(int)ang_vec.size(); j++){
      // ROS_INFO("R%i-\n x:%f y:%f z:%f", j, currPnt.x(), currPnt.y(), currPnt.z());
      x = currPnt.x() * cos(ang_unit) + currPnt.y() * sin(ang_unit);
      y = -1*currPnt.x() * sin(ang_unit) + currPnt.y() * cos(ang_unit);
      currPnt = Vector3f(x, y, currPnt.z()); //R0 * currPnt;
      //check if x,y parameters are within rectangle
      if( currPnt.x() < dist_vec[j] and currPnt.y() > near.y() and currPnt.y() < far.y() and currPnt.z() > near.z() and currPnt.z() < far.z()){
        dist_vec[j] = currPnt.x();
      }
    }

  }

  // Construct Ground Obstacle Grid
  // Z<0 -> No Obstacle else Obstacle

  int num_ground_points = 15;
  int obsArr[num_ground_points][num_ground_points] = {0};
  vector<Vector3f> ground_obs;

  float x_ground_diff = x_max - x_min;
  float y_ground_diff = y_max - y_min;

  float x_ground_step = x_ground_diff/(float)num_ground_points;
  float y_ground_step = y_ground_diff/(float)num_ground_points;
  temp_cloud = ground_point_cloud;

  temp_cloud = ground_point_cloud;
  for(int i = 0; i<(int)temp_cloud.points.size(); i++){
    Vector3f currPnt = ConvertPointToVector(temp_cloud.points[i]);
    int x_ind = (int)((currPnt.x()-x_min)/x_ground_step);
    int y_ind = (int)((currPnt.y()-y_min)/y_ground_step);
    
    if(x_ind >= num_ground_points) x_ind = num_ground_points-1;
    if(y_ind >= num_ground_points) y_ind = num_ground_points-1;

    obsArr[x_ind][y_ind] = 1;
  }

  for(int i = 0; i<num_ground_points; i++){
    for(int j = 0; j<num_ground_points; j++){
      float z_val = -1.0;
      if(obsArr[i][j]>0) z_val = 1.0;
      Vector3f new_pnt = Vector3f(x_min+(0.5*x_ground_step)+(i*x_ground_step), y_min+(0.5*y_ground_step)+(j*y_ground_step), z_val);
      ground_obs.push_back(new_pnt);
    }
  }

  //Hole In Ground Avoidance
  for (int i = 0; i<(int)ground_obs.size(); i++){
    Vector3f currPnt = ground_obs[i];
    currPnt = Vector3f(currPnt.y(), currPnt.x(), currPnt.z());

    // ROS_INFO("OG-\n x:%f y:%f z:%f", currPnt.x(), currPnt.y(), currPnt.z());

    float x = currPnt.x() * cos(rot_init) + currPnt.y() * sin(rot_init);
    float y = -1*currPnt.x() * sin(rot_init) + currPnt.y() * cos(rot_init);
    currPnt = Vector3f(x, y, currPnt.z()); //R0 * currPnt;

    for(int j = 0; j<(int)ang_vec.size(); j++){
      // ROS_INFO("R%i-\n x:%f y:%f z:%f", j, currPnt.x(), currPnt.y(), currPnt.z());
      x = currPnt.x() * cos(ang_unit) + currPnt.y() * sin(ang_unit);
      y = -1*currPnt.x() * sin(ang_unit) + currPnt.y() * cos(ang_unit);
      currPnt = Vector3f(x, y, currPnt.z()); //R0 * currPnt;
      //check if x,y parameters are within rectangle
      if( currPnt.x() < dist_vec[j] and currPnt.z() < 0.0 and currPnt.x() >= min_sensor_dist and currPnt.y() > near.y() and currPnt.y() < far.y() and currPnt.z() > near.z() and currPnt.z() < far.z()){
        dist_vec[j] = currPnt.x();
      }
    }

  }



  //TESTING

  sensor_msgs::PointCloud temp_pnt_cloud_pub = obstacles_point_cloud;

  temp_pnt_cloud_pub.points.resize(ground_obs.size());
  for (size_t i = 0; i < ground_obs.size(); ++i) {
    temp_pnt_cloud_pub.points[i] =
        ConvertVectorToPoint(ground_obs[i]);
  }

  filtered_point_cloud_publisher_3.publish(temp_pnt_cloud_pub);

  //END TESTING



  angle_distances = dist_vec;
  angles = ang_vec;

  float test = angle_distances[0];
  printf("1: %f\n", test);
  test = angle_distances[1];
  printf("2: %f\n", test);
  test = angle_distances[2];
  printf("3: %f\n", test);
  test = angle_distances[3];
  printf("4: %f\n", test);
  test = angle_distances[4];
  printf("5: %f\n\n", test);

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "KBSL");
  ros::NodeHandle n;

  filtered_point_cloud_publisher_ = n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/FilteredPointCloud", 3);
  filtered_point_cloud_publisher_2 = n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/PCloud", 3);
  filtered_point_cloud_publisher_3 = n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/FloorGrid", 3);

  ros::Subscriber point_cloud_subscriber =

    // n.subscribe("/COMPSCI403/PointCloud", 3, PointCloudCallback);
    n.subscribe("/camera/depth/points", 3, PointCloudCallback); 


    Sound sound;

    ros::Rate loop(2);
    while (ros::ok()) {

      //Call Function To update floor and obstacle point clouds

      updateClouds();
      obsAvoid();

      sound.scan_to_sound_2(angles, angle_distances);

      ros::spinOnce();
      loop.sleep();
    }

  return 0;
}
