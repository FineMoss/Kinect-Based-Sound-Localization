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
#include <sensor_msgs/PointCloud.h>

#include "KBSL/TransformPointSrv.h"
#include "KBSL/FitMinimalPlaneSrv.h"
#include "KBSL/FindInliersSrv.h"
#include "KBSL/FitBestPlaneSrv.h"
#include "KBSL/PlaneParametersMsg.h"

using Eigen::Matrix3f;
using Eigen::Vector3f;
using geometry_msgs::Point32;
using namespace std;

// Publisher for 3D plane filtered point clouds.
ros::Publisher filtered_point_cloud_publisher_;

// Publisher for plane parameters.
ros::Publisher plane_parameters_publisher_;

// Message for published filtered 3D point clouds.
sensor_msgs::PointCloud filtered_point_cloud_msg;

// RANSAC parameters.
static const int kIterations = 10;
static const float kMinInlierFraction = 0.7;
static const float kEpsilon = 0.01;

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

bool TransformPointService(
    KBSL::TransformPointSrv::Request& req,
    KBSL::TransformPointSrv::Response& res) {
  const Vector3f P(req.P.x, req.P.y, req.P.z);

  Matrix3f R;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      R(row, col) = req.R[col * 3 + row];
    }
  }
  const Vector3f T(req.T.x, req.T.y, req.T.z);

  // Compute P_prime from P, R, T.
  Vector3f P_prime = R * P + T;

  // Convert to ROS type to return the result.
  res.P_prime = ConvertVectorToPoint(P_prime);
  return true;
}

void FitMinimalPlane(const Vector3f& P1, const Vector3f& P2, const Vector3f& P3,
                     Vector3f* n, Vector3f* P0) {
  *P0 = P1;
  const Vector3f P21 = P2 - P1;
  const Vector3f P31 = P3 - P1;
  *n = (P21.cross(P31)).normalized();
}

bool FitMinimalPlaneService(
    KBSL::FitMinimalPlaneSrv::Request& req,
    KBSL::FitMinimalPlaneSrv::Response& res) {

  const Vector3f P1 = ConvertPointToVector(req.P1);
  const Vector3f P2 = ConvertPointToVector(req.P2);
  const Vector3f P3 = ConvertPointToVector(req.P3);

  // Compute n and P0 from P1, P2, P3.
  Vector3f n;
  Vector3f P0;
  FitMinimalPlane(P1, P2, P3, &n, &P0);

  res.n = ConvertVectorToPoint(n);
  res.P0 = ConvertVectorToPoint(P0);
  return true;
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

bool FindInliersService(
    KBSL::FindInliersSrv::Request& req,
    KBSL::FindInliersSrv::Response& res) {

  const Vector3f n = ConvertPointToVector(req.n);
  const Vector3f P0 = ConvertPointToVector(req.P0);
  const float epsilon = req.epsilon;

  // Copy over all the points.
  vector<Vector3f> point_cloud(req.P.size());
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    point_cloud[i] = ConvertPointToVector(req.P[i]);
  }

  vector<Vector3f> inliers;
  // Compute the inliers from n, P0, epsilon, and the points in the PointCloud
  FindInliers(n, P0, epsilon, point_cloud, &inliers);

  res.P.resize(inliers.size());
  for (size_t i = 0; i < inliers.size(); ++i) {
    res.P[i] = ConvertVectorToPoint(inliers[i]);
  }
  return true;
}

Vector3f PickRandomPoint(const vector<Vector3f>& point_cloud) {
  const int i = rand() % (point_cloud.size());
  return point_cloud[i];
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

bool FitBestPlaneService(
    KBSL::FitBestPlaneSrv::Request& req,
    KBSL::FitBestPlaneSrv::Response& res) {
  // Copy over all the points.
  vector<Vector3f> point_cloud(req.P.size());
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    point_cloud[i] = ConvertPointToVector(req.P[i]);
  }

  Vector3f n;
  Vector3f P0;
  // Compute n and P0 from the point cloud.
  FitBestPlane(point_cloud, &n, &P0);

  res.n = ConvertVectorToPoint(n);
  res.P0 = ConvertVectorToPoint(P0);
  return true;
}

void RANSAC(const vector<Vector3f>& point_cloud, Vector3f* n_ptr,
            Vector3f* P0_ptr) {
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
    const Vector3f P1 = PickRandomPoint(point_cloud);
    const Vector3f P2 = PickRandomPoint(point_cloud);
    const Vector3f P3 = PickRandomPoint(point_cloud);
    FitMinimalPlane(P1, P2, P3, &n, &P0);
    FindInliers(n, P0, kEpsilon, point_cloud, &inliers);
    inlier_fraction = static_cast<float>(inliers.size()) /
        static_cast<float>(point_cloud.size());
  } while (i < kIterations && inlier_fraction < kMinInlierFraction);
  FitBestPlane(inliers, n_ptr, P0_ptr);
}

void PointCloudCallback(const sensor_msgs::PointCloud& point_cloud_msg) {
  filtered_point_cloud_msg.header = point_cloud_msg.header;

  // Create a Vector3f point cloud, of the same size as the input point cloud.
  vector<Vector3f> point_cloud(point_cloud_msg.points.size());

  // Copy over the input point cloud.
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    point_cloud[i] = ConvertPointToVector(point_cloud_msg.points[i]);
  }

  vector<Vector3f> filtered_point_cloud;
  Vector3f n;
  Vector3f P0;
  // Extract filtered point cloud.
  RANSAC(point_cloud, &n, &P0);
  FindInliers(n, P0, kEpsilon, point_cloud, &filtered_point_cloud);

  // Copy over the output point cloud.
  filtered_point_cloud_msg.points.resize(filtered_point_cloud.size());
  for (size_t i = 0; i < filtered_point_cloud.size(); ++i) {
    filtered_point_cloud_msg.points[i] =
        ConvertVectorToPoint(filtered_point_cloud[i]);
  }

  filtered_point_cloud_publisher_.publish(filtered_point_cloud_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "assignment3");
  ros::NodeHandle n;

  ros::ServiceServer transform_point_service =
    n.advertiseService("/COMPSCI403/TransformPoint", TransformPointService);
  ros::ServiceServer fit_min_plane_service =
    n.advertiseService("/COMPSCI403/FitMinimalPlane", FitMinimalPlaneService);
  ros::ServiceServer find_inliers_service =
    n.advertiseService("/COMPSCI403/FindInliers", FindInliersService);
  ros::ServiceServer fit_best_plane_service =
    n.advertiseService("/COMPSCI403/FitBestPlane", FitBestPlaneService);

  filtered_point_cloud_publisher_ =
    n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/FilteredPointCloud", 1);
  plane_parameters_publisher_ =
    n.advertise<KBSL::PlaneParametersMsg>(
      "/COMPSCI403/PlaneParameters", 1);

  ros::Subscriber point_cloud_subscriber =
    n.subscribe("/COMPSCI403/PointCloud", 1, PointCloudCallback);

  ros::spin();

  return 0;
}
