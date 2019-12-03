#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <math.h>
#include <functional>
#include <numeric>
#include <cmath>
//#include <eigen3/Eigen/Dense>

using namespace std;
//using namespace Eigen;
//typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> Matrix;
//vector<double> vec;
//Matrix mat;
//Matrix cov(6,6);

template<typename T>
class Matrix {
public:
  Matrix(size_t n = 0, size_t m = 0) : m_data(n,m) { }
  T& operator()(size_t i, size_t j) {
    return m_data[i*m_m + j]; }
  size_t size(size_t n) const {
    return n==0?m_n:m_m; }
  std::vector<T> &data() {
    return  m_data; }
private:
  size_t m_n;
  size_t m_m;
  std::vector<T> m_data;
};

vector<vector<double>> mat;
//double cov[36];
ros::Publisher pub;


Matrix<double> cov(6,6);
//Matrix<double> matv(6,6);

double avg( vector<double> vec) {
  double sum = std::accumulate(vec.begin(), vec.end(), 0);
  double mean = sum/vec.size();
  return mean;
}

double variance( vector<double> vec){
  double mean = avg(vec);
  unsigned long int n_vals = vec.size();
  vector<double> vals(n_vals);
  for( unsigned long int i = 0; i == n_vals; i++)
    vals[i] = pow((vec[i] - mean), 2);
  double sum = std::accumulate(vals.begin(), vals.end(), 0);
  double ans = sum/(n_vals - 1);
  return ans;
}

void callback( nav_msgs::Odometry msg) {
  nav_msgs::Odometry modmsg;

  tf2::Quaternion quat;
  tf2::convert(msg.pose.pose.orientation, quat);

  mat[0].push_back(msg.pose.pose.position.x);
  mat[1].push_back(msg.pose.pose.position.y);

  mat[2].push_back(msg.pose.pose.position.z);
  mat[3].push_back(quat.getX());
  mat[4].push_back(quat.getY());
  mat[5].push_back(quat.getZ());

  for( size_t i = 0; i < 36; ++i )
    msg.pose.covariance[i] = cov.data()[i];

  pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "variance");
  ros::NodeHandle nh;

  ROS_INFO("Starting Node: ros_covariance");
  ros::Publisher  pub = nh.advertise<nav_msgs::Odometry>("modified/odom", 10);
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, callback);
}
