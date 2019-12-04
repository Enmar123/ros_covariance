#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <iostream>
//#include <math.h>
//#include <functional>
//#include <numeric>
//#include <cmath>


using namespace std;


//template<typename T>
//class Matrix {
//public:
//  Matrix(size_t n = 0, size_t m = 0) : m_data(n,m) { }
//  T& operator()(size_t i, size_t j) {
//    return m_data[i*m_m + j]; }
//  size_t size(size_t n) const {
//    return n==0?m_n:m_m; }
//  std::vector<T> &data() {
//    return  m_data; }
//private:
//  size_t m_n;
//  size_t m_m;
//  std::vector<T> m_data;
//};

//Matrix<double> cov(6,6);

// Global Variables
static size_t n_data_max = 100;
static double vars[6] = {1,1,1,1,1,1};
static double cov[36] = {0};
static ros::Publisher pub;
static vector<vector<double>> data(6, vector<double>(1, 0));


double avg( const vector<double> vec ) {
  double sum = std::accumulate(vec.begin(), vec.end(), 0);
  double mean = sum/vec.size();
  return mean;
}

double variance( const vector<double> vec ) {
  double mean = avg(vec);
  unsigned long int n_vals = vec.size();
  vector<double> vals(n_vals);
  for( unsigned long int i = 0; i == n_vals; i++)
    vals[i] = pow((vec[i] - mean), 2);
  double sum = std::accumulate(vals.begin(), vals.end(), 0);
  double ans = sum/(n_vals - 1);
  return ans;
}

void variances( const vector<vector<double>> data ) {
  for( size_t i = 0; i < 6; i++)
    vars[i] = variance( data[i] );
}

void updateData( const nav_msgs::Odometry msg ) {
  tf2::Quaternion quat;
  tf2::convert(msg.pose.pose.orientation, quat);

  data[0].push_back(msg.pose.pose.position.x);
  data[1].push_back(msg.pose.pose.position.y);
  data[2].push_back(msg.pose.pose.position.z);
  data[3].push_back(quat.getX());
  data[4].push_back(quat.getY());
  data[5].push_back(quat.getZ());

  if( data.size() > n_data_max )
    for( size_t i = 0 ; i < 6 ; i++ )
      data[i].pop_back();
}

void callback( nav_msgs::Odometry msg ) {
  updateData(msg);
  variances(data);

  for( size_t i = 0 ; i < 6 ; i++ )
    cov[i*7] = vars[i];

  for( size_t i = 0 ; i < 36 ; i++ )
    msg.pose.covariance[i] = cov[i];

  pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "variance");
  ros::NodeHandle nh;

  string topic_in  = "jackal_velocity_controller/odom";
  string topic_out = "modified/odom";

  ROS_INFO("Starting Node: covariance_calculator");
  pub = nh.advertise<nav_msgs::Odometry>(topic_out, 10);
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(topic_in, 10, callback);

  while(ros::ok()) {
    ros::spin();
  }

  return 0;
}
