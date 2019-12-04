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
static int debug = 1;
static size_t n_data_max = 100;
static double vars[6] = {0};
static double cov[36] = {0};
static ros::Publisher pub;
static vector<vector<double>> data(6, vector<double>(1, 0));


double avg( const vector<double> vec ) {
  // Calculate the average of an array
  double sum = std::accumulate(vec.begin(), vec.end(), 0.0); // fricking 0.0 ...
  double mean = sum/vec.size();
  cout << vec[0] << endl;
  cout << sum << endl;
  cout << mean << endl;
  return mean;
}

double variance( const vector<double> vec ) {
  // Calculate the variance of an array
  double mean = avg(vec);
  size_t n_vals = vec.size();
  vector<double> vals(n_vals);
  for( size_t i = 0; i < n_vals; i++){                     // ah... i had put ==
    vals[i] = pow((vec[i] - mean), 2);
    cout << vals[i] << " ";
  }
  cout << endl;
  double sum = std::accumulate(vals.begin(), vals.end(), 0.0); // fricking 0.0 ...
  double ans = sum/(n_vals - 1);
  cout << ans << endl;
  cout << endl;
  return ans;
}

void variances( const vector<vector<double>> data ) {
  // Create a 1x6 array of variances
  for( size_t i = 0; i < 6; i++)
    vars[i] = variance( data[i] );
}

void updateData( const nav_msgs::Odometry msg ) {
  // Update data for variance calculation
  tf2::Quaternion quat;
  tf2::convert(msg.pose.pose.orientation, quat);

  data[0].push_back(msg.pose.pose.position.x);
  data[1].push_back(msg.pose.pose.position.y);
  data[2].push_back(msg.pose.pose.position.z);
  data[3].push_back(quat.getX());
  data[4].push_back(quat.getY());
  data[5].push_back(quat.getZ());

  if( data[0].size() > n_data_max ){
    for( size_t i = 0 ; i < 6 ; i++ )
      data[i].erase(data[i].begin());
  }
  // Debug code
  if (debug == 1){
    cout << "data    ";
    for( size_t i = 0 ; i < data[0].size() ; i++ )
      cout << data[0][i] << " ";
    cout << endl;
  }
}

void callback( nav_msgs::Odometry msg ) {
  updateData(msg);
  variances(data);

  if (debug == 1){
    cout << "vars    ";
    for( size_t i = 0 ; i < 6; i++)
      cout << vars[i] << " ";
    cout << endl;
  }

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
