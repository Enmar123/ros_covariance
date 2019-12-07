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
static int debug = 0;
static size_t n_data_max = 100;
static double vars[12] = {0};
static double pos_cov[36] = {0};
static double vel_cov[36] = {0};
static ros::Publisher pub;
static vector<vector<double>> data(12, vector<double>(1, 0));


double avg( const vector<double> &vec ) {
  // Calculate the average of an array
  double sum = std::accumulate(vec.begin(), vec.end(), 0.0); // fricking 0.0 ...
  double mean = sum/vec.size();
  return mean;
}

double variance( const vector<double> &vec ) {
  // Calculate the variance of an array
  double mean = avg(vec);
  size_t n_vals = vec.size();
  vector<double> vals(n_vals);
  for( size_t i = 0; i < n_vals; i++){                     // ah... i had put ==
    vals[i] = pow((vec[i] - mean), 2);
  }
  double sum = std::accumulate(vals.begin(), vals.end(), 0.0); // fricking 0.0 ...
  double ans = sum/(n_vals - 1);
  return ans;
}

void variances( const vector<vector<double>> &data ) {
  // Create an array of variances
  for( size_t i = 0; i < data.size() ; i++ )
    vars[i] = variance( data[i] );
}

bool isDataSame() {
  double diff;
  for ( size_t i = 0 ; i < data.size() ; i++ ) {
    diff = (data[i].end()[-1] - data[i].end()[-2]);
    if ( abs(diff) <= 0 ) {
      return false; //dunno why no work atm
    }
  }
  return false;
}


void updateData( const nav_msgs::Odometry &msg ) {
  // Update data for variance calculation
  tf2::Quaternion quat;
  tf2::convert(msg.pose.pose.orientation, quat);

  data[0].push_back(msg.pose.pose.position.x);
  data[1].push_back(msg.pose.pose.position.y);
  data[2].push_back(msg.pose.pose.position.z);
  data[3].push_back(quat.getX());
  data[4].push_back(quat.getY());
  data[5].push_back(quat.getZ());

  data[6].push_back(msg.twist.twist.linear.x);
  data[7].push_back(msg.twist.twist.linear.y);
  data[8].push_back(msg.twist.twist.linear.z);
  data[9].push_back(msg.twist.twist.angular.x);
  data[10].push_back(msg.twist.twist.angular.y);
  data[11].push_back(msg.twist.twist.angular.z);

  if ( data[0].size() > n_data_max or  isDataSame() ) {
    for ( size_t i = 0 ; i < data.size() ; i++ )
      data[i].erase(data[i].begin());
  }

  // Debug code
  if (debug == 1) {
    cout << "--------" << endl;
    cout << "data    ";
    for( size_t i = 0 ; i < data[0].size() ; i++ )
      cout << data[0][i] << " ";
    cout << endl;
  }
}


void callback( nav_msgs::Odometry msg ) {
  // Obtain Odom msg and modify covariance matrix
  updateData(msg);
  variances(data);

  // Debug Code
  if (debug == 1){
    cout << "vars    ";
    for( size_t i = 0 ; i < data.size(); i++)
      cout << vars[i] << " ";
    cout << endl;
  }

//  for ( size_t i = 0, c = 0 ; i < 6 ; ++i, c+=7 ){
//    pos_cov[c] = vars[i];
//    vel_cov[c] = vars[i+6];
//  }

  // Check for min covariance
  for( size_t i = 0 ; i < 12 ; i++)
    if (vars[i] < 0.001)
      vars[i] = 0.001;     // Set min covariance value

  int n = 6;
  auto *u = vars;
  auto *w = vars + 6;
  auto *p = pos_cov;
  auto *v = vel_cov;
  while( n-- ) {
    *p = *u++;
    *v = *w++;
    p += 7;
    v += 7;
  }


  for ( size_t i = 0 ; i < 36 ; i++ ){
    msg.pose.covariance[i] = pos_cov[i];
    msg.twist.covariance[i] = vel_cov[i];
  }

  pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "variance");
  ros::NodeHandle nh("~");

  string topic_in, topic_out;

//  nh.getParam("/topic_in", topic_in);
//  nh.getParam("/topic_out", topic_out);

//  ros::param::get("topic_in", topic_in);
//  ros::param::get("topic_out", topic_out);

  if (nh.getParam("topic_in", topic_in)) {
    ROS_INFO("Got param: %s", topic_in.c_str());
  }
  else {
    ROS_ERROR("Failed to get param 'topic_in'");
  }

  if (nh.getParam("topic_out", topic_out)) {
    ROS_INFO("Got param: %s", topic_out.c_str());
  }
  else {
    ROS_ERROR("Failed to get param 'topic_out'");
  }


//  topic_in  = "jackal_velocity_controller/odom";
//  topic_out = "modified/odom";

  ROS_INFO("Starting Node: covariance_calculator");
  pub = nh.advertise<nav_msgs::Odometry>(topic_out.c_str(), 10);
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(topic_in.c_str(), 10, callback);

  while(ros::ok()) {
    ros::spin();
  }

  return 0;
}
