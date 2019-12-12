#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <iostream>
#include <math.h>
//#include <functional>
//#include <numeric>
//#include <cmath>


using namespace std;


// Global Variables
static bool is_filter_on = false;
static bool is_min_variance_provided = true;

static int debug = 0;
static double diff_reject_threshold = 0;
static double min_variance = 0.001;

static size_t n_data_max = 100;
static double vars[12] = {0};
static double pos_cov[36] = {0};
static double vel_cov[36] = {0};
static ros::Publisher pub;

static vector<double> min_cov(36, 0.0);
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
  double ans = sum/(n_vals - 1);  // Sample variance
  //double ans = sum/(n_vals);        // Total variance
  return ans;
}

void variances( const vector<vector<double>> &data ) {
  // Create an array of variances
  for( size_t i = 0; i < data.size() ; i++ )
    vars[i] = variance( data[i] );
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

  double diff;
  for( size_t i = 0 ; i < data.size() ; i++) {
    diff = (data[i].end()[-1] - data[i].end()[-2]);
    if( is_filter_on ){
      if( abs(diff) <= diff_reject_threshold )
        data[i].pop_back();
    }
    else if( data[i].size() > n_data_max)
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

//  // Check for min covariance
//  for( size_t i = 0 ; i < data.size() ; i++)
//      if (vars[i] < min_variances[i])
//        vars[i] = min_variances[i];     // Set min covariance value

  // Iterate to create to covariance message
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

  for( size_t i = 0 ; i < 36 ; i++ ) {
    if( is_min_variance_provided ) {
      if( pos_cov[i] > msg.pose.covariance[i])
        msg.pose.covariance[i] = pos_cov[i];
      if( vel_cov[i] > msg.twist.covariance[i] )
        msg.twist.covariance[i] = vel_cov[i];
      }
    else {
      if( pos_cov[i] > min_variance )
        msg.pose.covariance[i] = pos_cov[i];
      else
        msg.pose.covariance[i] = min_cov[i];
      if( vel_cov[i] > min_variance )
        msg.twist.covariance[i] = vel_cov[i];
      else
        msg.twist.covariance[i] = min_cov[i];
    }
  }

  pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "variance");
  ros::NodeHandle nh("~");

  string topic_in, topic_out;

  if (nh.getParam("topic_in", topic_in))
    ROS_INFO("Got param: %s", topic_in.c_str());
  else
    ROS_ERROR("Failed to get param 'topic_in'");

  if (nh.getParam("topic_out", topic_out))
    ROS_INFO("Got param: %s", topic_out.c_str());
  else
    ROS_ERROR("Failed to get param 'topic_out'");

//  if (nh.getParam("min_varainces", min_varainces))
//    ROS_INFO("Got param: %s", min_varainces.c_str());
//  else
//    ROS_ERROR("Failed to get param 'min_varainces'");


//  topic_in  = "/jackal_velocity_controller/odom";
//  topic_out = "/odom";

  for( size_t i=0; i < min_cov.size() ; i+=7)
    min_cov[i] = min_variance;

  ROS_INFO("Starting Node: covariance_calculator");
  pub = nh.advertise<nav_msgs::Odometry>(topic_out.c_str(), 10);
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(topic_in.c_str(), 10, callback);

  while(ros::ok()) {
    ros::spin();
  }

  return 0;
}
