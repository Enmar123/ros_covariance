#include <iostream>
#include <vector>

#include <math.h>  //for pow
#include <numeric>

#include <sstream>>


using namespace std;

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
  //double ans = sum/(n_vals - 1);
  double ans = sum/(n_vals);
  return ans;
}

void test1() {

  for( int i = 0 ; i < 6 ; i++ )
    cout << i << endl;

  cout << "----------------------" << endl;
  int ree[6];
  for( int i = 0 ; i < 6 ; i++ )
    cout << ree[i] << endl;

  cout << "----------------------" << endl;
  int kek[6] = {0};
  for( int i = 0 ; i < 6 ; i++ )
    cout << kek[i] << endl;

  cout << "----------------------" << endl;
  int foo[6] = {1,1,1,1,1,1};
  for( int i = 0 ; i < 6 ; i++ )
    cout << foo[i] << endl;

  cout << "----------------------" << endl;
  vector<size_t> bar = {0};
  for( size_t i = 0 ; i < 6 ; i++ ){
    bar.push_back(i);
    cout << bar[i] << endl;
  }

  cout << "----------------------" << endl;
  vector<vector<int>> rek(5, vector<int>(5, 42));
  cout << rek.size() << endl;
}

int main() {

  double wee = 0.123456789;
  vector<double> vec0 = {1,0};
  vector<double> vec1 = {1,1,1,1,1,0,0,0,0,0};
  vector<double> vec2 = {1,1,1,1,1,0,0,0,0,0,
                        1,1,1,1,1,0,0,0,0,0,
                        1,1,1,1,1,0,0,0,0,0,
                        1,1,1,1,1,0,0,0,0,0,
                        1,1,1,1,1,0,0,0,0,0,
                        1,1,1,1,1,0,0,0,0,0,
                        1,1,1,1,1,0,0,0,0,0,
                        1,1,1,1,1,0,0,0,0,0,
                        1,1,1,1,1,0,0,0,0,0,
                        1,1,1,1,1,0,0,0,0,0};

  cout << variance( vec0 ) << endl;
  cout << variance( vec1 ) << endl;
  cout << variance( vec2 ) << endl;
  cout << wee << endl;

  cout << "----------------------" << endl;
  string s = "0.123";
  stringstream geek(s);
  double x = 0;
  geek >> x;
  cout << x << endl;

  return 0;
}
