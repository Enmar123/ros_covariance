#include <iostream>
#include <vector>

using namespace std;

int main() {

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
  return 0;
}
