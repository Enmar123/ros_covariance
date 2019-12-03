#include <iostream>

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


  return 0;
}
