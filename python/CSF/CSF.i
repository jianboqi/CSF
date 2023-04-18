%module CSF
%{
  #define SWIG_FILE_WITH_INIT
  #include "../src/CSF.h"
  #include "../src/Cloth.h"
%}

%include "std_string.i"
%include "std_vector.i"

namespace std
{
    %template(VecInt) vector<int>;
    %template(VecFloat) vector<float>;
    %template(VecVecFloat) vector< vector<float> >;
    %template(VecDouble) vector<double>;
}

%include "../src/CSF.h"
