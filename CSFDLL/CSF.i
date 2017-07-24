%module CSF
%{
  #define SWIG_FILE_WITH_INIT
  #include "../CSFDLL/CSF.h"
%}

%include "std_vector.i"

namespace std
{
    %template(VecInt) vector<int>;
    %template(Vecf) vector<float>;
    %template(VecVecf) vector< vector<float> >;
}

%include "../CSFDLL/CSF.h"
