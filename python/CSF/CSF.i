%module CSF
%{
  #define SWIG_FILE_WITH_INIT
  #include "../src/CSF.h"
  #include "../src/Cloth.h"
%}

%include "std_string.i"
%include "std_vector.i"
%include "numpy.i"

%init %{
import_array();
%}

namespace std
{
    %template(VecInt) vector<int>;
    %template(VecFloat) vector<float>;
    %template(VecVecFloat) vector< vector<float> >;
    %template(VecDouble) vector<double>;
}

%apply (double* IN_ARRAY2, int DIM1, int DIM2) {(double *points, int rows, int cols)};
%include "../src/CSF.h"
