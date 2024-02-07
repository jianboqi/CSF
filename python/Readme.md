# Dependencies

- SWIG (4.0)
- numpy.i (delivered in the documentation part of the numpy package)

# Generate bindings
```shell
swig4.0 -python -c++ -o CSF/CSF_wrap.cxx -module CSF_3DFin CSF/CSF.i
```