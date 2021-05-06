# IPFT

C++ implementation of Information Particle Filter Tree algorithm

### [Doxygen documentation](/doxygen/index.html)
### [Coverage report](/coverage/index.html)

## Notes on Memorypool

This implementation of the IPFT solver uses the concept of pooling for dynamic memory allocation of `State` and `Observation` objects. Therefore, always allocate `State` and `Observation` objects through the specified functions in the `POMDP` model class. Make sure that you do not forget to destroy the objects (again only through the functions in `POMDP`), when you are done using them. 

TODO(max) give example here!