# Compatibility shim for CMeel bundles where urdfdom-headers is not shipped
# alongside Pinocchio. The robot project does not compile Pinocchio's URDF
# parser; it reads the frozen URDF with tinyxml2 and constructs pinocchio::Model
# directly. This target only satisfies Pinocchio/urdfdom package discovery.
if(NOT TARGET urdfdom_headers::urdfdom_headers)
  add_library(urdfdom_headers::urdfdom_headers INTERFACE IMPORTED)
endif()
set(urdfdom_headers_VERSION "3.0.0")
set(urdfdom_headers_INCLUDE_DIRS "")
set(urdfdom_headers_LIBRARIES urdfdom_headers::urdfdom_headers)
set(urdfdom_headers_FOUND TRUE)
