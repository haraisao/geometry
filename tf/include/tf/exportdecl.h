/*
 * Cross platform macros.
 *
 */
#ifndef TF__DECL_H_
#define TF__DECL_H_

#include <ros/macros.h>

#ifdef ROS_BUILD_SHARED_LIBS  // ros is being built around shared libraries
  #ifdef tf_EXPORTS  // we are building a shared lib/dll
    #define TF_DECL ROS_HELPER_EXPORT
  #else  // we are using shared lib/dll
    #define TF_DECL ROS_HELPER_IMPORT
  #endif
#else  // ros is being built around static libraries
  #define TF_DECL
#endif

#endif