/*
 * Cross platform macros.
 *
 */
#ifndef TF_CONVERSIONS__DECL_H_
#define TF_CONVERSIONS__DECL_H_

#include <ros/macros.h>

#ifdef ROS_BUILD_SHARED_LIBS  // ros is being built around shared libraries
  #ifdef tf_conversions_EXPORTS  // we are building a shared lib/dll
    #define TF_CONVERSIONS_DECL ROS_HELPER_EXPORT
  #else  // we are using shared lib/dll
    #define TF_CONVERSIONS_DECL ROS_HELPER_IMPORT
  #endif
#else  // ros is being built around static libraries
  #define TF_CONVERSIONS_DECL
#endif

#endif