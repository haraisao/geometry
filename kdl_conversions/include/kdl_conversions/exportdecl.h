/*
 * Cross platform macros.
 *
 */
#ifndef KDL_CONVERSIONS__DECL_H_
#define KDL_CONVERSIONS__DECL_H_

#include <ros/macros.h>

#ifdef ROS_BUILD_SHARED_LIBS  // ros is being built around shared libraries
  #ifdef kdl_conversions_EXPORTS  // we are building a shared lib/dll
    #define KDL_CONVERSIONS_DECL ROS_HELPER_EXPORT
  #else  // we are using shared lib/dll
    #define KDL_CONVERSIONS_DECL ROS_HELPER_IMPORT
  #endif
#else  // ros is being built around static libraries
  #define KDL_CONVERSIONS_DECL
#endif

#endif