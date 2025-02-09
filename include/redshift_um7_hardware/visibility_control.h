#ifndef REDSHIFT_UM7_HARDWARE__VISIBILITY_CONTROL_H_
#define REDSHIFT_UM7_HARDWARE__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define REDSHIFT_UM7_HARDWARE_EXPORT __attribute__((dllexport))
#define REDSHIFT_UM7_HARDWARE __attribute__((dllimport))
#else
#define REDSHIFT_UM7_HARDWARE_EXPORT __declspec(dllexport)
#define REDSHIFT_UM7_HARDWARE __declspec(dllimport)
#endif
#ifdef REDSHIFT_UM7_HARDWARE_BUILDING_DLL
#define REDSHIFT_UM7_HARDWARE_PUBLIC REDSHIFT_UM7_HARDWARE_EXPORT
#else
#define REDSHIFT_UM7_HARDWARE_PUBLIC REDSHIFT_UM7_HARDWARE
#endif
#define REDSHIFT_UM7_HARDWARE_PUBLIC_TYPE REDSHIFT_UM7_HARDWARE_PUBLIC
#define REDSHIFT_UM7_HARDWARE_LOCAL
#else
#define REDSHIFT_UM7_HARDWARE_EXPORT __attribute__((visibility("default")))
#define REDSHIFT_UM7_HARDWARE
#if __GNUC__ >= 4
#define REDSHIFT_UM7_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define REDSHIFT_UM7_HARDWARE_LOCAL __attribute__((visibility("hidden")))
#else
#define REDSHIFT_UM7_HARDWARE_PUBLIC
#define REDSHIFT_UM7_HARDWARE_LOCAL
#endif
#define REDSHIFT_UM7_HARDWARE_PUBLIC_TYPE
#endif

#endif  // REDSHIFT_UM7_HARDWARE__VISIBILITY_CONTROL_H_