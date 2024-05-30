#ifndef RPLIDARC1NATIVEPLUGIN_LIBRARY_H
#define RPLIDARC1NATIVEPLUGIN_LIBRARY_H

#include "sl_lidar.h"

#define LIDAR_BUFFER_LENGTH 1024
#define LIDAR_BUFFER_MASK 1023

struct Lidar;

#define RPLIDARC1_API __declspec(dllexport)

extern "C" {

RPLIDARC1_API Lidar *create();
RPLIDARC1_API void destroy(Lidar *lidar);
RPLIDARC1_API bool connect(Lidar *lidar, const char *port);
RPLIDARC1_API void disconnect(Lidar *lidar);
RPLIDARC1_API bool start(Lidar *lidar);
RPLIDARC1_API bool stop(Lidar *lidar);
RPLIDARC1_API int head(Lidar *lidar);
RPLIDARC1_API float getAngle(Lidar *lidar, int index);
RPLIDARC1_API float getDistance(Lidar *lidar, int index);

}

#endif //RPLIDARC1NATIVEPLUGIN_LIBRARY_H
