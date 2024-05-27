#include "library.h"

struct Lidar {
    sl::ILidarDriver *driver;
    sl::IChannel *channel;
    struct LidarBufferData {
        float angle;
        float distance;
    } *buffer;
    size_t head;
};

sl_lidar_response_measurement_node_hq_t buffer[LIDAR_BUFFER_LENGTH];

Lidar *create() {
    auto *lidar = new Lidar();
    lidar->driver = *sl::createLidarDriver();
    lidar->channel = nullptr;
    lidar->buffer = new Lidar::LidarBufferData[LIDAR_BUFFER_LENGTH];
    lidar->head = LIDAR_BUFFER_LENGTH - 1;
    return lidar;
}

void destroy(Lidar *lidar) {
    delete lidar->buffer;
    delete lidar->driver;
    delete lidar;
}

bool connect(Lidar *lidar, const char *port) {
    lidar->channel = *sl::createSerialPortChannel(port, 460800);
    if (SL_IS_OK(lidar->driver->connect(lidar->channel))) return true;
    delete lidar->channel;
    lidar->channel = nullptr;
    return false;
}

void disconnect(Lidar *lidar) {
    lidar->driver->disconnect();
    delete lidar->channel;
    lidar->channel = nullptr;
}

bool start(Lidar *lidar) {
    return SL_IS_OK(lidar->driver->startScan(false, true));
}

bool stop(Lidar *lidar) {
    return SL_IS_OK(lidar->driver->stop());
}

int read(Lidar *lidar) {
    size_t bufferLength = LIDAR_BUFFER_LENGTH;
    if (!SL_IS_OK(lidar->driver->grabScanDataHq(buffer, bufferLength)))
        return 0;
    for (int i = 0; i < bufferLength; i++) {
        float angle = ((float)buffer[i].angle_z_q14) * 90 / (1 << 14);
        float distance = ((float)buffer[i].dist_mm_q2) / 1000 / (1 << 2);
        if (distance < 1e-7) continue;
        lidar->head = (lidar->head + 1) & LIDAR_BUFFER_MASK;
        lidar->buffer[lidar->head].angle = angle;
        lidar->buffer[lidar->head].distance = distance;
    }
    return (int)bufferLength;
}

int head(Lidar *lidar) {
    return (int)lidar->head;
}

float getAngle(Lidar *lidar, int index) {
    return lidar->buffer[index].angle;
}

float getDistance(Lidar *lidar, int index) {
    return lidar->buffer[index].distance;
}
