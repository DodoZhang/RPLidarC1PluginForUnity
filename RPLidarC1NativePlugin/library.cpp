#include "library.h"

#include <thread>
#include <chrono>

struct Lidar {
    sl::ILidarDriver *driver;
    sl::IChannel *channel;
    std::thread reader;
    bool isScanning;
    struct LidarBufferData {
        float angle;
        float distance;
    } *buffer;
    size_t head;

    Lidar() : driver(nullptr)
            , channel(nullptr)
            , isScanning(false)
            , buffer(nullptr)
            , head(0) { }
};

sl_lidar_response_measurement_node_hq_t buffer[LIDAR_BUFFER_LENGTH];

Lidar *create() {
    auto *lidar = new Lidar();
    lidar->driver = *sl::createLidarDriver();
    lidar->channel = nullptr;
    lidar->isScanning = false;
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

void read(Lidar *lidar) {
    while (lidar->isScanning) {
        size_t bufferLength = LIDAR_BUFFER_LENGTH;
        if (!SL_IS_OK(lidar->driver->grabScanDataHq(buffer, bufferLength))) {
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
            continue;
        }
        for (int i = 0; i < bufferLength; i++) {
            float angle = ((float)buffer[i].angle_z_q14) * 90 / (1 << 14);
            float distance = ((float)buffer[i].dist_mm_q2) / 1000 / (1 << 2);
            if (distance < 1e-7) continue;
            lidar->head = (lidar->head + 1) & LIDAR_BUFFER_MASK;
            lidar->buffer[lidar->head].angle = angle;
            lidar->buffer[lidar->head].distance = distance;
        }
    }
}

bool start(Lidar *lidar) {
    if (!SL_IS_OK(lidar->driver->startScan(false, true))) return false;
    lidar->isScanning = true;
    lidar->reader = std::thread(read, lidar);
    return true;
}

bool stop(Lidar *lidar) {
    if (!SL_IS_OK(lidar->driver->stop())) return false;
    lidar->isScanning = false;
    lidar->reader.join();
    return true;
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
