//
// Created by jordan on 01/31/2024.
//

// DetectionBuffer.h

#ifndef DETECTION_BUFFER_H
#define DETECTION_BUFFER_H

struct Detection {
    char class_name[20];
    float confidence;
    float depth_mm;
    float depth_in;
    float x, y, z;
    float horizontal_angle;
    char direction[10];
    long long timestamp;

    Detection& operator=(const Detection& other);

};

const int BUFFER_SIZE = 10; // Adjust the buffer size as needed

void addDetectionToBuffer(const Detection& detection);
Detection getDetectionFromBuffer(int index);
int getBufferSize();
Detection getLatestDetection();
Detection getClosestDetection();
void clearBuffer();

extern Detection buffer[BUFFER_SIZE];
extern int bufferIndex;

#endif
