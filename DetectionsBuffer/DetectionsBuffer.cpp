//
// Created by jordan on 01/31/2024.
//

// DetectionBuffer.cpp

#include "DetectionsBuffer.h"
#include <string.h>

// Global buffer and index definitions
Detection buffer[BUFFER_SIZE];
int bufferIndex = 0;

Detection& Detection::operator=(const Detection& other) {
    if (this != &other) { // Protect against self-assignment
        // Copy each field from 'other' to 'this'
        strcpy(this->class_name, other.class_name);
        this->confidence = other.confidence;
		this->timestamp = other.timestamp;
        this->depth_mm = other.depth_mm;
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
        this->horizontal_angle = other.horizontal_angle;
        strcpy(this->direction, other.direction);
    }
    return *this;
}


void addDetectionToBuffer(const Detection& detection) {
    buffer[bufferIndex] = detection;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
}

Detection getDetectionFromBuffer(int index) {
    if (index < 0 || index >= BUFFER_SIZE) {
        // Return an empty detection or handle error
        Detection emptyDetection{};
        return emptyDetection;
    }
    return buffer[(bufferIndex + index) % BUFFER_SIZE];
}

int getBufferSize() {
    return BUFFER_SIZE;
}


Detection getClosestDetection() {
    int latestIndex = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    return buffer[latestIndex];
}

Detection getLatestDetection() {
    float minTime = 1000; // Set initial timestamp to maximum possible float value
    Detection latestDetection;

    for (auto & i : buffer) {
        // Check if the detection's timestamp is less than the current minimum
        if (i.timestamp > 0 && i.timestamp < minTime) {
            minTime = i.timestamp;
            latestDetection = i;
        }
    }
    return latestDetection;
}

void clearBuffer() {
    bufferIndex = 0;
}
