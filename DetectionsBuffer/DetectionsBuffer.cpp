//
// Created by jordan on 01/31/2024.
//

// DetectionBuffer.cpp - Implementation of functions for managing detections

#include "DetectionsBuffer.h"
#include <string.h>

// Global buffer and index for storing detections
Detection buffer[BUFFER_SIZE];
int bufferIndex = 0;

// Overloaded assignment operator for Detection structure
Detection& Detection::operator=(const Detection& other) {
    if (this != &other) { // Check for self-assignment
        // Copy data from 'other' Detection to this Detection
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
    return *this; // Return reference to this object
}

// Function to add a detection to the global buffer
void addDetectionToBuffer(const Detection& detection) {
    buffer[bufferIndex] = detection; // Store detection
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE; // Update index circularly
}

// Function to retrieve a detection from the buffer
Detection getDetectionFromBuffer(int index) {
    if (index < 0 || index >= BUFFER_SIZE) {
        // Return an empty detection if index is out of bounds
        Detection emptyDetection{};
        return emptyDetection;
    }
    return buffer[(bufferIndex + index) % BUFFER_SIZE]; // Retrieve detection
}

// Function to get the current buffer size
int getBufferSize() {
    return BUFFER_SIZE;
}

// Function to get the closest detection (most recently added)
Detection getClosestDetection() {
    int latestIndex = (bufferIndex - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    return buffer[latestIndex]; // Return the latest detection
}

// Function to get the detection with the earliest timestamp
Detection getLatestDetection() {
    float minTime = 1000; // Initialize with a large timestamp value
    Detection latestDetection;

    // Loop through the buffer to find the detection with the earliest timestamp
    for (auto & i : buffer) {
        if (i.timestamp > 0 && i.timestamp < minTime) {
            minTime = i.timestamp;
            latestDetection = i;
        }
    }
    return latestDetection; // Return the detection with the earliest timestamp
}

// Function to clear the detection buffer
void clearBuffer() {
    bufferIndex = 0; // Reset the buffer index to start
}
