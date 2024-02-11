//
// Created by jordan on 01/31/2024.
//

// DetectionBuffer.h - Header file for managing detections and their storage

#include <string.h>

#ifndef DETECTION_BUFFER_H
#define DETECTION_BUFFER_H

// Structure to represent a single detection
struct Detection {
    char class_name[20]; // Name of the detected class (e.g., "person", "car")
    float confidence; // Confidence score of the detection
    float timestamp; // Timestamp when the detection occurred
    float depth_mm; // Depth of the detected object in millimeters
    float depth_in; // Depth of the detected object in inches
    float x, y, z; // 3D coordinates of the detected object
    float horizontal_angle; // Horizontal angle of the detected object relative to the sensor
    char direction[10]; // Direction of the detected object (e.g., "left", "right")

    // Constructor to initialize the Detection structure
    Detection(const char* _class_name = "",
              float _confidence = 0.0f,
              float _timestamp = 0.0f,
              float _depth_mm = 0.0f,
              float _depth_in = 0.0f,
              float _x = 0.0f,
              float _y = 0.0f,
              float _z = 0.0f,
              float _horizontal_angle = 0.0f,
              const char* _direction = ""
              )
    {
        strncpy(class_name, _class_name, sizeof(class_name) - 1);
        class_name[sizeof(class_name) - 1] = '\0'; // Ensure null-termination
        confidence = _confidence;
        timestamp = _timestamp;
        depth_mm = _depth_mm;
        depth_in = _depth_in;
        x = _x;
        y = _y;
        z = _z;
        horizontal_angle = _horizontal_angle;
        strncpy(direction, _direction, sizeof(direction) - 1);
        direction[sizeof(direction) - 1] = '\0'; // Ensure null-termination
    }

    // Assignment operator
    Detection& operator=(const Detection& other);
};




const int BUFFER_SIZE = 10; // Size of the buffer to store detections

// Function declarations
void addDetectionToBuffer(const Detection& detection); // Adds a detection to the buffer
Detection getDetectionFromBuffer(int index); // Retrieves a detection from the buffer
int getBufferSize(); // Returns the current size of the buffer
Detection getLatestDetection(); // Retrieves the most recent detection
Detection getClosestDetection(); // Retrieves the detection closest to the sensor
void clearBuffer(); // Clears the detection buffer

// Global variables
extern Detection buffer[BUFFER_SIZE]; // Buffer to store detections
extern int bufferIndex; // Current index in the buffer

#endif
