//
// Created by jordan on 01/31/2024.
//

// DetectionBuffer.h
#include <string.h>

#ifndef DETECTION_BUFFER_H
#define DETECTION_BUFFER_H

struct Detection {
    char class_name[20]; // Adjust size as needed
    float confidence;;
	float timestamp;
    float depth_mm;
    float depth_in;
    float x, y, z;
    float horizontal_angle;
    char direction[10]; // Adjust size as needed
    

    // Constructor
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
