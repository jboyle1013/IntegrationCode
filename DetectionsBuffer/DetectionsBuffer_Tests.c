//
// Created by jorda on 01/31/2024.
//
#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() function
#include "catch.hpp"
#include "DetectionsBuffer.h"
#include<stdio.h>
#include <string.h>


TEST_CASE("Add and Retrieve Detection from Buffer", "[DetectionBuffer]") {
    // Clear buffer before starting
    clearBuffer();

    // Create a detection
    Detection detection;
    strcpy(detection.class_name, "TestObject");
    detection.confidence = 0.8f;
    detection.depth_mm = 100.0f;
    detection.depth_in = 3.93701f;
    detection.x = detection.y = detection.z = 10.0f;
    detection.horizontal_angle = 45.0f;
    strcpy(detection.direction, "left");
    detection.timestamp = 1234567890;

    SECTION("Adding a Detection") {
        addDetectionToBuffer(detection);

        REQUIRE(bufferIndex == 1);
        Detection retrieved = getLatestDetection();
        REQUIRE(strcmp(retrieved.class_name, "TestObject") == 0);
        REQUIRE(retrieved.confidence == Approx(0.8f));
        // ... Continue with other fields ...
    }

    SECTION("Retrieving the Closest Detection") {
        addDetectionToBuffer(detection);
        Detection closest = getClosestDetection();
        REQUIRE(closest.depth_mm == detection.depth_mm);
    }

    SECTION("Buffer Overflow") {
        for (int i = 0; i < BUFFER_SIZE + 5; ++i) {
            addDetectionToBuffer(detection);
        }
        REQUIRE(bufferIndex == 5);
    }

    SECTION("Clearing the Buffer") {
        addDetectionToBuffer(detection);
        clearBuffer();
        REQUIRE(bufferIndex == 0);
        Detection emptyDetection = getLatestDetection();
        REQUIRE(emptyDetection.timestamp == 0); // Assuming default timestamp is 0
    }
}

// ... Additional test cases ...
