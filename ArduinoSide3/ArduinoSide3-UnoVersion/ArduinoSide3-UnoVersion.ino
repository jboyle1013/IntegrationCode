// Include necessary libraries and define constants
#include <pt.h>
#include <DetectionsBuffer.h>
#include <SoftwareSerial.h>
#define BUFFER_SIZE 512 // Size of the data buffer
#define MAX_DETECTION_LENGTH 15 // Max length for detection string

// Initialize software serial communication on pins 10 (RX) and 11 (TX)
SoftwareSerial mySerial(10, 11);

// Global variables
char dataBuffer[BUFFER_SIZE]; // Buffer to store incoming data
volatile bool newDataAvailable = false; // Flag to indicate new data arrival
unsigned long previousMillis = 0;  // Timestamp of the last request
const long interval = 10000; // Interval between requests in milliseconds
volatile bool printDetectionsFlag = false; // Flag to control detection printing

// Protothread structures for multitasking
struct pt ptRead, ptProcess, ptPrint;

void setup() {
    Serial.begin(9600); // Start serial communication with the computer
    mySerial.begin(9600); // Start serial communication with the sensor module
    clearBuffer(); // Clear the data buffer
    // Wait for everything to stabilize
    // delay(60000); // Use this delay if starting at the same time as the Jetson
    delay(2000); // Use this delay if starting after Jetson is set up and has been running

    // Initialize protothreads
    PT_INIT(&ptRead);
    PT_INIT(&ptProcess);
    PT_INIT(&ptPrint);
}

void loop() {
    // Run protothreads
    protothreadRead(&ptRead);
    protothreadProcess(&ptProcess);
    protothreadPrint(&ptPrint);
}

// Protothread for reading data from the sensor module
static int protothreadRead(struct pt* pt) {
    PT_BEGIN(pt);

    while (1) {
        delay(50);
        mySerial.println("REQUEST"); // Send a request to the sensor module

        // Wait for a response with a timeout of 5 seconds
        unsigned long startTime = millis();
        while (!mySerial.available() && millis() - startTime < 5000) {
            // Waiting for response
        }

        // Read the response if available and set flag
        if (mySerial.available()) {
            String data = mySerial.readStringUntil('\n');
            data.toCharArray(dataBuffer, BUFFER_SIZE);
            newDataAvailable = true;
        }

        PT_YIELD(pt); // Yield control to other protothreads
    }

    PT_END(pt);
}

// Protothread for processing received data
static int protothreadProcess(struct pt* pt) {
    PT_BEGIN(pt);

    while (1) {
        if (newDataAvailable) {
            Serial.println("Received Detections");
            processDetections(dataBuffer); // Process the data
            newDataAvailable = false; // Reset the data availability flag
            printDetectionsFlag = true; // Set flag to print detections
        }

        PT_YIELD(pt);
    }

    PT_END(pt);
}

// Protothread for printing detection information
static int protothreadPrint(struct pt* pt) {
    PT_BEGIN(pt);

    while (1) {
        if (printDetectionsFlag) {
            printDetections(); // Print detection data
            printDetectionsFlag = false; // Reset the print flag
        }

        PT_YIELD(pt);
    }

    PT_END(pt);
}


void processDetections(char data[]) {
    // Process the received data into individual detections
    char* detection = strtok(data, ";");

    while (detection != NULL) {
        parseDetection(detection); // Parse each detection
        detection = strtok(NULL, ";"); // Move to next detection
    }
}


void parseDetection(char* detection) {
	// Parse a single detection string into its components
    char class_name[MAX_DETECTION_LENGTH];
    float confidence;
    float depth_mm;

    float x, y, z;
    float horizontal_angle, timestamp;
    char direction[6];
    char* token;
    char* rest = detection;
	
	// Extract and convert detection components from the string
	
	// Get ClassName
    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        strncpy(class_name, token, MAX_DETECTION_LENGTH);
    }

	// Get Confidence
    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        confidence = atof(token);
    }

	// Get Timestamp
    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        timestamp = atof(token);
    }

	// Get Depth
    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        depth_mm = atof(token);
    }
	
	// Get X Component of Depth
    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        x = atof(token);
    }

	// Get Y Component of Depth
    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        y = atof(token);
    }

	// Get Z Component of Depth
    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        z = atof(token);
    }
	
	// Get Horizontal Angle
    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        horizontal_angle = atof(token);
    }
	
	// Get Direction
    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        strncpy(direction, token, MAX_DETECTION_LENGTH);
    }


    Detection newDetection(class_name, confidence,timestamp, depth_mm, x, y, z, horizontal_angle, direction);
    // Serial.println("Newest Detection");
    // printDetection(newDetection);
    addDetectionToBuffer(newDetection);

}

void printDetections() {
    // Print the closest detection
    Detection closest = getClosestDetection();
    Serial.println("Closest Detection:");
    printDetection(closest);

    // Print the latest detection
    Detection latest = getLatestDetection();
    Serial.println("Latest Detection:");
    printDetection(latest);

    // Loop through and print all detections
    Serial.println("All Detections:");
    for (int i = 0; i < getBufferSize(); i++) {
        Detection d = getDetectionFromBuffer(i);
        printDetection(d);
    }
}


void printDetection(const Detection& d) {
    if (strlen(d.class_name) > 0) { // Check if the detection is valid
        Serial.print("Class Name: ");
        Serial.println(d.class_name);
        Serial.print("Confidence: ");
        Serial.println(d.confidence, 2);
        Serial.print("Timestamp: ");
        Serial.println(d.timestamp, 2);
        Serial.print("Depth MM: ");
        Serial.println(d.depth_mm);
        Serial.print("X Component: ");
        Serial.println(d.x);
        Serial.print("Y Component: ");
        Serial.println(d.y);
        Serial.print("Z Component: ");
        Serial.println(d.z);
        Serial.print("Horizontal Angle: ");
        Serial.println(d.horizontal_angle);
        Serial.print("Direction: ");
        Serial.println(d.direction);
        Serial.println("-------------------");
    } else {
        Serial.println("No Detection Data");
    }
}
