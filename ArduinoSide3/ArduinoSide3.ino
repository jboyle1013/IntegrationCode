#include <pt.h>
#include <DetectionsBuffer.h>
#include <SoftwareSerial.h>

#define BUFFER_SIZE 512  // Define the size of the data buffer
#define MAX_DETECTION_LENGTH 40  // Define the maximum length of a single detection string

// pins 10 (RX) and 11 (TX) (Note Orange to 10, Yellow to 11, Black to Ground)
SoftwareSerial mySerial(10, 11); // Initialize SoftwareSerial for communication

char dataBuffer[BUFFER_SIZE];  // Buffer to store incoming data
volatile bool newDataAvailable = false;  // Flag to indicate new data availability
unsigned long previousMillis = 0;  // Stores the last time a request was made
const long interval = 10000;  // Interval for periodic actions (in milliseconds)
volatile bool printDetectionsFlag = false;  // Flag to trigger detections printing

// Protothread control structures
struct pt ptRead, ptProcess, ptPrint;

// Setup function: initializes serial communication and protothreads
void setup() {
    Serial.begin(9600);  // Start the Serial communication
    mySerial.begin(9600);  // Start the SoftwareSerial communication
    clearBuffer();  // Clear the data buffer
    delay(30000); // Initial delay for system stabilization

    // Initialize protothreads
    PT_INIT(&ptRead);
    PT_INIT(&ptProcess);
    PT_INIT(&ptPrint);
}

// Main loop function: executes the protothreads
void loop() {
    protothreadRead(&ptRead);
    protothreadProcess(&ptProcess);
    protothreadPrint(&ptPrint);
}

// Protothread for reading data from the SoftwareSerial
static int protothreadRead(struct pt* pt) {
    PT_BEGIN(pt);

    while (1) {
        delay(250);
        mySerial.println("REQUEST");  // Send request command to the external device

        // Wait for a response with a timeout of 5 seconds
        unsigned long startTime = millis();
        while (!mySerial.available() && millis() - startTime < 5000) {
          // Idle loop, waiting for response
        }

        // Read and store the response if available
        if (mySerial.available()) {
            String data = mySerial.readStringUntil('\n');
            data.toCharArray(dataBuffer, BUFFER_SIZE);
            newDataAvailable = true; // Set flag to indicate new data received
        }

        PT_YIELD(pt);  // Yield control to other protothreads
    }

    PT_END(pt);
}

// Protothread for processing the received data
static int protothreadProcess(struct pt* pt) {
    PT_BEGIN(pt);

    while (1) {
        if (newDataAvailable) {
            // Process new data in buffer
            Serial.println("Received Detections");
            processDetections(dataBuffer);
            newDataAvailable = false; // Reset new data flag
            printDetectionsFlag = true;  // Set flag to print detections
        }

        PT_YIELD(pt);  // Yield control to other protothreads
    }

    PT_END(pt);
}

// Protothread for printing the detections
static int protothreadPrint(struct pt* pt) {
    PT_BEGIN(pt);

    while (1) {
        if (printDetectionsFlag) {
            printDetections();  // Print the detections
            printDetectionsFlag = false;  // Reset print flag
        }

        PT_YIELD(pt);  // Yield control to other protothreads
    }

    PT_END(pt);
}


// Function to process the received data and extract individual detections
void processDetections(char data[]) {
    // Tokenize the data string into individual detections using strtok
    char* detection = strtok(data, ";");

    while (detection != NULL) {
        parseDetection(detection);  // Parse each detection string
        detection = strtok(NULL, ";"); // Get the next detection
    }
}

// Function to parse a single detection string into its components
void parseDetection(char* detection) {
    char class_name[MAX_DETECTION_LENGTH];  // Buffer to hold the class name
    float confidence;  // Variable to hold the confidence value
    float depth_mm;  // Variable to hold the depth in millimeters
    float depth_in;  // Variable to hold the depth in inches
    float x, y, z;  // Variables to hold the 3D coordinates
    float horizontal_angle, timestamp;  // Variables for horizontal angle and timestamp
    char direction[MAX_DETECTION_LENGTH];  // Buffer to hold the direction string
    char* token;  // Pointer for tokenization
    char* rest = detection;  // Pointer to keep track of the rest of the string

    // Tokenize the detection string and extract each component
    // The tokens are expected to be separated by commas
    token = strtok_r(rest, ",", &rest);
    // ... (rest of the tokenization and extraction process)

    Detection newDetection(class_name, confidence, timestamp, depth_mm, depth_in, x, y, z, horizontal_angle, direction);
    addDetectionToBuffer(newDetection);  // Add the parsed detection to the buffer
}

// Function to print the detections
void printDetections() {
    // Print the closest detection
    Detection closest = getClosestDetection();
    Serial.println("Closest Detection:");
    printDetection(closest);

    // Print the latest detection
    Detection latest = getLatestDetection();
    Serial.println("Latest Detection:");
    printDetection(latest);

    // Loop through and print all detections in the buffer
    Serial.println("All Detections:");
    for (int i = 0; i < getBufferSize(); i++) {
        Detection d = getDetectionFromBuffer(i);
        printDetection(d);
    }
}

// Function to print the details of a single detection
void printDetection(const Detection& d) {
    if (strlen(d.class_name) > 0) { // Check if the detection data is valid
        // Print each component of the detection
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
