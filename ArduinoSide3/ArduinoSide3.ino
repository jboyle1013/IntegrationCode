#include <pt.h>
#include <DetectionsBuffer.h>
#include <SoftwareSerial.h>
#define BUFFER_SIZE 512
#define MAX_DETECTION_LENGTH 15

SoftwareSerial mySerial(10, 11); // RX, TX pins
char dataBuffer[BUFFER_SIZE];
volatile bool newDataAvailable = false;
unsigned long previousMillis = 0;  // Stores the last time a request was made
const long interval = 10000;      
volatile bool printDetectionsFlag = false;



struct pt ptRead, ptProcess, ptPrint;


void setup() {
    Serial.begin(9600);
    mySerial.begin(9600);
    clearBuffer();
    delay(3000); // Wait for everything to stabilize

    PT_INIT(&ptRead);
    PT_INIT(&ptProcess);
    PT_INIT(&ptPrint);
}


void loop() {
    protothreadRead(&ptRead);
    protothreadProcess(&ptProcess);
    protothreadPrint(&ptPrint);
}


static int protothreadRead(struct pt* pt) {
    PT_BEGIN(pt);

    while (1) {
        delay(75);
        mySerial.println("REQUEST");

        // Wait for a response with a timeout
        unsigned long startTime = millis();
        while (!mySerial.available() && millis() - startTime < 5000) {
          // Waiting for response with 5 seconds timeout
        }
        // Read and store the response
        if (mySerial.available()) {
            String data = mySerial.readStringUntil('\n');
            data.toCharArray(dataBuffer, BUFFER_SIZE);
            newDataAvailable = true; // Set the flag to indicate new data
        }

        PT_YIELD(pt);  // Allow other protothreads to run
    }

    PT_END(pt);
}


static int protothreadProcess(struct pt* pt) {
    PT_BEGIN(pt);

    while (1) {
        if (newDataAvailable) {
            // Process the data in dataBuffer
            Serial.println("Received Detections");
            // Serial.println(dataBuffer);
            processDetections(dataBuffer);
            newDataAvailable = false; // Reset the flag after processing
            printDetectionsFlag = true;
        }

        PT_YIELD(pt);  // Allow other protothreads to run
    }

    PT_END(pt);
}

static int protothreadPrint(struct pt* pt) {
    PT_BEGIN(pt);

    while (1) {
        // Check if there are detections to print
        if (printDetectionsFlag) {
            printDetections();
            printDetectionsFlag = false; // Reset the flag
        }

        PT_YIELD(pt); // Allow other protothreads to run
    }

    PT_END(pt);
}



void processDetections(char data[]) {
    // Tokenize the data string into individual detections using strtok
    char* detection = strtok(data, ";");

    while (detection != NULL) {
        // Serial.println(detection);
        parseDetection(detection);
        detection = strtok(NULL, ";"); // Get next detection
    }
}


void parseDetection(char* detection) {
    char class_name[MAX_DETECTION_LENGTH];
    float confidence;
    float depth_mm;
    float depth_in;
    float x, y, z;
    float horizontal_angle, timestamp;
    char direction[MAX_DETECTION_LENGTH];
    char* token;
    char* rest = detection;

        token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        strncpy(class_name, token, MAX_DETECTION_LENGTH);
    }

    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        confidence = atof(token);
    }

    // Continue for the rest of the fields
    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        timestamp = atof(token);
    }

    // Continue for the rest of the fields
    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        depth_mm = atof(token);
    }

    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        x = atof(token);
    }

    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        y = atof(token);
    }

    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        z = atof(token);
    }

    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        horizontal_angle = atof(token);
    }

    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        strncpy(direction, token, MAX_DETECTION_LENGTH);
    }

    // Now you can print or process the parsed data
    // Serial.print("Class Name: ");
    // Serial.println(class_name);
    // Serial.print("Confidence: ");
    // Serial.println(confidence, 4);
    // Serial.print("Timestamp: ");
    // Serial.println(timestamp, 4);
    // Serial.print("Depth MM: ");
    // Serial.println(depth_mm, 4);
    // Serial.print("X Component: ");
    // Serial.println(x, 4);
    // Serial.print("Y Component: ");
    // Serial.println(y, 4);
    // Serial.print("Z Component: ");
    // Serial.println(z, 4);
    // Serial.print("Horizontal Angle: ");
    // Serial.println(horizontal_angle, 4);
    // Serial.print("Direction: ");
    // Serial.println(direction);

    Detection newDetection(class_name, confidence,timestamp, depth_mm, depth_in, x, y, z, horizontal_angle, direction);
    // Serial.println("Newest Detection");
    // printDetection(newDetection);
    addDetectionToBuffer(newDetection);

}

void printDetections() {
    // Print the closest detection
    Detection closest = getClosestDetection();
    Serial.println("Closest Detection:");
    printDetection(closest);

    // Print the latest detection - Not Right Now - Working on it
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

