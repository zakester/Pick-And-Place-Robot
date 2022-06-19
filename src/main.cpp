#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>

/** Pins definitions */
#define IN_1 16  // D0
#define IN_2 5   // D1
#define ENA 4    // D2

#define IN_3 0  // D3
#define IN_4 2  // D4
#define ENB 14  // D5

#define ENCODER_PIN 12  // D6

#define ECHO 13 // D7
#define TRIG 15 // D8

/** Define Robot speed */
double speed = 0.0;
int ref = 0.0;

const char *ssid = "PAP Robot";
const char *PAP_ROBOT_HTML = "<html><body><h1>PAP Robot</h1></body></html>";

ESP8266WebServer server(80);

/** To Configure IP Address and make it static */
IPAddress localIP(192, 168, 5, 1);
IPAddress gateway(192, 168, 5, 1);
IPAddress mask(255, 255, 255, 0);

/** Convert string response to JSON */
DynamicJsonDocument strToJSON(String data) {
    DynamicJsonDocument document(1024);
    deserializeJson(document, data);

    return document;
}


void motor(int dir, int pwmPin, int pwmSpeed, int in1, int in2) {
    analogWrite(pwmPin, pwmSpeed); // set speed
    if (dir == 1) { // rotate to clockwise
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (dir == -1) { // rotate to anti clockwise
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else { // stop motor
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
}

long duration;
double distance;
double ultrasonic() {
    // Clears TRIG pin if HIGH
    digitalWrite(TRIG, LOW);
    delayMicroseconds(10);
    // Sets the TRIG to HIGH (ACTIVE) for 10 microseconds
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    // Reads the ECHO pin
    duration = pulseIn(ECHO, HIGH);
    // Calculating the distance
    distance =
        duration * 0.034 / 2; 

    return distance;
}

int dir = 0;
int pos = 0;
bool startUltrasonic = false;
bool startRotation = true;

// Interaption function, if encoder is rise HIGH, pos++
void IRAM_ATTR readEncoder() {
    pos++;
}

bool isReady = false;
bool isCaptured = false;


long prevT = 0;
float eprev = 0;
float eintegral = 0;

void rotateRobot(double kp, double ki, double kd) {

    /** Calculate u(t) PID Controller */

    // time difference
    long currT = micros();
    float deltaT =
        ((float)(currT - prevT)) / (1.0e6);  // 1.0e6 to convert microsec to sec
    prevT = currT;

    // error
    int e = ref - pos;

    // derivative
    float dedt = (e - eprev) / (deltaT);

    // integral
    eintegral = eintegral + e * deltaT;

    // input signal
    float u = kp * e + kd * dedt + ki * eintegral;

    // motor power
    speed = fabs(u);

    // set speed max valeu to 150
    int max_speed = 150;
    if (speed > max_speed) {
        speed = max_speed;
    }

    // start rotating
    if (e >= 0) {
        startUltrasonic = false;
    } else {
        // Turn Off The Motor
        speed = 0;
        dir = 0;
        startUltrasonic = true;
        startRotation = false;
    }

    // signal the motor
    motor(dir, ENA, speed, IN_1, IN_2);

    if (startUltrasonic == true) {
        delay(1000);
        pos = 0; // reset position
    }
    // store previous error
    eprev = e;
}




double robotRotationDegree(double width, double y0, double x0) {
    double degreePerPx = 0.0425;
    double dist = abs(640 - x0);

    return dist * degreePerPx;
}

/* The main page */
void handle_root() {
    if (!server.hasArg("plain")) {
        server.send(200, "text/html", PAP_ROBOT_HTML);
        return;
    }

    String data = server.arg("plain");

    DynamicJsonDocument document = strToJSON(data);

    /** BBox Information */
    String x = document["x"];
    String y = document["y"];
    String label = document["label"];
    String width = document["width"];

    String rotationDegree = document["rotationDegree"];


    // Do Nothing While the App didn't detect anything
    while (x.toInt() == -9999) {
        isReady = false;
        return;
    }

    double rotationDegreeRobot = 0.0;

    // Wait for 1 second till the AI complete processing
    if (!isReady) {
        if (label == "zone" && isCaptured == false) return;
        delay(500);
        pos = 0;
        rotationDegreeRobot =
            robotRotationDegree(width.toDouble(), y.toDouble(), x.toDouble());
        ref = std::round(rotationDegreeRobot / 2.3); 
        if (x.toInt() < 500) {
            dir = -1;
        } else if (x.toInt() > 500) {
            dir = 1;
        }

        if (ref == 0) return;
        if (dir == 1) {
            ref -= 5;
        }


        isReady = true;


        std::string d =
            "{ \"speed\":" + std::to_string(speed/10) +
            ", \"ref\":" + std::to_string(ref) +
            ", \"pos\":" + std::to_string(pos) + "}";
        server.send(200, "text/html", d.c_str());
    } else {
        std::string d =
            "{ \"speed\":" + std::to_string(speed/10) +
            ", \"ref\":" + std::to_string(ref) +
            ", \"pos\":" + std::to_string(pos) + "}";
        server.send(200, "text/html", d.c_str());

        if (label == "zone" && isCaptured == false) return;
        if (startRotation == true) {
            rotateRobot(13, 0.3, 1.6); // Kp = 13, Ki = 0.1, Kd = 1
        } else if (startUltrasonic == true) {
            double dist = ultrasonic();
            // Move foward with ON / OFF Controller
            if (dist >= 30) {
                motor(1, ENA, 100, IN_1, IN_2);
                motor(1, ENB, 100, IN_3, IN_4);
            } else { // stop
                motor(0, ENA, 0, IN_1, IN_2);
                motor(0, ENB, 0, IN_3, IN_4);
                startUltrasonic = false;
            }
        }

    }
}

/* When Page is not found */
void handle_notFound() { server.send(404, "text/html", "Not Found"); }

/* Setup Access */
void setup_AP() {
    /* WiFi Mode Access Point */
    WiFi.mode(WIFI_AP);

    /* Access Point Configuration */
    if (!WiFi.softAPConfig(localIP, gateway, mask)) {
        Serial.println("AP Configuration Faild!");
    } else {
        Serial.println("AP Configuration Sccee!");
    }

    /* Start Access Point */
    WiFi.softAP(ssid);
}

void setup() {
    // LED OUTPUT
    pinMode(16, OUTPUT);

    /** Encoder Input PIN */
    pinMode(ENCODER_PIN, INPUT);

    /** Ultrasonic setup */
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    /** Pins for the first motor */
    pinMode(IN_1, OUTPUT);
    pinMode(IN_2, OUTPUT);
    pinMode(ENA, OUTPUT);

    /** Pins for the second motor */
    pinMode(IN_3, OUTPUT);
    pinMode(IN_4, OUTPUT);
    pinMode(ENB, OUTPUT);
    /** Start Serial at port 115200 */
    Serial.begin(115200);
    while (!Serial)
        ;


    /* Setup and configure Access Point */
    setup_AP();

    /* Server configuration */
    server.on("/", handle_root);
    server.onNotFound(handle_notFound);

    /* Server begin */
    server.begin();
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), readEncoder, RISING);
}

void loop() { server.handleClient(); }