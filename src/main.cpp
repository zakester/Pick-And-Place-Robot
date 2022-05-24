#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <PID_v1.h>

/** Pins definitions */
#define IN_1 16 // D0
#define IN_2 5  // D1
#define ENA 4   // D2

#define IN_3 0 // D3
#define IN_4 2 // D4
#define ENB 14 // D5

/** Define Robot speed */
double speed = 0.0;
double speedRight = 0.0;

const char *ssid = "PAP Robot";
const char *PAP_ROBOT_HTML = "<html><body><h1>MH Robot</h1></body></html>";

ESP8266WebServer server(80);

IPAddress localIP(192, 168, 5, 1);
IPAddress gateway(192, 168, 5, 1);
IPAddress mask(255, 255, 255, 0);

/** Convert string response to JSON */
DynamicJsonDocument strToJSON(String data)
{
  DynamicJsonDocument document(1024);
  deserializeJson(document, data);

  return document;
}

void stopLeftMotor()
{
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, LOW);
}

void stopRightMotor()
{
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, LOW);
}

/** Stop robot */
void stopRobot()
{
  analogWrite(ENB, 0);
  analogWrite(ENA, 0);

  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, LOW);

  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, LOW);
}

/** Move robot foward */
void moveFoward()
{
  /* First motor */
  analogWrite(ENA, speed);
  digitalWrite(IN_1, HIGH);
  digitalWrite(IN_2, LOW);

  /* Second motor */
  analogWrite(ENB, speed);
  digitalWrite(IN_3, HIGH);
  digitalWrite(IN_4, LOW);
}

/** Move robot backwards */
void moveBackward()
{
  /* First motor */
  analogWrite(ENA, speed);
  digitalWrite(IN_1, LOW);
  digitalWrite(IN_2, HIGH);

  /* Second motor */
  analogWrite(ENB, speed);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_4, HIGH);
}

/** Move robot to the right */
void moveToRight()
{
  /* First motor */
  if (speedRight != 0)
  {
    analogWrite(ENB, speedRight);
    digitalWrite(IN_3, HIGH);
    digitalWrite(IN_4, LOW);
  }
  else
  {
    analogWrite(ENB, speed);
    digitalWrite(IN_3, LOW);
    digitalWrite(IN_4, HIGH);
  }

  // /* Move First Motor in Reverse */
  // digitalWrite(IN_1, LOW);
  // digitalWrite(IN_2, HIGH);
}

/** Move robot to the left */
void moveToLeft()
{
  /* Second motor */
  if (speed != 0)
  {
    analogWrite(ENA, speed);
    digitalWrite(IN_1, HIGH);
    digitalWrite(IN_2, LOW);
  }
  else
  {
    analogWrite(ENA, speedRight);
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, HIGH);
  }

  // /* Move First Motor in Reverse */
  // digitalWrite(IN_3, LOW);
  // digitalWrite(IN_4, HIGH);
}

bool isReady = false;
bool isCaptured = false;
bool isMoving = false;
bool isObjectLeftSide = false;
int direction = 0; // 0: Middel, 1: Right, -1: Left
int error = 5;
double targetRotationAngle = 0.0;

double setpointRotation = 0.0;
double currentRotation = 0.0;

void do180Rotation(int currentRotationDegree)
{
  if (!(targetRotationAngle + 360 + 5 <= currentRotationDegree && targetRotationAngle + 360 - 55 <= currentRotationDegree))
  {
    moveToLeft();
  }
  else
    stopRobot();
}
double Kp = 3, Ki = 3, Kd = 0;
PID leftMotorPID(&currentRotation, &speed, &setpointRotation, Kp, Ki, Kd, DIRECT);
PID rightMotorPID(&currentRotation, &speedRight, &setpointRotation, Kp, Ki, Kd, REVERSE);

/* The main page */
void handle_root()
{

  if (!server.hasArg("plain"))
  {
    server.send(200, "text/html", PAP_ROBOT_HTML);
    return;
  }

  String data = server.arg("plain");

  DynamicJsonDocument document = strToJSON(data);

  String x = document["x"];
  String y = document["y"];
  String label = document["label"];
  String width = document["width"];
  String height = document["height"];
  String rotationDegree = document["rotationDegree"];
  String objectAngle = document["objectAngle"];
  String diffAngle = document["diffAngle"];
  currentRotation = map(rotationDegree.toDouble(), 0, 360, 0, 255);
  // Do Nothing While the App didn't detect anything
  while (x.toInt() == -9999)
  {
    isReady = false;
    return;
  }

  // Serial.println("x: " + objectAngle + "diffAngle: " + diffAngle);

  double rotationDegreeRobot = 0.0;

  // Wait for 1 second till the AI complete processing
  if (!isReady)
  {
    if (label == "zone" && !isCaptured)
      return;
    delay(1000);
    rotationDegreeRobot = abs(objectAngle.toDouble() - diffAngle.toDouble());
    if (x.toInt() < 500)
    {
      targetRotationAngle = rotationDegree.toInt() - rotationDegreeRobot;
      direction = -1;
    }
    else if (x.toInt() > 500)
    {
      targetRotationAngle = rotationDegree.toInt() + rotationDegreeRobot;
      direction = 1;
    }
    if (targetRotationAngle == 0)
      return;
    isReady = true;
    setpointRotation = map(targetRotationAngle, 0, 360, 0, 255);
    leftMotorPID.Compute();
    rightMotorPID.Compute();
    Serial.println(targetRotationAngle);
    Serial.println(setpointRotation);

    Serial.print("speed: ");
    Serial.println(speed);
    Serial.println("==========================================");
    std::string d = "{ \"speedLeft\":" + std::to_string(speed) + ", \"speedRight\":" + std::to_string(speedRight) + ", \"setpoint\":" + std::to_string(setpointRotation) + ", \"currentRotation\":" + std::to_string(currentRotation) + "}";
    server.send(200, "text/html", d.c_str());
  }
  else
  {
    leftMotorPID.Compute();
    rightMotorPID.Compute();
    std::string d = "{ \"speedLeft\":" + std::to_string(speed) + ", \"speedRight\":" + std::to_string(speedRight) + ", \"setpoint\":" + std::to_string(setpointRotation) + ", \"currentRotation\":" + std::to_string(currentRotation) + "}";
    server.send(200, "text/html", d.c_str());
    Serial.print("setPointRotation: ");
    Serial.println(setpointRotation);
    Serial.print("currentRotation: ");
    Serial.println(currentRotation);

    Serial.print("speed: ");
    Serial.println(speed);

    Serial.print("speed Right: ");
    Serial.println(speedRight);

    if (label == "zone" && !isCaptured)
      return;
    moveToLeft();
    moveToRight();
  }
}

/* When Page is not found */
void handle_notFound()
{
  server.send(404, "text/html", "Not Found");
}

/* Setup Access */
void setup_AP()
{

  /* WiFi Mode Access Point */
  WiFi.mode(WIFI_AP);

  /* Access Point Configuration */
  if (!WiFi.softAPConfig(localIP, gateway, mask))
  {
    Serial.println("AP Configuration Faild!");
  }
  else
  {
    Serial.println("AP Configuration Sccee!");
  }

  /* Start Access Point */
  WiFi.softAP(ssid);
}

void setup()
{
  // LED OUTPUT
  pinMode(16, OUTPUT);

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
  leftMotorPID.SetOutputLimits(0, 110);
  rightMotorPID.SetOutputLimits(0, 110);
  leftMotorPID.SetMode(AUTOMATIC);
  rightMotorPID.SetMode(AUTOMATIC);
  // Adjust PID values
  leftMotorPID.SetTunings(Kp, Ki, Kd);
  rightMotorPID.SetTunings(Kp, Ki, Kd);
  /* Setup and configure Access Point */
  setup_AP();

  /* Server configuration */
  server.on("/", handle_root);
  server.onNotFound(handle_notFound);

  /* Server begin */
  server.begin();
}

void loop()
{
  server.handleClient();
}