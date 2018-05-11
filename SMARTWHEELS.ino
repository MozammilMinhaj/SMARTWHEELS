#include <AFMotor.h>
#include <Servo.h>

#define Pi 3.14
//Steering Servo degree
#define MAX_LEFT 80
#define CENTER 100
#define MAX_RIGHT 120

AF_DCMotor motorLeft(1);
AF_DCMotor motorRight(2);

Servo steering_servo;
Servo ultraSonic_servo;

//Bluetooth variable
const byte btPin = 50;
boolean btConnected = false;

//ultrasonic variables
int trigPin = 38, echoPin = 40, distance, rightDistance, leftDistance, d1, d2, d3, aDistance;
long pingTime;

//RGB LED variables
int headLight = 0, prevRGBStatus = 0;
int hRedPin = 26, hGreenPin = 24, hBluePin = 22;
int dRedPin = 34, dGreenPin = 32, dBluePin = 30;

//Mode variables
int setManMode = 0, setAutoMode = 0, flag = 0;
char command;

//Loop variables & others
int i, j, servo_pos = CENTER;

//GPS variables
int gpsStatus, dotStatus, lonPos;
String location, stringLat, stringLon;
double sourceX, sourceY;
int gpsInitialized = 0;

//destination data variables
String destLocation, stringDestX, stringDestY;
double destX, destY;
int destStatus, destSet = 0;

//Navigate variables
double dx, dy, destRad, destDeg, heading, dir;

//Magnetometer variables
String stringMag;
int magnetoStatus;
double currentDir;

//classifyReading variables
int dataStatus, gCount, mCount, gBoundary, mBoundary;
int mBegin, gBegin, dot1, dot2;
String data;

//function declaration
int measureDistance();
int avgDistance();
void printDistance(int d);
void moveForward();
void moveBackward();
void hardRightTurn();
void hardLeftTurn();
void driveToTarget(double steerAngle);
void doNotMove();
void manual();
void automatic();
int checkMode();
void checkBtConnection();
void dodgeObstacle();
void readGPS();
void readMagnetometer();
void readDestination();
void classifyReading();
void navigate();
void stopNavigation();
void rgbHeadLightOn();                //Headlight on
void rgbHeadLightOff();               //Headlight off
void rgbHeadLightToggle();            //Headlight color : White
void rgbObsDetected();                //Headlight color : Red
void rgbGpsData();                    //Data Indicator color : Green when gps data is received
void rgbMagData();                    //Data Indicator color : Magenta when magnetometer data is received
void rgbDataIndicator(int rgbStatus); //Data Indicator color : Red, White, Blue, Cyan, Green, Yellow

int measureDistance()
{
  int tempDistance;
  delay(100);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pingTime = pulseIn(echoPin, HIGH);
  tempDistance = pingTime * 0.034 / 2;
  return tempDistance;
}

int avgDistance()
{
  d1 = measureDistance();
  d2 = measureDistance();
  d3 = measureDistance();
  aDistance = (d1 + d2 + d3) / 3;
  return aDistance;
}

void printDistance(int d)
{
  Serial.print("The Distance of object is : ");
  Serial.print(d);
  Serial.println(" cm");
}

void moveForward()
{
  servo_pos = CENTER;
  steering_servo.write(servo_pos);
  motorRight.run(FORWARD);
  motorRight.setSpeed(255);
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(255);
}

void moveBackward()
{
  servo_pos = CENTER;
  steering_servo.write(servo_pos);
  motorRight.run(BACKWARD);
  motorRight.setSpeed(255);
  motorLeft.run(BACKWARD);
  motorLeft.setSpeed(255);
}

void hardRightTurn()
{
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(255);
  motorRight.run(FORWARD);
  motorRight.setSpeed(255);

  for (i = servo_pos; i < MAX_RIGHT; i += 6)
  {
    steering_servo.write(i);
    servo_pos = i;
    delay(100);
  }
}

void hardLeftTurn()
{
  motorRight.run(FORWARD);
  motorRight.setSpeed(255);
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(255);

  for (i = servo_pos; i > MAX_LEFT; i -= 6)
  {
    steering_servo.write(i);
    servo_pos = i;
    delay(100);
  }
}

void driveToTarget(double steerAngle)
{
  motorRight.run(FORWARD);
  motorRight.setSpeed(255);
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(255);

  Serial.print("Steering at ");
  Serial.print(steerAngle);
  Serial.println(" deg");
  steering_servo.write(steerAngle);
  servo_pos = steerAngle;
}

void doNotMove()
{
  servo_pos = CENTER;
  steering_servo.write(servo_pos);
  ultraSonic_servo.write(90);
  motorRight.run(RELEASE);
  motorLeft.run(RELEASE);
}

void manual()
{
  Serial.println("Car is in Manual Mode");
  rgbDataIndicator(1);
  prevRGBStatus = 1;
  setManMode = 1;
  doNotMove();
  servo_pos = CENTER;
  ultraSonic_servo.write(90);
  steering_servo.write(servo_pos);

  while (setManMode && !setAutoMode)
  {
    if ( digitalRead(btPin) == HIGH)
    {
      btConnected = true;
      if (Serial1.available() > 0)
      {
        command = Serial1.read();
        switch (command)
        {
          case 'F' : Serial.println("Moving Forward ");
            moveForward();
            break;
          case 'B' : Serial.println("Moving Backward ");
            moveBackward();
            break;
          case 'L' : Serial.println("Moving Left ");
            hardLeftTurn();
            break;
          case 'R' : Serial.println("Moving Right ");
            hardRightTurn();
            break;
          case 'S' : Serial.println("Stopped");
            doNotMove();
            break;
          case 'O' : Serial.println("Lights ON");
            rgbHeadLightOn();
            break;
          case 'o' : Serial.println("Lights OFF");
            rgbHeadLightOff();
            break;
          case 'A' : setAutoMode = 1; setManMode = 0; return;
            break;
          case 'M' : setAutoMode = 0; setManMode = 1;
            break;
        }
        rgbHeadLightToggle();
      }
      Serial.println("Waiting for instructions... ");
      distance = avgDistance();
      printDistance(distance);
    }
    else
    {
      checkBtConnection();
    }
  }
}

void automatic()
{
  Serial.println("Car is in Automatic Mode");
  rgbDataIndicator(2);
  prevRGBStatus = 2;
  setAutoMode = 1;
  doNotMove();
  destSet = 0;
  gpsInitialized = 0;
  servo_pos = CENTER;
  ultraSonic_servo.write(90);
  steering_servo.write(servo_pos);

  while (setAutoMode && !setManMode)
  {
    if ( digitalRead(btPin) == HIGH)
    {
      btConnected = true;
      if (Serial1.available() > 0)
      {
        command = Serial1.read();
        switch (command)
        {
          case 'O' : Serial.println("Lights ON");
            rgbHeadLightOn();
            break;
          case 'o' : Serial.println("Lights OFF");
            rgbHeadLightOff();
            break;
          case 'A' : setAutoMode = 1; setManMode = 0;
            break;
          case 'M' : setAutoMode = 0; setManMode = 1; return ;
            break;
        }
      }

      rgbHeadLightToggle();
      if (!destSet )
      {
        readDestination();
      }

      distance = avgDistance();
      printDistance(distance);
      if (distance <= 30)
      {
        rgbObsDetected();
        dodgeObstacle();
      }
      rgbHeadLightToggle();

      while (!gpsInitialized && setAutoMode)
      {
        classifyReading();
      }
      classifyReading();

      if (((destX - sourceX) == 0) && ((destY - sourceY) == 0) && setAutoMode)
      {
        Serial.println("Car Has Reached Destination");
        stopNavigation();
        gpsInitialized = 0;
        destSet = 0;
        flag = 0;
        setAutoMode = 0;
      }
      else if ( setAutoMode )
      {
        navigate();
      }
    }
    else
    {
      checkBtConnection();
    }
  }
}

int checkMode()
{
  if ( digitalRead(btPin) == HIGH)
  {
    btConnected = true;
    if (Serial1.available() > 0)
    {
      command = Serial1.read();
      switch (command)
      {
        case 'A' : setAutoMode = 1; setManMode = 0; return 1;
          break;
        case 'M' : setAutoMode = 0; setManMode = 1; return 2;
          break;
      }
    }
  }
  else
  {
    checkBtConnection();
  }
  return 0;
}

void checkBtConnection()
{
  rgbDataIndicator(5);
  doNotMove();
  btConnected = false;
  Serial.println("Bluetooth is Disconnected : Connecting...");
  while (!btConnected)
  {
    if ( digitalRead(btPin) == HIGH)
    {
      btConnected = true;
      rgbDataIndicator(prevRGBStatus);
    }
  }
}

void dodgeObstacle()
{
  int temp;
  doNotMove();

  ultraSonic_servo.write(0);
  delay(500);
  rightDistance = measureDistance();
  Serial.println("Right Object Distance ");
  printDistance(rightDistance);

  ultraSonic_servo.write(180);
  delay(500);
  leftDistance = measureDistance();
  Serial.println("Left Object Distance");
  printDistance(leftDistance);

  moveBackward();
  delay(2000);

  if (rightDistance >= leftDistance)
  {
    Serial.println("Dodging From Right");
    Serial.println("Taking Right Turn");
    ultraSonic_servo.write(180);
    hardRightTurn();
    if (checkMode())
      return;
    delay(2000);
    while ((temp = measureDistance()) < 30)
    {
      Serial.println("Moving Forward");
      moveForward();
      if (checkMode())
        return;
      delay(2000);
    }
    Serial.println("Taking Left Turn");
    hardLeftTurn();
    if (checkMode())
      return;
    delay(2000);
    Serial.println("Dodging complete");
  }
  else
  {
    Serial.println("Dodging From Left");
    Serial.println("Taking Left Turn");
    ultraSonic_servo.write(0);
    hardLeftTurn();
    if (checkMode())
      return;
    delay(2000);
    while ((temp = measureDistance()) < 30)
    {
      Serial.println("Moving Forward");
      moveForward();
      if (checkMode())
        return;
      delay(2000);
    }
    Serial.println("Taking Right Turn");
    hardRightTurn();
    if (checkMode())
      return;
    delay(2000);
    Serial.println("Dodging complete");
  }
  doNotMove();
}

void readGPS()
{
  gpsStatus = 0;
  dotStatus = 0;

  if (location.length() > 0)
  {
    for (i = 0; i < location.length(); i++)
    {
      if (location[i] == '.')
      {
        dotStatus++;
        if (dotStatus == 2)
        {
          lonPos = i - 2;
        }
      }
    }
    if (dotStatus == 2)
    {
      gpsStatus = 1;
      Serial.println("Valid GPS data read");
      rgbGpsData();

      for (i = 0; i < lonPos; i++)
      {
        stringLat.concat(location[i]);
      }

      for (i = lonPos; i < location.length(); i++)
      {
        stringLon.concat(location[i]);
      }

      sourceX = stringLat.toDouble();
      sourceY = stringLon.toDouble();
      Serial.println("Current position is ");
      Serial.print("Latitude : ");
      Serial.println(sourceX, 4);
      Serial.print("Longitude : ");
      Serial.println(sourceY, 4);

      gpsInitialized = 1;
    }
    else
    {
      Serial.println("Not a GPS data");
      Serial.println("Waiting for GPS data");
      rgbDataIndicator(4);
      prevRGBStatus = 4;
      dotStatus = 0;
    }
  }
  else
  {
    Serial.println("Waiting for GPS data");
    rgbDataIndicator(4);
    prevRGBStatus = 4;
  }
  location = "";
}

void readMagnetometer()
{
  magnetoStatus = 0;
  dotStatus = 0;

  if (stringMag.length() > 0)
  {
    for (i = 0; i < stringMag.length(); i++)
    {
      if (stringMag[i] == '.')
      {
        dotStatus++;
      }
    }
    if (dotStatus == 1)
    {
      Serial.println("Valid Magnetometer data");
      rgbMagData();
      currentDir = stringMag.toDouble();
      magnetoStatus = 1;
    }
    else
    {
      Serial.println("Not a valid Magnetometer data");
      Serial.println("waiting for Magnetometer data");
      rgbDataIndicator(4);
      prevRGBStatus = 4;
      dotStatus = 0;
    }
  }
  else
  {
    Serial.println("Waiting for Magnetometer data");
    rgbDataIndicator(4);
    prevRGBStatus = 4;
  }

  distance = avgDistance();
  printDistance(distance);
  if (distance <= 30)
  {
    rgbObsDetected();
    dodgeObstacle();
  }
  rgbHeadLightToggle();
}

void readDestination()
{
  destStatus = 0;
  dotStatus = 0;

  while (destStatus == 0 && setAutoMode )
  {
    if ( digitalRead(btPin) == HIGH)
    {
      btConnected = true;
      if (Serial1.available() > 0)
      {
        destLocation = Serial1.readString();
        Serial.println(destLocation);
        for (i = 0; i < destLocation.length(); i++)
        {
          if (destLocation[i] == '.')
          {
            dotStatus++;
            if (dotStatus == 2)
            {
              lonPos = i - 2;
            }
          }
          else if ( destLocation[i] == 'O' )
          {
            rgbHeadLightOn();
            command = 'O';
          }
          else if ( destLocation[i] == 'o' )
          {
            rgbHeadLightOff();
            command = 'o';
          }
          else if ( destLocation[i] == 'A' )
          {
            setAutoMode = 1;
            setManMode = 0;
            command = 'A';
          }
          else if ( destLocation[i] == 'M' )
          {
            setAutoMode = 0;
            setManMode = 1;
            command = 'M';
            return  ;
          }
        }
        if (dotStatus == 2)
        {
          Serial.println("Valid Destination GPS data");
          destStatus = 1;
          destSet = 1;
          rgbDataIndicator(3);
          prevRGBStatus = 3;

          for (i = 0; i < lonPos; i++)
          {
            stringDestX.concat(destLocation[i]);
          }

          for (i = lonPos; i < destLocation.length(); i++)
          {
            stringDestY.concat(destLocation[i]);
          }

          destX = stringDestX.toDouble();
          destY = stringDestY.toDouble();
          Serial.println("Destination is Set to ");
          Serial.print("Latitude : ");
          Serial.println(destX, 4);
          Serial.print("Longitude : ");
          Serial.println(destY, 4);
        }
        else
        {
          Serial.println("Not a valid Destination GPS data");
          Serial.println("Waiting for Destination GPS data");
          dotStatus = 0;
          destStatus = 0;
        }
      }
      else
      {
        Serial.println("Waiting for Destination GPS data");
        destStatus = 0;
      }

      distance = avgDistance();
      printDistance(distance);
      if (distance <= 30)
      {
        rgbObsDetected();
        dodgeObstacle();
      }
      rgbHeadLightToggle();
    }
    else
    {
      checkBtConnection();
    }
  }
}

void classifyReading()
{
  dataStatus = 0;
  mBegin = 0; gBegin = 0; dot1 = 0; dot2 = 0;
  while (dataStatus == 0 && setAutoMode)
  {
    if ( digitalRead(btPin) == HIGH)
    {
      if (Serial1.available() > 0)
      {
        Serial1.setTimeout(30);
        data = Serial1.readString();
        for (i = 0; i < data.length(); i++)
        {
          if ( data[i] == '.')
          {
            dot1 = dot2;
            dot2 = i;
          }
          else if (data[i] == 'g')
          {
            gCount++;
            if (gCount == 1)
            {
              gBegin = ((dot1 - 2) >= 0) ? (dot1 - 2) : 0 ;
              gBoundary = i - 1;
            }
          }
          else if (data[i] == 'm')
          {
            mCount++;
            if (mCount == 1)
            {
              mBegin = i + 1;
            }
            else if (mCount == 2)
            {
              mBoundary = i - 1;
            }
          }
          else if ( data[i] == 'O' )
          {
            rgbHeadLightOn();
            command = 'O';
          }
          else if ( data[i] == 'o' )
          {
            rgbHeadLightOff();
            command = 'o';
          }
          else if ( data[i] == 'A' )
          {
            setAutoMode = 1;
            setManMode = 0;
            command = 'A';
          }
          else if ( data[i] == 'M' )
          {
            setAutoMode = 0;
            setManMode = 1;
            command = 'M';
            return ;
          }
        }

        if (gCount > 0)
        {
          for (i = gBegin; i <= gBoundary; i++)
          {
            location.concat(data[i]);
          }
          dataStatus = 1;
          readGPS();
        }
        if (mCount > 0)
        {
          for (i = mBegin; i <= mBoundary; i++)
          {
            stringMag.concat(data[i]);
          }
          dataStatus = 1;
          readMagnetometer();
        }
      }
      else
      {
        Serial.println("Send GPS or Magnetometer data");
        rgbDataIndicator(4);
        prevRGBStatus = 4;
      }
      gCount = 0;
      mCount = 0;
      location = "";
      stringMag = "";
      data = "";
      mBegin = 0;
      gBegin = 0;
      dot1 = 0;
      dot2 = 0;

      distance = avgDistance();
      printDistance(distance);
      if (distance <= 30)
      {
        rgbObsDetected();
        dodgeObstacle();
      }
      rgbHeadLightToggle();
    }
    else
    {
      checkBtConnection();
    }
  }
}

void navigate()
{
  dx = destX - sourceX;
  dy = destY - sourceY;

  destRad = atan2(dy, dx);
  destDeg = destRad * (180 / Pi);
  destDeg = destDeg < 0 ? destDeg + 360 : destDeg;

  heading = currentDir;
  Serial.print("\nHeading towards: ");
  Serial.print(heading);
  Serial.println(" deg from North");

  dir = destDeg - heading;
  if (dir < 0)
    dir += 360;
  if (dir > 180)
    dir -= 360;

  double steerAngle;
  if (dir >= 0 && dir <= 180) //right side
  {
    if (dir >= 0 && dir <= 10)
    {
      // move straight for +/- 10deg
      Serial.println("Steering Straight");
      moveForward();

    }
    else if (dir > 10 && dir <= 90)
    {
      //soft right turn : destination lies in 1st quadrant
      steerAngle = ((dir - 10) / 4) + CENTER;
      Serial.println("Driving towards target");
      driveToTarget(steerAngle);
    }
    else
    {
      //hard right turn : destination lies in 2nd quadrant
      Serial.println("Steering Right");
      hardRightTurn();
    }
  }
  else //left side
  {
    if (dir < 0 && dir >= -10)
    {
      // move straight for +/- 10deg
      Serial.println("Steering Straight");
      moveForward();
    }
    else if (dir < -10 && dir >= -90)
    {
      // soft left turn : destination lies in 4th quadrant
      steerAngle = ((dir + 10) / 4) + CENTER;
      Serial.println("Driving towards target");
      driveToTarget(steerAngle);
    }
    else
    {
      //hard left turn : destination lies in 3rd quadrant
      Serial.println("Steering Left");
      hardLeftTurn();
    }
  }
}

void stopNavigation()
{
  doNotMove();
}

void rgbHeadLightOn()
{
  headLight = 1;
  pinMode(hRedPin, OUTPUT);
  pinMode(hGreenPin, OUTPUT);
  pinMode(hBluePin, OUTPUT);
}

void rgbHeadLightOff()
{
  headLight = 0;
  pinMode(hRedPin, LOW);
  pinMode(hGreenPin, LOW);
  pinMode(hBluePin, LOW);
}

void rgbHeadLightToggle()
{
  if (headLight)
  {
    analogWrite(hRedPin, 0);
    analogWrite(hGreenPin, 0);
    analogWrite(hBluePin, 0);
  }
}
void rgbObsDetected()
{
  if (headLight)
  {
    analogWrite(hRedPin, 0);
    analogWrite(hGreenPin, 255);
    analogWrite(hBluePin, 255);
  }
}
void rgbGpsData()
{
  //green color indicating gps data received
  analogWrite(dRedPin, 255);
  analogWrite(dGreenPin, 0);
  analogWrite(dBluePin, 255);
  delay(200);
}
void rgbMagData()
{
  //magenta color indicating magnetometer data received
  analogWrite(dRedPin, 0);
  analogWrite(dGreenPin, 255);
  analogWrite(dBluePin, 0);
  delay(100);
}
void rgbDataIndicator(int rgbStatus)
{
  switch (rgbStatus)
  {
    case 0: analogWrite(dRedPin, 0);    //waiting for mode selection - Color : white
      analogWrite(dGreenPin, 0);
      analogWrite(dBluePin, 0);
      break;
    case 1: analogWrite(dRedPin, 255);    //manual mode - Color : blue
      analogWrite(dGreenPin, 255);
      analogWrite(dBluePin, 0);
      break;
    case 2: analogWrite(dRedPin, 255);      //automatic mode - Color : cyan
      analogWrite(dGreenPin, 0);
      analogWrite(dBluePin, 0);
      break;
    case 3: analogWrite(dRedPin, 255);      //automatic mode destination set - Color : green
      analogWrite(dGreenPin, 0);
      analogWrite(dBluePin, 255); delay(200);
      break;
    case 4: analogWrite(dRedPin, 0);      //waiting for gps/mag data - Color : yellow
      analogWrite(dGreenPin, 0);
      analogWrite(dBluePin, 255);
      break;
    case 5: analogWrite(dRedPin, 0);      //waiting for bluetooth connection - Color : red
      analogWrite(dGreenPin, 255);
      analogWrite(dBluePin, 255);
      break;
    default: analogWrite(dRedPin, 255);      //default - Color : Off
      analogWrite(dGreenPin, 255);
      analogWrite(dBluePin, 255);
      break;
  }
}

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  steering_servo.attach(9); //For Steering
  ultraSonic_servo.attach(10); //For ultrasonic

  steering_servo.write(CENTER);
  ultraSonic_servo.write(90);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(hRedPin, OUTPUT);
  pinMode(hGreenPin, OUTPUT);
  pinMode(hBluePin, OUTPUT);

  pinMode(dRedPin, OUTPUT);
  pinMode(dGreenPin, OUTPUT);
  pinMode(dBluePin, OUTPUT);

  pinMode(btPin, INPUT);

  rgbHeadLightOff();

  motorRight.setSpeed(255);
  motorLeft.setSpeed(255);
  motorRight.run(RELEASE);
  motorLeft.run(RELEASE);

  Serial.println("Connect Android Device to Bluetooth");
  while (!btConnected)
  {
    rgbDataIndicator(5);
    if ( digitalRead(btPin) == HIGH)  {
      btConnected = true;
      Serial.println("Bluetooth is now Connected");
    }
  }
}

void loop()
{
  while (!flag) {
    if ( digitalRead(btPin) == HIGH)
    {
      btConnected = true;
      if (Serial1.available() > 0)
      {
        command = Serial1.read();
        flag = 1;
      }
      else
      {
        Serial.println("Waiting for Mode Selection");
        prevRGBStatus = 0;
        rgbDataIndicator(0);
        flag = 0;
      }
    }
    else
    {
      checkBtConnection();
    }
  }
  if (command == 'M')
  {
    Serial.println("Car Set to Manual Mode");
    manual();
  }
  else if (command == 'A')
  {
    Serial.println("Car Set to Automatic Mode");
    automatic();
  }
}
