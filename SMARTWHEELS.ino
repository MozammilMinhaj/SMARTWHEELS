#include <AFMotor.h>
#include <Servo.h>

#define Pi 3.14

AF_DCMotor motorLeft(1);
AF_DCMotor motorRight(2);

Servo steering_servo;
Servo ultraSonic_servo;

//Bluetooth variable
const byte btPin = 50;
boolean btConnected = false;


//ultrasonic variables
int trigPin = 38, echoPin = 40, distance, rightDistance, leftDistance;
long pingTime;

//RGB LED variables
int headLight = 0,prevRGBStatus=0;
//int hRedPin = 26, hGreenPin = 24, hBluePin = 22;
//int dRedPin = 32, dGreenPin = 30, dBluePin = 28;
int hRedPin = 32, hGreenPin = 30, hBluePin = 28;
int dRedPin = 26, dGreenPin = 24, dBluePin = 22;

//Mode variables
int setManMode = 0, setAutoMode = 0, flag = 0;
char command;

//Loop variables & others
int i, servo_pos = 90;

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
String data;

//function declaration
int measureDistance();
void printDistance(int d);
void moveForward();
void moveBackward();
void hardRightTurn();
void hardLeftTurn();
void driveToTarget(double steerAngle);
void doNotMove();
void manual();
void automatic();
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
void rgbMagData();                    //Data Indicator color : Magenta when Magnetometer data is received
void rgbDataIndicator(int rgbStatus); //Data Indicator color : White, Blue, Red, Orange, Yellow

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

void printDistance(int d)
{
  Serial.print("The Distance of object is : ");
  Serial.print(d);
  Serial.println(" cm");
}

void moveForward()
{
  servo_pos = 90;
  steering_servo.write(servo_pos);
  motorRight.run(FORWARD);
  motorRight.setSpeed(255);
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(255);
}

void moveBackward()
{
  servo_pos = 90;
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

  for (i = servo_pos; i < 110; i += 6)
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

  for (i = servo_pos; i > 70; i -= 6)
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
  servo_pos = 90;
  steering_servo.write(servo_pos);
  ultraSonic_servo.write(servo_pos);
  motorRight.run(RELEASE);
  motorLeft.run(RELEASE);
}

void manual()
{
  Serial.println("Car is in manual Mode");
  rgbDataIndicator(1);
  prevRGBStatus=1;
  setManMode = 1;
  doNotMove();
  servo_pos = 90;
  ultraSonic_servo.write(servo_pos);
  steering_servo.write(servo_pos);

  while (setManMode && !setAutoMode)
  {
    if ( digitalRead(btPin) == HIGH)
    {
      btConnected = true;
      command = Serial.read();
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
        case 'A' : setAutoMode = 1; setManMode = 0;
          break;
        case 'M' : setAutoMode = 0; setManMode = 1;
          break;
        default: Serial.println("Waiting for instructions... ");
          rgbHeadLightToggle();
      }
      delay(400);
    }
    else
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
  }
}

void automatic()
{
  Serial.println("Car is in Automatic Mode");
  rgbDataIndicator(2);
  prevRGBStatus=2;
  setAutoMode = 1;
  doNotMove();
  destSet = 0;
  gpsInitialized = 0;
  servo_pos = 90;
  ultraSonic_servo.write(servo_pos);
  steering_servo.write(servo_pos);

  while (setAutoMode && !setManMode)
  {
    if ( digitalRead(btPin) == HIGH)
    {
      btConnected = true;
      if (Serial.available() > 0)
      {
        command = Serial.read();
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
          case 'M' : setAutoMode = 0; setManMode = 1;
            break;
        }
      }

      rgbHeadLightToggle();
      if (!destSet )
      {
        readDestination();
      }

      distance = measureDistance();
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
      //delay(1000);
    }
    else
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
    delay(2000);
    while ((temp = measureDistance()) < 30)
    {
      Serial.println("Moving Forward");
      moveForward();
      delay(2000);
    }
    Serial.println("Taking Left Turn");
    hardLeftTurn();
    delay(2000);
    Serial.println("Dodging complete");
  }
  else
  {
    Serial.println("Dodging From Left");
    Serial.println("Taking Left Turn");
    ultraSonic_servo.write(0);
    hardLeftTurn();
    delay(2000);
    while ((temp = measureDistance()) < 30)
    {
      Serial.println("Moving Forward");
      moveForward();
      delay(2000);
    }
    Serial.println("Taking Right Turn");
    hardRightTurn();
    delay(2000);
    Serial.println("Dodging complete");
  }
  doNotMove();

}

void readGPS()
{
  gpsStatus = 0;
  dotStatus = 0;

  while (gpsStatus == 0)
  {
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
        prevRGBStatus=4;
        dotStatus = 0;
      }
    }
    else
    {
      Serial.println("Waiting for GPS data");
      rgbDataIndicator(4);
      prevRGBStatus=4;
    }
    location = "";
    //delay(1000);
  }
}

void readMagnetometer()
{
  magnetoStatus = 0;
  dotStatus = 0;

  while (magnetoStatus == 0)
  {
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
        prevRGBStatus=4;
        dotStatus = 0;
      }
    }
    else
    {
      Serial.println("Waiting for Magnetometer data");
      rgbDataIndicator(4);
      prevRGBStatus=4;
    }
    //delay(1000);

    distance = measureDistance();
    printDistance(distance);
    if (distance <= 30)
    {
      rgbObsDetected();
      dodgeObstacle();
    }
    rgbHeadLightToggle();
  }
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
      if (Serial.available() > 0)
      {
        destLocation = Serial.readString();
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
          }
        }
        if (dotStatus == 2)
        {
          Serial.println("Valid Destination GPS data");
          destStatus = 1;
          destSet = 1;
          rgbDataIndicator(3);
          prevRGBStatus=3;

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
      delay(200);

      distance = measureDistance();
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
  }
}

void classifyReading()
{
  dataStatus = 0;
  int mBegin = 0, g = 0;
  while (dataStatus == 0 && setAutoMode)
  {
    if ( digitalRead(btPin) == HIGH)
    {
      if (Serial.available() > 0)
      {
        Serial.println("Inside classify Reading 1");
        Serial.setTimeout(20);
        data = Serial.readString();
        Serial.println("Inside classify Reading 2");
        Serial.println(data);
        for (i = 0; i < data.length(); i++)
        {
          if (data[i] == 'g')
          {
            gCount++;
            if (gCount == 1)
            {
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
          }
        }

        Serial.print("GPS Data Count : ");
        Serial.println(gCount);
        Serial.print("Mag Data Count : ");
        Serial.println(mCount);

        if (gCount > 0)
        {
          if (gBoundary - 19 >= 0)
          {
            for (i = (gBoundary - 19); i <= gBoundary; i++)
            {
              location.concat(data[i]);
            }
            dataStatus = 1;
            //gpsInitialized = 1;
            Serial.println("Inside classify Reading read gps");
            Serial.println(location);
            readGPS();
          }
        }
        if (mCount > 0)
        {
          for (i = mBegin; i <= mBoundary; i++)
          {
            stringMag.concat(data[i]);
          }
          dataStatus = 1;
          Serial.println("Inside classify Reading read mag");
          Serial.println(stringMag);
          readMagnetometer();
        }
      }
      else
      {
        Serial.println("Send GPS or Magnetometer data");
        rgbDataIndicator(4);
        prevRGBStatus=4;
      }
      gCount = 0;
      mCount = 0;
      location = "";
      stringMag = "";
      data = "";
      //delay(1000);

      distance = measureDistance();
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
      //steering_servo.write(90); // move straight for +/- 10deg
      Serial.println("Steering Straight");
      moveForward();

    }
    else if (dir > 10 && dir <= 90)
    {
      steerAngle = ((dir - 10) / 4) + 90;
      //steering_servo.write(steerAngle); //soft right turn : destination lies in 1st quadrant
      Serial.println("Driving towards target");
      driveToTarget(steerAngle);
    }
    else
    {
      //steering_servo.write(110); //hard right turn : destination lies in 2nd quadrant
      Serial.println("Steering Right");
      hardRightTurn();
    }
  }
  else //left side
  {
    if (dir < 0 && dir >= -10)
    {
      //steering_servo.write(90); // move straight for +/- 10deg
      Serial.println("Steering Straight");
      moveForward();
    }
    else if (dir < -10 && dir >= -90)
    {
      steerAngle = ((dir + 10) / 4) + 90;
      //steering_servo.write(steerAngle); // soft left turn : destination lies in 4th quadrant
      Serial.println("Driving towards target");
      driveToTarget(steerAngle);
    }
    else
    {
      //steering_servo.write(70); //hard left turn : destination lies in 3rd quadrant
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
    /*delay(200);
      analogWrite(hRedPin, 255);
      analogWrite(hGreenPin, 255);
      analogWrite(hBluePin, 0);
      delay(200);*/
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
  delay(200);
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
    case 3: analogWrite(dRedPin, 0);      //automatic mode destination set - Color : orange
      analogWrite(dGreenPin, 128);
      analogWrite(dBluePin, 255);
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

  //analogWrite(dRedPin, 0);      //waiting for gps/mag data - Color : yellow
  //analogWrite(dGreenPin, 0);
  //analogWrite(dBluePin, 255);
  //delay(50);
}

void setup()
{
  Serial.begin(9600);

  steering_servo.attach(9); //For Steering
  ultraSonic_servo.attach(10); //For ultrasonic

  steering_servo.write(90);
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

  Serial.println("Connect Bluetooth to Android device");
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
      delay(200);
      if (Serial.available() > 0)
      {
        command = Serial.read();
        flag = 1;
      }
      else
      {
        Serial.println("Waiting for Mode Selection");
        prevRGBStatus=0;
        rgbDataIndicator(0);
        flag = 0;
      }
    }
    else
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
  }
  if (command == 'M')
  {
    Serial.println("Car Set to manual Mode");
    manual();
  }
  else if (command == 'A')
  {
    Serial.println("Car Set to Automatic Mode");
    automatic();
  }
}
