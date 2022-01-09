#include <Stepper.h>
#include <SD.h>
#include <Servo.h>
#include <SPI.h>

#define baud 57600
#define stepDelay  2.0
#define stepsPerTurn 2038
#define csPin 10
#define step2mm 0.02
#define xBorderPin 3
#define yBorderPin 2

#define stepperXPin1 4
#define stepperXPin2 5
#define stepperXPin3 6
#define stepperXPin4 7
#define stepperYPin1 14
#define stepperYPin2 15
#define stepperYPin3 16
#define stepperYPin4 17

Servo servo;
int servoRetractedPos = 70;
int servoNotRetractedPos = 95;
int stepNumberX = 0;
int stepNumberY = 0;
int incomingByte = 0;
float px, py;

//class that deals with SD card
class SDhandler {
  public:
    File file;

    SDhandler(String filename) {
      //SD card initialization
      Serial.println("Initializing SD card...");
      pinMode(csPin, OUTPUT);
      if (SD.begin()) {
        Serial.println("SD card is ready to use.");
      } else {
        Serial.println("SD card initialization failed");
        return;
      }
      //file opening
      file = SD.open(filename);
      if (file) {
        Serial.println("File opened with success!");
      } else {
        Serial.println("Error opening file...");
      }
    }

    //method that reads next line from file
    String readLine() {
      String received = "";
      char ch;
      while (file.available()) {
        ch = file.read();
        if (ch == '\n') {
          return String(received);
        }
        else {
          received += ch;
        }
      }
      return "";
    }

    void fileClose() {
      file.close();
    }
};

//steppers
//Stepper has to turn 4 times to move across whole axis (120mm) -> 1 step = 0.02mm

void stepX(int dir) {
  if (dir > 0) {
    switch (stepNumberX) {
      case 0:
        digitalWrite(stepperXPin1, HIGH);
        digitalWrite(stepperXPin2, LOW);
        digitalWrite(stepperXPin3, LOW);
        digitalWrite(stepperXPin4, LOW);
        break;
      case 1:
        digitalWrite(stepperXPin1, LOW);
        digitalWrite(stepperXPin2, HIGH);
        digitalWrite(stepperXPin3, LOW);
        digitalWrite(stepperXPin4, LOW);
        break;
      case 2:
        digitalWrite(stepperXPin1, LOW);
        digitalWrite(stepperXPin2, LOW);
        digitalWrite(stepperXPin3, HIGH);
        digitalWrite(stepperXPin4, LOW);
        break;
      case 3:
        digitalWrite(stepperXPin1, LOW);
        digitalWrite(stepperXPin2, LOW);
        digitalWrite(stepperXPin3, LOW);
        digitalWrite(stepperXPin4, HIGH);
        break;
    }
  } else {
    switch (stepNumberX) {
      case 0:
        digitalWrite(stepperXPin1, LOW);
        digitalWrite(stepperXPin2, LOW);
        digitalWrite(stepperXPin3, LOW);
        digitalWrite(stepperXPin4, HIGH);
        break;
      case 1:
        digitalWrite(stepperXPin1, LOW);
        digitalWrite(stepperXPin2, LOW);
        digitalWrite(stepperXPin3, HIGH);
        digitalWrite(stepperXPin4, LOW);
        break;
      case 2:
        digitalWrite(stepperXPin1, LOW);
        digitalWrite(stepperXPin2, HIGH);
        digitalWrite(stepperXPin3, LOW);
        digitalWrite(stepperXPin4, LOW);
        break;
      case 3:
        digitalWrite(stepperXPin1, HIGH);
        digitalWrite(stepperXPin2, LOW);
        digitalWrite(stepperXPin3, LOW);
        digitalWrite(stepperXPin4, LOW);
    }
  }
  stepNumberX++;
  if (stepNumberX > 3) {
    stepNumberX = 0;
  }
}

void stepY(int dir) {
  if (dir > 0) {
    switch (stepNumberY) {
      case 0:
        digitalWrite(stepperYPin1, HIGH);
        digitalWrite(stepperYPin2, LOW);
        digitalWrite(stepperYPin3, LOW);
        digitalWrite(stepperYPin4, LOW);
        break;
      case 1:
        digitalWrite(stepperYPin1, LOW);
        digitalWrite(stepperYPin2, HIGH);
        digitalWrite(stepperYPin3, LOW);
        digitalWrite(stepperYPin4, LOW);
        break;
      case 2:
        digitalWrite(stepperYPin1, LOW);
        digitalWrite(stepperYPin2, LOW);
        digitalWrite(stepperYPin3, HIGH);
        digitalWrite(stepperYPin4, LOW);
        break;
      case 3:
        digitalWrite(stepperYPin1, LOW);
        digitalWrite(stepperYPin2, LOW);
        digitalWrite(stepperYPin3, LOW);
        digitalWrite(stepperYPin4, HIGH);
        break;
    }
  } else {
    switch (stepNumberY) {
      case 0:
        digitalWrite(stepperYPin1, LOW);
        digitalWrite(stepperYPin2, LOW);
        digitalWrite(stepperYPin3, LOW);
        digitalWrite(stepperYPin4, HIGH);
        break;
      case 1:
        digitalWrite(stepperYPin1, LOW);
        digitalWrite(stepperYPin2, LOW);
        digitalWrite(stepperYPin3, HIGH);
        digitalWrite(stepperYPin4, LOW);
        break;
      case 2:
        digitalWrite(stepperYPin1, LOW);
        digitalWrite(stepperYPin2, HIGH);
        digitalWrite(stepperYPin3, LOW);
        digitalWrite(stepperYPin4, LOW);
        break;
      case 3:
        digitalWrite(stepperYPin1, HIGH);
        digitalWrite(stepperYPin2, LOW);
        digitalWrite(stepperYPin3, LOW);
        digitalWrite(stepperYPin4, LOW);
    }
  }
  stepNumberY++;
  if (stepNumberY > 3) {
    stepNumberY = 0;
  }
}

//Bresenham line algorithm
void line(float newx, float newy) {
  long i;
  long over = 0;

  long dx = newx - px;
  long dy = newy - py;

  int dirx, diry;
  if (dx > 0) dirx = 1; else dirx = -1;
  if (dy > 0) dirx = 1; else diry = -1;
  dx = abs(dx);
  dy = abs(dy);
  long dxs = dx / step2mm; //dx in steps
  long dys = dy / step2mm; //dy in steps

  if (dxs > dys) {
    over = dxs / 2;
    for (i = 0; i < dxs; ++i) {
      stepX(dirx);
      over += dys;
      if (over >= dxs) {
        over -= dxs;
        stepY(diry);
      }
      delay(stepDelay);
    }
  } else {
    over = dys / 2;
    for (i = 0; i < dys; ++i) {
      stepY(diry);
      over += dxs;
      if (over >= dys) {
        over -= dys;
        stepX(dirx);
      }
      delay(stepDelay);
    }
  }
  px = newx;
  py = newy;
}

//homeing procedure
void goHome() {
  while (digitalRead(xBorderPin) != LOW) {
    stepX(-1);
    delay(2.0);
  }
  px = 0;
  while (digitalRead(yBorderPin) != LOW) {
    stepY(-1);
    delay(2.0);
  }
  py = 0;
}

//servo
void servoRetract() {
  servo.write(servoRetractedPos);
  delay(2.0);
}

void servoIn() {
  servo.write(servoNotRetractedPos);
  delay(2.0);
}

class commandInterpreter {
  public:

    SDhandler *SDh;
    char command[64];
    float currE = 0;
    float prevE = 0; //extruder previous pos, needed to know if it is needed to retract servo

    commandInterpreter() {
      SDh = new SDhandler("2DP.txt");
    }

    //Method that moves currently interpreted line to next one
    void nextCommand() {
      String commandString = SDh->readLine();
      if (commandString == ";End of Gcode") SDh->fileClose();
      strcpy(command, commandString.c_str());
    }

    //Method that looks for code in currently interpreted line and returns its argument
    float readCode(char code) {
      char *ptr = command;
      while (*ptr != '\0') {
        if (*ptr == code) {
          Serial.println(ptr);
          Serial.println(code);
          Serial.println(atof(ptr + 1));
          return atof(ptr + 1);
        }
        ptr = strchr(ptr, ' ') + 1;
      }
      return -5000;//If code is not found
    }

    void processCommand() {
      nextCommand();
      //In this usage only commands with code G will be usefull so program
      //interprets only them. Other commands can be easly added if its needed
      int codeARG = readCode('G');
      switch (codeARG) {
        case  0:
        case  1: { //draw line
            currE = readCode('E');
            if (prevE > currE) servoRetract(); //If prevE > currE fillament needs to be retracted from hotend so in our case we need to retract servo
            else servoIn();
            prevE = currE;
            line(readCode('X'), readCode('Y'));
          }
          break;
        case 28: { //home
            goHome();
          } break;
        default:  break;
      }
    }
};

commandInterpreter interpreter;
void setup() {
  Serial.begin(baud);
  pinMode(yBorderPin, INPUT_PULLUP);
  pinMode(xBorderPin, INPUT_PULLUP);
  servo.attach(9);
}

void loop() {
  interpreter.processCommand();
}
