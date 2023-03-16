/* 
*   S.W.A.M.P. Arduino Driver 
*   Michael Van Kirk, Matthew Slusser
*
*/

// Notes
//
// Servo Movement Directions
// -------------------------
// The to move the left legs in a forward direction, the hip angle
// need to Increase from 60 degrees. The opposite is true for the left


#include <Servo.h>

#define DEBUG_BAUDRATE 115200
#define TFMINI_BAUDRATE 115200

// LegGroup - A struct that defines the interface used to 
// define a leg in software. Each leg consists of 3 servos
// that each serve as a joint
struct LegGroup {
    Servo ankle;
    Servo knee;
    Servo hip;
};

// distance measurements are currently taken in centimeters.
// since swamp is placed at the center of the navgrid, the formula
// for grid dimension is (maxRange * 2) / scale where maxRange = max range of
// lidar sensor and scale = the size in physical space the each grid
// cell represents (cm)
// However, the arduino mega won't allow us to put more that 8k bytes
// into memory so for now we have to go with an 80x80 grid
#define MAP_SIZE 80
#define MIDDLE_POSITION 40

int distance = 0;
int strength = 0;
boolean receiveComplete = false;
byte navGrid[MAP_SIZE][MAP_SIZE];
LegGroup leftFront;
LegGroup leftMid;
LegGroup leftRear;
LegGroup rightFront;
LegGroup rightMid;
LegGroup rightRear;

//----------------------------------------------------------------
// getTFMiniData: read lidar data from the tfmini. loops until it
//                has read and verified a complete message, then
//                sets receiveComplete to true to notify the outer 
//                loop to exit
//     Parameters:
//         distance (int*): pointer to distance variable
//         strength (int*): pointer to strength variable
//         complete (boolean*): pointer complete variable
//     Returns: 
//         void
//----------------------------------------------------------------
void getTFminiData(int* distance, int* strength, boolean* complete) {
    static char i = 0;
    char j = 0;
    int checksum = 0;
    static int rx[9];

    if(Serial1.available()) {

        // read the first byte of data from the serial port
        rx[i] = Serial1.read();

        // the first two bytes of every message from the tfmini
        // are the header byte, which is 0x59. So to ensure that
        // we have a properly formatted message, we toss out all
        // bytes read until we read two 0x59s in a row. Then we
        // read 9 bytes into rx, verify the checksum, and print
        // the message to the debug serial port
        if(rx[0] != 0x59) {
            i = 0;
        } else if(i == 1 && rx[i] != 0x59) {
            i = 0;
        } else if(i == 8) {

            // the checksum of every message sent is the lower 8 bits
            // of the cumulative sum of the first 8 bytes 
            for(j = 0; j < 8; j++) {
                checksum += rx[j];
            }
            if(rx[8] == (checksum & 0x00FF)) {
                *distance = (int)((rx[3] << 8) | rx[2]);
                *strength = (int)((rx[5] << 8) | rx[4]);
                *complete = true;
            }
            i = 0;
        } else {
            i++;
        }
    }
}

void getGridObstacle(const int degRotation ) {

    while(!receiveComplete) {
        getTFminiData(&distance, &strength, &receiveComplete);
    }

    double radians = radians(degRotation);
    double cosine = cos(radians);
    double sine = sin(radians);

    int x = cosine * distance + MIDDLE_POSITION;
    int y = sine * distance + MIDDLE_POSITION;

    Serial.print("distance: ");
    Serial.println(distance);
    Serial.print("x component: ");
    Serial.println(x);
    Serial.print("y component: ");
    Serial.println(y);
    
    if((x < 80 && y < 80) && (x >=0 && y >= 0)) {
        navGrid[x][y] = 1;
    }

}

void forwardStep() {
    int midPosition = 60;
    int kneeMidPoint = 60;

    for(int i = 1; i <= 20; i++) {
        // moving set
        leftFront.hip.write(midPosition + i);
        leftRear.hip.write(midPosition + i);
        rightMid.hip.write(midPosition - i);

        // static set
        rightFront.hip.write(midPosition + i);
        rightRear.hip.write(midPosition + i);
        leftMid.hip.write(midPosition - i);

        if(i <= 10) {
            kneeMidPoint-=5;
            leftFront.knee.write(kneeMidPoint);
            leftRear.knee.write(kneeMidPoint);
            rightMid.knee.write(kneeMidPoint);
        } else {
            kneeMidPoint+=5;
            leftFront.knee.write(kneeMidPoint);
            leftRear.knee.write(kneeMidPoint);
            rightMid.knee.write(kneeMidPoint);
        }
        delay(20);
    }

    for(int i = 1; i <= 20; i++) {
        // moving set
        rightFront.hip.write(midPosition - i);
        rightRear.hip.write(midPosition - i);
        leftMid.hip.write(midPosition + i);

        // static set
        leftFront.hip.write(midPosition - i);
        leftRear.hip.write(midPosition - i);
        rightMid.hip.write(midPosition + i);

        if(i <= 10) {
            kneeMidPoint-=5;
            rightFront.knee.write(kneeMidPoint);
            rightRear.knee.write(kneeMidPoint);
            leftMid.knee.write(kneeMidPoint);
        } else {
            kneeMidPoint+=5;
            rightFront.knee.write(kneeMidPoint);
            rightRear.knee.write(kneeMidPoint);
            leftMid.knee.write(kneeMidPoint);
        }
        delay(20);
    }
}

void centipedeStep() {
    int leftHipPosition = 60;
    int rightHipPosition = 60;
    int kneePosition = 60;

    LegGroup leftLegs[] = {leftFront, leftMid, leftRear};
    LegGroup rightLegs[] = {rightFront, rightMid, rightRear};

    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 60; j++) {
            if(j % 3 == 0) {
                leftHipPosition++;
                leftLegs[i].hip.write(leftHipPosition);
            }
            if(j <= 30) {
                kneePosition++;
                leftLegs[i].knee.write(kneePosition);
            } else {
                kneePosition--;
                leftLegs[i].knee.write(kneePosition);
                leftLegs[i].ankle.write(kneePosition);
            }
            delay(5);
            kneePosition = 60;
            if(j % 3 == 0) {
                rightHipPosition--;
                rightLegs[i].hip.write(rightHipPosition);
            }
            if(j <= 30) {
                kneePosition++;
                rightLegs[i].knee.write(kneePosition);
            } else {
                kneePosition--;
                rightLegs[i].knee.write(kneePosition);
                rightLegs[i].ankle.write(kneePosition);
            }
        }
    }

    for(int i = 0; i < 30; i++) {
        for(int j = 0; j < 3; j++) {
            rightHipPosition++;
            leftHipPosition--;
            leftLegs[j].hip.write(leftHipPosition);
            leftLegs[j].hip.write(rightHipPosition);
        }
        delay(5);
    }

}

void setup() {

    leftFront.hip.attach(48);
    leftFront.knee.attach(49);
    leftFront.ankle.attach(2);
    leftMid.hip.attach(6);
    leftMid.knee.attach(7);
    leftMid.ankle.attach(8);
    leftRear.hip.attach(12);
    leftRear.knee.attach(13);
    leftRear.ankle.attach(44);
    rightFront.hip.attach(3);
    rightFront.knee.attach(4);
    rightFront.ankle.attach(5);
    rightMid.hip.attach(9);
    rightMid.knee.attach(10);
    rightMid.ankle.attach(11);
    rightRear.hip.attach(45);
    rightRear.knee.attach(46);
    rightRear.ankle.attach(47);

    // Initialize servo positions

    leftFront.hip.write(60);
    leftFront.knee.write(60);
    leftFront.ankle.write(60);

    leftMid.hip.write(60);
    leftMid.knee.write(60);
    leftMid.ankle.write(60);

    leftRear.hip.write(60);
    leftRear.knee.write(60);
    leftRear.ankle.write(60);

    rightFront.hip.write(60);
    rightFront.knee.write(60);
    rightFront.ankle.write(60);

    rightMid.hip.write(60);
    rightMid.knee.write(60);
    rightMid.ankle.write(60);

    rightRear.hip.write(60);
    rightRear.knee.write(60);
    rightRear.ankle.write(60);

    delay(2000);

    memset(navGrid, 0, sizeof(navGrid));

    // Initialize serial ports
    Serial.println ("Initializing...");
    Serial.begin(DEBUG_BAUDRATE);
    while (!Serial);

    // TODO - uncomment when ready to integrate lidar sensor
    //Serial1.begin(TFMINI_BAUDRATE);
    //while(!Serial1);
}


void loop() {

    forwardStep();
    delay(20);


    // TODO - uncomment this code when ready to integrate lidar sensor
    // test rotating the lidar sensor 360 degrees and 
    // mapping the location 
    /*for(int i = 0; i <= 360; i += 2) {
        getGridObstacle(i);
        receiveComplete = false;
    }

    // print the navGrid to the serial port
    for(int i = 0; i < MAP_SIZE; i++) {

        for(int j = 0; j < MAP_SIZE; j++) {
            Serial.print(navGrid[i][j]);
            Serial.print(" ");
        }
        Serial.println();
    }

    // receiveComplete = false;
    memset(navGrid, 0, sizeof(navGrid));
    //delay(10000);*/

}
