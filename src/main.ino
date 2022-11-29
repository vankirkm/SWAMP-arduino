/* 
*   S.W.A.M.P. Arduino Driver 
*   Michael Van Kirk, Matthew Susser
*
*/ 

#define DEBUG_BAUDRATE 115200
#define TFMINI_BAUDRATE 115200

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

void setup() {

    memset(navGrid, 0, sizeof(navGrid));

    // Initialize serial ports
    Serial.println ("Initializing...");
    Serial.begin(DEBUG_BAUDRATE);
    while (!Serial);
    Serial1.begin(TFMINI_BAUDRATE);
    while(!Serial1);  
}


void loop() {

    // test rotating the lidar sensor 360 degrees and 
    // mapping the location 
    for(int i = 0; i <= 360; i += 2) {
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
    delay(10000);

}

// serialEvent1 is called automatically at the end of every
// main loop iteration. 
void serialEvent1() {
    // getTFminiData(&distance, &strength, &receiveComplete);
}
