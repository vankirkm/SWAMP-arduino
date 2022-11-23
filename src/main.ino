/* 
*   S.W.A.M.P. Arduino Driver 
*   Michael Van Kirk, Matthew Susser
*
*/ 

#define DEBUG_BAUDRATE 115200
#define TFMINI_BAUDRATE 115200

int distance = 0;
int strength = 0;
boolean receiveComplete = false;

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

void setup() {

    // Initialize serial ports
    Serial.println ("Initializing...");
    Serial.begin(DEBUG_BAUDRATE);
    while (!Serial);
    Serial1.begin(TFMINI_BAUDRATE);
    while(!Serial1);  
}


void loop() {
    
    if(receiveComplete) {
        receiveComplete = false;
        Serial.print(distance);
        Serial.print("cm\t");
        Serial.print("strength: ");
        Serial.print(strength);
        Serial.print("\n");
    }
}

// serialEvent1 is called automatically at the end of every
// main loop iteration. 
void serialEvent1() {
    getTFminiData(&distance, &strength, &receiveComplete);
}
