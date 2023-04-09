/* 
*   S.W.A.M.P. Arduino Driver 
*   Michael Van Kirk, Matthew Slusser
*
*/

#include <Servo.h>
#include <hp_BH1750.h>

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
bool receiveComplete = false;
byte navGrid[MAP_SIZE][MAP_SIZE];

Servo leg1;
Servo leg2;
Servo leg3;

#define waterPumpPin 22

hp_BH1750 BH1750; // creates sensor object


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

    Serial.print(F("distance: "));
    Serial.println(distance);
    Serial.print(F("x component: "));
    Serial.println(x);
    Serial.print(F("y component: "));
    Serial.println(y);
    
    if((x < 80 && y < 80) && (x >=0 && y >= 0)) {
        navGrid[x][y] = 1;
    }

}

int getWaterLevelStatus(){
    #define liquidLevelPin 24
    int liquidLevelStatus;
    if(digitalRead(liquidLevelPin) == 1){
        liquidLevelStatus = 0;
    }else{
        liquidLevelStatus = 1;
    }
    return liquidLevelStatus;
}
void printWaterLevelStatus(){
    bool waterLevelStatus = getWaterLevelStatus();
    Serial.print(F("waterLevelStatus = "));
    Serial.println(waterLevelStatus, DEC);
}

int getSoilMoistureStatus(){
    #define soilMoistureSensorPin 4
    int soilMoistureStatus = analogRead(soilMoistureSensorPin);
    return soilMoistureStatus;
}
void printSoilMoistureStatus(){
    int soilMoistureStatus = getSoilMoistureStatus();
    Serial.print(F("soilMoistureStatus = "));
    Serial.println(soilMoistureStatus, DEC);
}
int getSoilMoisturePercent(int soilMoistureStatus){
    // The sensor has a range of 280 to 625 in my apartment
    // The sensor has a range of 260 to 570 in the Boffin Factory
    #define dry 625
    #define wet 280
    return map(soilMoistureStatus, wet, dry, 100, 0);
}
void printSoilMoisturePercent(){
    int soilMoistureStatus = getSoilMoistureStatus();
    Serial.print(F("soilMoisturePercent = "));
    Serial.print(getSoilMoisturePercent(soilMoistureStatus), DEC);
    Serial.println(F("%"));
}

bool lightSensorConnected(){
    if(!BH1750.begin(BH1750_TO_GROUND))
    { // init the sensor with address pin connetcted to ground
        Serial.println(F("No BH1750 sensor found!"));
        return false;
    }
    Serial.println(F("BH1750 sensor found!"));
    return true;
}
void lightSensorStartup()
{
    if (!lightSensorConnected())
    {
        while (true)
        {
        };
    }
    if (BH1750.calibrateTiming() < 2)
        Serial.println(F("Calibration OK"));
    else
        Serial.println(F("Calibration FAILED"));
    BH1750.start();
}
int getLuxReading(){
    BH1750.start();              // starts a measurement
    return BH1750.getLux(); //  waits until a conversion finished
}
void printLuxReading(){
    Serial.print(F("Lux = "));
    Serial.println(getLuxReading(), DEC);
}

void readSerialExample(){
    if (Serial.available() > 0){
        String data = Serial.readStringUntil('\n');
        Serial.print("You sent me: ");
        Serial.println(data);
    }
}
void readSerial(){
    if (Serial.available() > 0)
    {
        char c = Serial.read();
        if (c == 'l'){
            getLuxReading();
        }
        else if (c == 'w'){
            printWaterLevelStatus();
        }
        else if (c == 's'){
            printSoilMoistureStatus();
        }
        else if (c == 'm'){
            printSoilMoisturePercent();
        }
        else if (c == 'p'){
            // TODO: turn on water pump here, use function with preset time
        }
    }
}
void piComm(){
    printWaterLevelStatus();
    printSoilMoistureStatus();
    printSoilMoisturePercent();
    printLuxReading();
}

void setup() {
    // Initialize serial ports
    Serial.println (F("Initializing..."));
    Serial.begin(DEBUG_BAUDRATE);

    leg1.attach(2);
    leg2.attach(3);
    leg3.attach(4);

    memset(navGrid, 0, sizeof(navGrid));

    // Sensor Initialization
    pinMode(liquidLevelPin,INPUT_PULLUP);
    lightSensorStartup();

    while (!Serial);
    // TODO - uncomment when ready to integrate lidar sensor
    //Serial1.begin(TFMINI_BAUDRATE);
    //while(!Serial1);

}


void loop() {
    leg1.write(45);
    leg2.write(45);
    leg3.write(45);
    delay(1000);
    leg1.write(90);
    leg2.write(90);
    leg3.write(90);
    delay(1000);

    piComm();
    readSerial();

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
