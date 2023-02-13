#include <Arduino.h>
#include <SPI.h>
#include "RF24.h"
#include <ESP32Servo.h> 

#define MAX_STEER 160
#define MIN_STEER 20

#define PACKET_SIZE 4
#define MIN_UPDATE_RATE 10 //1000ms / MIN_UPDATE_RATE 
#define MILLI_TO_EXPECT_UPDATE 1000 / MIN_UPDATE_RATE //Stop all motors if no update recieved in this amount of time

// Possible PWM GPIO pins on the ESP32: 0(used by on-board button),2,4,5(used by on-board LED),12-19,21-23,25-27,32-33 
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42 
//TODO: UPDATE THESE
#define STEER_PIN 13
#define ESC_PIN 14
#define PAN_PIN 15
#define TILT_PIN 16

RF24 radio(16, 5);  // using pin 16 for the CE pin, and pin 5 for the CSN pin
// Let these addresses be used for the pair
uint8_t address[][6] = { "1Node", "2Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving
bool role = false;  // true = TX role, false = RX role

unsigned long MaxNextEventTime = millis(); //Stop motor if update not recieved by this time

//data structure of device
struct State {
    int steer;
    int esc;
    int pan;
    int tilt;
    bool auxButton[8];
};

Servo steer;
Servo esc;
Servo pan;
Servo tilt;

void setup() {
    Serial.begin(115200);
    Serial.println("Setting Up...");

    if (!radio.begin()) {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {}  // hold in infinite loop
    }
    radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
    radio.setPayloadSize(PACKET_SIZE);
    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1

    if (role) {
        radio.stopListening();  // put radio in TX mode
    } else {
        radio.startListening();  // put radio in RX mode
    }

    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

    steer.setPeriodHertz(50);// Standard 50hz servo
    esc.setPeriodHertz(50);// Standard 50hz servo
    pan.setPeriodHertz(50);
    tilt.setPeriodHertz(50);

    steer.attach(STEER_PIN, 1000, 2000); //Min: 600 Max: 2000 for pan/tilt servo
    esc.attach(ESC_PIN, 600, 2400);
    //pan.attach(PAN_PIN, 600,2000);
    //tilt.attach(TILT_PIN, 600,2000);

    steer.write(90);
    //pan.write(90);
    //tilt.write(90);

    Serial.println("FULL");
    esc.write(180);
    delay(5000);
    Serial.println("NONE");
    esc.write(0);
    delay(5000);
    Serial.println("CENTER");
    esc.write(90);
    delay(5000);
    Serial.println("Finished Setup");
}

void describeState(State state){
    Serial.println("******************************");
    Serial.print("Steer: ");
    Serial.println(state.steer);
    Serial.print("Esc: ");
    Serial.println(state.esc);
    Serial.print("Pan: ");
    Serial.println(state.pan);
    Serial.print("Tilt: ");
    Serial.println(state.tilt);

    for(int i=0; i<8; i++){
        Serial.print("Button ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(state.auxButton[i]);
    }
    Serial.println("******************************");
}

State readState(){
    uint8_t bytes = radio.getPayloadSize();
    uint8_t buffer[PACKET_SIZE];
    radio.read(buffer, bytes);

    /*Serial.print("RX Data: ");
    for(int i=0; i<bytes; i++){
        Serial.print(buffer[i], DEC);
    }
    Serial.println("");*/

    State newState;
    newState.steer = buffer[0];
    newState.esc = buffer[1];

    //newState.tilt = map(buffer[2] & 15, 0, 127, 0, 180);//Get bottom 4 bits
    //newState.pan = map(buffer[2] >> 4, 0, 127, 0, 180);//Shift top 4 bits right and take bottom value

    for(int i=0; i<8;i++){
        int mask = pow(2,i);
        if(buffer[3] & mask){
            newState.auxButton[i] = true;
        }
    }
    //describeState(newState);
    return newState;
}

void applyState(State state){
    int steerValue;
    if(state.steer > MAX_STEER){
        steerValue = MAX_STEER;
    }else if(state.steer < MIN_STEER){
        steerValue = MIN_STEER;
    }else{
        steerValue = state.steer;
    }

    Serial.print("Steer: ");
    Serial.println(steerValue);
    steer.write(steerValue);
    esc.write(state.esc);
}

void loop() {
    unsigned long eventTime = millis();
    uint8_t pipe;
    if(radio.available(&pipe)){
        State newState = readState(); 
        MaxNextEventTime = millis() + MILLI_TO_EXPECT_UPDATE;
        applyState(newState);
    }
    if(eventTime > MaxNextEventTime){
        esc.write(90);
    }
}