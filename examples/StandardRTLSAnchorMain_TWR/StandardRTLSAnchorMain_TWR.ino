/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * StandardRTLSAnchorMain_TWR.ino
 *
 * This is an example master anchor in a RTLS using two way ranging ISO/IEC 24730-62_2013 messages
 */

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgRTLS.hpp>

typedef struct Position {
    double x;
    double y;
} Position;

struct tagInfo{
    byte short_Addr[2];
    double range_A = 0;
    double range_B = 0;
    double range_C = 0;
    bool received_B = false;
    double x = 0;
    double y = 0;

    void transimitPosReport() {
        byte positionReport[] ={DATA,SHORT_SRC_AND_DEST, DW1000NgRTLS::increaseSequenceNumber(), 0,0, 0,0, 0,0, 0x33 , 0,0, 0,0, 0,0, 0,0, 0,0};
        DW1000Ng::getNetworkId(&positionReport[3]);
        memcpy(&positionReport[5], short_Addr , 2);
        DW1000Ng::getDeviceAddress(&positionReport[7]);
        DW1000NgUtils::writeValueToBytes(&positionReport[10], static_cast<uint16_t>((range_A*1000)), 2);
        DW1000NgUtils::writeValueToBytes(&positionReport[12], static_cast<uint16_t>((range_B*1000)), 2);
        DW1000NgUtils::writeValueToBytes(&positionReport[14], static_cast<uint16_t>((range_C*1000)), 2);
        DW1000NgUtils::writeValueToBytes(&positionReport[16], static_cast<uint16_t>((x*1000)), 2);
        DW1000NgUtils::writeValueToBytes(&positionReport[18], static_cast<uint16_t>((y*1000)), 2);
        DW1000Ng::setTransmitData(positionReport, sizeof(positionReport));
        DW1000Ng::startTransmit();
    }
};
tagInfo tagList[20];//TODO:change this into vector; then don't worry about the number of the tags;
int tagNumCnt=0;

// connection pins
const uint8_t PIN_RST = 9;
const uint8_t PIN_SS = SS; // spi select pin

Position position_self = {0,0};
Position position_B = {3,0};
Position position_C = {3,2.5};

double range_self;
double range_B;
double range_C;

boolean received_B = false;

byte target_eui[8];
byte tag_shortAddress[] = {0x05, 0x00};

byte anchor_b[] = {0x02, 0x00};
uint16_t next_anchor = 2;
byte anchor_c[] = {0x03, 0x00};

device_configuration_t DEFAULT_CONFIG = {
        false,
        true,
        true,
        true,
        false,
        SFDMode::STANDARD_SFD,
        Channel::CHANNEL_5,
        DataRate::RATE_850KBPS,
        PulseFrequency::FREQ_16MHZ,
        PreambleLength::LEN_256,
        PreambleCode::CODE_3
};

frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
        false,
        false,
        true,
        false,
        false,
        false,
        false,
        true /* This allows blink frames */
};

long long  debugMillis;

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    Serial.println(F("### DW1000Ng-arduino-ranging-anchorMain ###"));
    // initialize the driver
#if defined(ESP8266)
    DW1000Ng::initializeNoInterrupt(PIN_SS);
#else
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
#endif
    Serial.println(F("DW1000Ng initialized ..."));
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);

    DW1000Ng::setEUI("AA:BB:CC:DD:EE:FF:00:01");

    DW1000Ng::setPreambleDetectionTimeout(64);
    DW1000Ng::setSfdDetectionTimeout(273);
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(5000);

    DW1000Ng::setNetworkId(RTLS_APP_ID);
    DW1000Ng::setDeviceAddress(1);

    DW1000Ng::setAntennaDelay(16436);

    Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
    tagNumCnt = 0;

    debugMillis= millis();
}

/* using https://math.stackexchange.com/questions/884807/find-x-location-using-3-known-x-y-location-using-trilateration */
void calculatePosition(double &x, double &y) {//todo: changing this 2D algorithm into 3D algorithm.

    /* This gives for granted that the z plane is the same for anchor and tags */
    double A = ( (-2*position_self.x) + (2*position_B.x) );
    double B = ( (-2*position_self.y) + (2*position_B.y) );
    double C = (range_self*range_self) - (range_B*range_B) - (position_self.x*position_self.x) + (position_B.x*position_B.x) - (position_self.y*position_self.y) + (position_B.y*position_B.y);
    double D = ( (-2*position_B.x) + (2*position_C.x) );
    double E = ( (-2*position_B.y) + (2*position_C.y) );
    double F = (range_B*range_B) - (range_C*range_C) - (position_B.x*position_B.x) + (position_C.x*position_C.x) - (position_B.y*position_B.y) + (position_C.y*position_C.y);

    x = (C*E-F*B) / (E*A-B*D);
    y = (C*D-A*F) / (B*D-A*E);
}

int getTagNo(byte tag_Short_Addr[]){
    for (int i = 0; i <tagNumCnt; ++i) {
        if(tagList[i].short_Addr[0]==tag_Short_Addr[0] && tagList[i].short_Addr[0]==tag_Short_Addr[0])
            return i;
    }
    //If it's not in the list, creat a new one and assign the information;
    memcpy(tagList[tagNumCnt].short_Addr,tag_Short_Addr,2);
    tagNumCnt++;
    return tagNumCnt-1;
}

void loop() {
    /*debug code, used to print out the tagInfo;
    while(millis() - debugMillis > 100){
        for (int i = 0; i <tagNumCnt ; ++i) {
            String tagInfoString = "TagNo:"; tagInfoString += i;
            tagInfoString += "\t byte[0]:"; tagInfoString +=tagList[i].short_Addr[0] ;
            tagInfoString += "\t byte[1]:"; tagInfoString += tagList[i].short_Addr[1] ;
            tagInfoString += "\n range_A: "; tagInfoString += tagList[i].range_A ; tagInfoString += " m";
            tagInfoString += "\n range_B: "; tagInfoString += tagList[i].range_B ; tagInfoString += " m";
            tagInfoString += "\n range_c: "; tagInfoString += tagList[i].range_C ; tagInfoString += " m";
            tagInfoString += "\n received_B: "; tagInfoString += tagList[i].received_B;
            tagInfoString += "\n (x,y)= ("; tagInfoString += tagList[i].x;tagInfoString += ",";tagInfoString += tagList[i].y;tagInfoString +=")";
            Serial.println(tagInfoString);
            Serial.println( tagList[i].short_Addr[0],BIN);
            Serial.println( tagList[i].short_Addr[1],BIN);
        }
        debugMillis = millis();
    }*/

    byte short_Addr[2];
    double range_A = 0;
    double range_B = 0;
    double range_C = 0;
    bool received_B = false;
    double x = 0;
    double y = 0;

    if(DW1000NgRTLS::receiveFrame()){
        size_t recv_len = DW1000Ng::getReceivedDataLength();
        byte recv_data[recv_len];
        DW1000Ng::getReceivedData(recv_data, recv_len);

        if(recv_data[0] == BLINK) {
            int tagNo = getTagNo(&recv_data[2]);
            DW1000NgRTLS::transmitRangingInitiation(&recv_data[2], tagList[tagNo].short_Addr);
            DW1000NgRTLS::waitForTransmission();

            RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::RANGING_CONFIRM, next_anchor);
            if(!result.success) return;
            tagList[tagNo].range_A = result.range;

            String rangeString = "Tag short Address:  "; rangeString += tagList[tagNo].short_Addr[1];rangeString += tagList[tagNo].short_Addr[0];//todo:maybe need to change the expression to two bytes; THE short_addr is wrong!!!
            rangeString += "\t Range: "; rangeString += tagList[tagNo].range_A ; rangeString += " m";
            rangeString += "\t RX power: "; rangeString += DW1000Ng::getReceivePower(); rangeString += " dBm";
            Serial.println(rangeString);

        } else if(recv_data[9] == 0x60) {
            int tagNo = getTagNo(&recv_data[12]);
            double range = static_cast<double>(DW1000NgUtils::bytesAsValue(&recv_data[10],2) / 1000.0);
            String rangeReportString ="Tag short Address:  "; rangeReportString += tagList[tagNo].short_Addr[0];
            rangeReportString += "\t Range from: "; rangeReportString += recv_data[7];
            rangeReportString += " = "; rangeReportString += range;
            Serial.println(rangeReportString);
            if(tagList[tagNo].received_B == false && recv_data[7] == anchor_b[0] && recv_data[8] == anchor_b[1]) {
                tagList[tagNo].range_B = range;
                tagList[tagNo].received_B = true;
            } else if(tagList[tagNo].received_B == true && recv_data[7] == anchor_c[0] && recv_data[8] == anchor_c[1]){
                tagList[tagNo].range_C = range;
                calculatePosition(tagList[tagNo].x,tagList[tagNo].y);
                String positioning = "Found position - x: ";
                positioning += tagList[tagNo].x; positioning +=" y: ";
                positioning += tagList[tagNo].y;
                Serial.println(positioning);
                tagList[tagNo].received_B = false;
            } else {
                tagList[tagNo].received_B = false;
            }
        }
    }
}