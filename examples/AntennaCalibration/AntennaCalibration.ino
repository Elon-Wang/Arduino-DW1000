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

struct tagInfo{
    byte short_Addr[2];
    double range_A = 0;
    double range_B = 0;
    double range_C = 0;
    bool received_B = false;
    double x = 0;
    double y = 0;
};
tagInfo tagList[20];//TODO:change this into vector; then don't worry about the number of the tags;
int tagNumCnt=0;

// connection pins
const uint8_t PIN_RST = 9;
const uint8_t PIN_SS = SS; // spi select pin

double range_self;

byte target_eui[8];
uint16_t blink_rate = 300;


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

            RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::ACTIVITY_FINISHED, blink_rate);
            if(!result.success) return;
            tagList[tagNo].range_A = result.range;

            String rangeString = "Tag short Address:  "; rangeString += tagList[tagNo].short_Addr[1];rangeString += tagList[tagNo].short_Addr[0];//todo:maybe need to change the expression to two bytes; THE short_addr is wrong!!!
            rangeString += "\t Range: "; rangeString += tagList[tagNo].range_A ; rangeString += " m";
            Serial.println(rangeString);
        }
    }
}
