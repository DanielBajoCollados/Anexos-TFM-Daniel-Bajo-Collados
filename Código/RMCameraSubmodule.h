#pragma once
#include "rm2_message.h"
#include "RMSubmodulesHandler.h"
#include "Wire.h"

const uint8_t ESP32_CAM_ADRESS = 9;

class RMCameraSubmodule
{

public:

    void send_request_for_position(double* MotorsPos){
        uint8_t buf[sizeof(MotorsPos)+2];
        if (Wire.requestFrom(ESP32_CAM_ADRESS, (uint8_t)(sizeof(MotorsPos)+2))){
            for (uint8_t i = 0; i < sizeof(MotorsPos)+2; i++)
                buf[i]=Wire.read();
            uint16_2byteConversor crc;
            crc.integer = CRC16::crc16(buf, sizeof(MotorsPos));
            if (buf[sizeof(MotorsPos)] == crc.sbytes[0] && buf[sizeof(MotorsPos)+1] == crc.sbytes[1]){  
                memcpy(MotorsPos, buf, sizeof(MotorsPos));
            }
            else
                BT_DEBUG("CAMERA SUBMODULE: ERROR ON CRC");
        }
        else
            BT_DEBUG("CAMERA SUBMODULE: ERROR ON SENDING REQUEST");
    }

    void setup(){
        Wire.begin(21, 23, 400000);
    }
};