
#pragma once

#include <cstdint>
#include "mFDCAN.hpp"

class mmain_FW_Class {
    public:
        void setup();
        void loop();

    /*TypeDef_Start*/
    mFDCAN_template_Class::fdcan_setting_Handle_TypeDef fdcan1_setting;
    mFDCAN_template_Class::fdcan_setting_Handle_TypeDef fdcan2_setting;
    mFDCAN_template_Class::fdcan_TxData_Handle_TypeDef fdcan_txdata;
    /*TypeDef_End*/
};

extern mmain_FW_Class mmain_FW;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
