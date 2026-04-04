
#pragma once

#include <cstdint>
#include "mFDCAN.hpp"

class mmain_FW_Class {
    public:
        void setup();
        void loop();

    /*TypeDef_Start*/
    mFDCAN_template_Class::fdcan_setting_HandleTypeDef fdcan_main_setting;
    mFDCAN_template_Class::fdcan_setting_HandleTypeDef fdcan_esc_setting;
    mFDCAN_template_Class::fdcan_TxData_HandleTypeDef fdcan_txdata;
    /*TypeDef_End*/
};

extern mmain_FW_Class mmain_FW;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
