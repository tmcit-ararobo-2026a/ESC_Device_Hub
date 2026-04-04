
#pragma once
#include "main.h"

#ifdef STM32H5xx_H
    #include "stm32h5xx_hal_fdcan.h"
#else
    #ifdef STM32H7xx_H
        #include "stm32h7xx_hal_fdcan.h"
    #else
        #include "fdcan.h"
    #endif
#endif
/**
 * fdcan.hが選択されなかったら main_FW.cppもしくは canを使うファイル内に FDCAN_HandleTypeDefを定義して
 */

#include "mFDCAN_data_template.hpp"

class mFDCAN_Class : mFDCAN_template_Class{
    public:
        bool Init(fdcan_setting_HandleTypeDef *set);
        bool Send(fdcan_TxData_HandleTypeDef *data);
        bool Enable_timeout(fdcan_ports port);
        /**
         * return value mean
         * 0 = COMPLETE
         * 1 = ERROR
         */

        void TxCallback(fdcan_CallBack_HandleTypeDef *data);
        void RxCallback_Fifo0(fdcan_CallBack_HandleTypeDef *data);
        void RxCallback_Fifo1(fdcan_CallBack_HandleTypeDef *data);
        
        void Callback_Port1(uint32_t Id, uint8_t *data_p, uint8_t Len);
        void Callback_Port2(uint32_t Id, uint8_t *data_p, uint8_t Len);
        void Callback_Port3(uint32_t Id, uint8_t *data_p, uint8_t Len);

        fdcan_CallBack_HandleTypeDef fdcan_TxCallBack;
        fdcan_CallBack_HandleTypeDef fdcan_RxCallBack_Fifo0;
        fdcan_CallBack_HandleTypeDef fdcan_RxCallBack_Fifo1;
        bool Rx_TimeOut_flag;

        fdcan_State_HandleTypeDef State;
    
};

extern mFDCAN_Class mFDCAN;
