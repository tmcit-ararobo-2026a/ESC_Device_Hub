
#pragma once
#include "main.h"

#include "stm32h5xx_hal_fdcan.h"
//#include "stm32h7xx_hal_fdcan.h"
//#include "fdcan.h"

#include "mFDCAN_data_template.hpp"

class mFDCAN_Class : mFDCAN_template_Class{
    public:
        bool Init(fdcan_setting_Handle_TypeDef *set);
        bool Send(fdcan_TxData_Handle_TypeDef *data);
        bool Enable_timeout(fdcan_ports port);
        /**
         * return value mean
         * 0 = COMPLETE
         * 1 = ERROR
         */

        void TxCallback(fdcan_CallBack_Handle_TypeDef *data);
        void RxCallback_Fifo0(fdcan_CallBack_Handle_TypeDef *data);
        void RxCallback_Fifo1(fdcan_CallBack_Handle_TypeDef *data);
        
        void Callback_Port1(uint32_t Id, uint8_t *data_p, uint8_t Len);
        void Callback_Port2(uint32_t Id, uint8_t *data_p, uint8_t Len);
        void Callback_Port3(uint32_t Id, uint8_t *data_p, uint8_t Len);

        fdcan_CallBack_Handle_TypeDef fdcan_TxCallBack;
        fdcan_CallBack_Handle_TypeDef fdcan_RxCallBack_Fifo0;
        fdcan_CallBack_Handle_TypeDef fdcan_RxCallBack_Fifo1;
        bool Rx_TimeOut_flag;

        fdcan_State_Handle_TypeDef State;
    
};

extern mFDCAN_Class mFDCAN;
