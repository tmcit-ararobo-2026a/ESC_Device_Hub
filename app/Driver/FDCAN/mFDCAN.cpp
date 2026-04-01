
#include "mFDCAN.hpp"
#include "mFDCAN_data_template.hpp"

bool mFDCAN_Class::Init(fdcan_setting_Handle_TypeDef *set)
{
    FDCAN_FilterTypeDef FDCAN_filter;
    bool using_error;
    
    HAL_FDCAN_Stop(set->hfdcanx);/*設定中はfdcanを止める*/
    HAL_FDCAN_DisableTimeoutCounter(set->hfdcanx);/*timeoutも止める*/

    FDCAN_filter.IdType = FDCAN_STANDARD_ID;
    FDCAN_filter.FilterIndex = 0;
    FDCAN_filter.FilterType = FDCAN_FILTER_MASK;

    if(set->fifo_num == Fifo_num_type::FIFO0)
    {
        FDCAN_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    }
    else if(set->fifo_num == Fifo_num_type::FIFO1)
    {
        FDCAN_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    }
    else{}

    /*使用するfifoを指定する*/

    FDCAN_filter.FilterID1 = 0;
    FDCAN_filter.FilterID2 = 0;

    /*----------------------------------------------------------------------------------------------------*/

    if(HAL_FDCAN_ConfigFilter(set->hfdcanx, &FDCAN_filter) != HAL_OK)
    {
        State.Init.Config = 1;
        using_error = 1;
    }

    /*----------------------------------------------------------------------------------------------------*/

    if(set->fifo_num == Fifo_num_type::FIFO0)
    {
        if(HAL_FDCAN_ActivateNotification(set->hfdcanx, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL, 0) != HAL_OK){
            State.Init.RxCallBack_Fifo0 = 1;
            using_error = 1;
        }
        
        if(HAL_FDCAN_DeactivateNotification(set->hfdcanx, FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_FULL) != HAL_OK){
            State.Init.RxCallBack_Fifo1 = 1;
            using_error = 1;
        }
    }else if(set->fifo_num == Fifo_num_type::FIFO1)
    {
        if(HAL_FDCAN_DeactivateNotification(set->hfdcanx, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL) != HAL_OK){
            State.Init.RxCallBack_Fifo0 = 1;
            using_error = 1;
        }

        if(HAL_FDCAN_ActivateNotification(set->hfdcanx, FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_FULL, 0) != HAL_OK){
            State.Init.RxCallBack_Fifo1 = 1;
            using_error = 1;
        }
    }
    else
    
    /*----------------------------------------------------------------------------------------------------*/

    if(HAL_FDCAN_ActivateNotification(set->hfdcanx, FDCAN_IT_TX_COMPLETE, 0) != HAL_OK){
        State.Init.TxCallBack = 1;
        using_error = 1;
    }

    /*----------------------------------------------------------------------------------------------------*/

    if(set->RxTimeOutCycle_ms != 0)
    {
        uint32_t bit_count;
        uint32_t bit_rate;
        //HAL_FDCAN_EnableTimeoutCounter(set->hfdcanx);

        switch(set->bit_rate){

            case bit_rate_type::_1Mbps_:
                bit_rate = 1000U;
            break;

            case bit_rate_type::_2Mbps_:
                bit_rate = 2000U;
            break;

            case bit_rate_type::_3Mbps_:
                bit_rate = 3000U;
            break;

            case bit_rate_type::_4Mbps_:
                bit_rate = 4000U;
            break;

            case bit_rate_type::_5Mbps_:
                bit_rate = 5000U;
            break;
        }

        bit_count = bit_rate * set->RxTimeOutCycle_ms;

        if(set->fifo_num == Fifo_num_type::FIFO0)
        {
            if(HAL_FDCAN_ConfigTimeoutCounter(set->hfdcanx, FDCAN_TIMEOUT_RX_FIFO0, bit_count) != HAL_OK){

            }
        }else if(set->fifo_num == Fifo_num_type::FIFO1)
        {
            if(HAL_FDCAN_ConfigTimeoutCounter(set->hfdcanx, FDCAN_TIMEOUT_RX_FIFO1, bit_count) != HAL_OK){

            }
        }
        else{}

        if(HAL_FDCAN_ActivateNotification(set->hfdcanx, FDCAN_IT_TIMEOUT_OCCURRED, 0) != HAL_OK)
        {
            State.Init.TimeOut_CallBack = 1;
            using_error = 1;
        }
    }
    else
    {
        HAL_FDCAN_DisableTimeoutCounter(set->hfdcanx);
        
        if(HAL_FDCAN_DeactivateNotification(set->hfdcanx, FDCAN_IT_TIMEOUT_OCCURRED) != HAL_OK)
        {
            State.Init.TimeOut_CallBack = 1;
            using_error = 1;
        }
    }
    
    /*----------------------------------------------------------------------------------------------------*/

    switch(set->hfdcan_port)
    {
        case fdcan_ports::FDCAN1_Port:
            FDCAN_Port1_set.hfdcanx = set->hfdcanx;
        break;
        
        case fdcan_ports::FDCAN2_Port:
            FDCAN_Port2_set.hfdcanx = set->hfdcanx;
        break;
        
        case fdcan_ports::FDCAN3_Port:
            FDCAN_Port3_set.hfdcanx = set->hfdcanx;
        break;
    }

    /*----------------------------------------------------------------------------------------------------*/

    if(HAL_FDCAN_Start(set->hfdcanx) != HAL_OK)
    {
        State.Init.Start = 1;
        using_error = 1;
    }

    /*----------------------------------------------------------------------------------------------------*/

    if(using_error) return 1;

    return 0;
}


/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/


bool mFDCAN_Class::Send(fdcan_TxData_Handle_TypeDef *data)
{
    FDCAN_TxHeaderTypeDef FDCAN_TxHeader;
    FDCAN_HandleTypeDef *hfdcanx;
    can_frame_type hfdcan_frame;

    switch(data->FDCAN_Port)
    {
        case fdcan_ports::FDCAN1_Port:
            hfdcanx = FDCAN_Port1_set.hfdcanx;
            hfdcan_frame = FDCAN_Port1_set.hfdcan_frame;
            if(FDCAN_Port1_Stack.tx_events > 15)
            {
                State.Send.User_TxFifo_full_Port = 1;
                return 1;
            }
        break;
        
        case fdcan_ports::FDCAN2_Port:
            hfdcanx = FDCAN_Port2_set.hfdcanx;
            hfdcan_frame = FDCAN_Port2_set.hfdcan_frame;
            if(FDCAN_Port2_Stack.tx_events > 15)
            {
                State.Send.User_TxFifo_full_Port = 1;
                return 1;
            }
        break;
        
        case fdcan_ports::FDCAN3_Port:
            hfdcanx = FDCAN_Port3_set.hfdcanx;
            hfdcan_frame = FDCAN_Port3_set.hfdcan_frame;
            if(FDCAN_Port3_Stack.tx_events > 15)
            {
                State.Send.User_TxFifo_full_Port = 1;
                return 1;
            }
        break;
    }

    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcanx) == 0)
    {
        State.Send.TxFifo_full_Port = 1;
        return 1;
    }

    if(hfdcan_frame == can_frame_type::classic_can)
    {
        if(data->Len > 8) data->Len = 8;
        if(data->Id > 0x7FF)
        {
            State.Send.over_Id_value = 1;
            return 0;
        }
    }
    else if(hfdcan_frame == can_frame_type::fdcan)
    {
        if(data->Len > 64) data->Len = 64;
        if(data->Id > 0x1FFFFFFF)
        {
            State.Send.over_Id_value = 1;
            return 0;
        }
    }

    FDCAN_TxHeader.Identifier = data->Id;
    FDCAN_TxHeader.IdType = FDCAN_STANDARD_ID;
    FDCAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    FDCAN_TxHeader.DataLength = Data_len(data->Len);
    FDCAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    FDCAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    FDCAN_TxHeader.FDFormat = FDCAN_FD_CAN;
    FDCAN_TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    FDCAN_TxHeader.MessageMarker = 0;

    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcanx, &FDCAN_TxHeader, data->data_p) != HAL_OK)
    {
        State.Send.Add_New_TxMessage = 1;
        return 1;
    }

    switch(data->FDCAN_Port)
    {
        case fdcan_ports::FDCAN1_Port:
            ++FDCAN_Port1_Stack.tx_events;
        break;
        
        case fdcan_ports::FDCAN2_Port:
            ++FDCAN_Port2_Stack.tx_events;
        break;
        
        case fdcan_ports::FDCAN3_Port:
            ++FDCAN_Port3_Stack.tx_events;
        break;
    }

    return 0;
}


/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/


bool mFDCAN_Class::Enable_timeout(fdcan_ports port)
{
    if(port == fdcan_ports::FDCAN1_Port)
    {
        if(HAL_FDCAN_EnableTimeoutCounter(FDCAN_Port1_set.hfdcanx) != HAL_OK)
        {
            return 1;
        }
    }
    else if(port == fdcan_ports::FDCAN2_Port)
    {
        if(HAL_FDCAN_EnableTimeoutCounter(FDCAN_Port2_set.hfdcanx) != HAL_OK)
        {
            return 1;
        }
    }
    else if(port == fdcan_ports::FDCAN3_Port)
    {
        if(HAL_FDCAN_EnableTimeoutCounter(FDCAN_Port3_set.hfdcanx) != HAL_OK)
        {
            return 1;
        }
    }

    return 0;
}


/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/


void mFDCAN_Class::TxCallback(fdcan_CallBack_Handle_TypeDef *data)
{
    if(data->State == FDCAN_IT_TX_COMPLETE)
    {
        if(data->hfdcanx == FDCAN_Port1_set.hfdcanx)
        {
            --FDCAN_Port1_Stack.tx_events;
        }
        else if(data->hfdcanx == FDCAN_Port2_set.hfdcanx)
        {
            --FDCAN_Port2_Stack.tx_events;
        }
        else if(data->hfdcanx == FDCAN_Port3_set.hfdcanx)
        {
            --FDCAN_Port3_Stack.tx_events;
        }
    }
}


/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/


void mFDCAN_Class::RxCallback_Fifo0(fdcan_CallBack_Handle_TypeDef *data)
{

    FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
    uint8_t* data_p = NULL;
    if(data->hfdcanx == FDCAN_Port1_set.hfdcanx)
    {
        if(data->State == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
        {
            if(HAL_FDCAN_GetRxMessage(data->hfdcanx, FDCAN_RX_FIFO0, &FDCAN_RxHeader, data_p) != HAL_OK)
            {

            }
        }

        if(data->State == FDCAN_IT_RX_FIFO0_FULL)
        {

        }
        Callback_Port1(FDCAN_RxHeader.Identifier, data_p, FDCAN_RxHeader.DataLength);
        return;
    }
    
    /*----------------------------------------------------------------------------------------------------*/

    else if(data->hfdcanx == FDCAN_Port2_set.hfdcanx)
    {
        if(data->State == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
        {
            if(HAL_FDCAN_GetRxMessage(data->hfdcanx, FDCAN_RX_FIFO0, &FDCAN_RxHeader, data_p) != HAL_OK)
            {

            }
        }

        if(data->State == FDCAN_IT_RX_FIFO0_FULL)
        {

        }
        Callback_Port2(FDCAN_RxHeader.Identifier, data_p, FDCAN_RxHeader.DataLength);
        return;
    }
    
    /*----------------------------------------------------------------------------------------------------*/
    
    else if(data->hfdcanx == FDCAN_Port3_set.hfdcanx)
    {
        if(data->State == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
        {
            if(HAL_FDCAN_GetRxMessage(data->hfdcanx, FDCAN_RX_FIFO0, &FDCAN_RxHeader, data_p) != HAL_OK)
            {

            }
        }

        if(data->State == FDCAN_IT_RX_FIFO0_FULL)
        {

        }
        Callback_Port3(FDCAN_RxHeader.Identifier, data_p, FDCAN_RxHeader.DataLength);
        return;
    }

}


/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/


void mFDCAN_Class::RxCallback_Fifo1(fdcan_CallBack_Handle_TypeDef *data)
{

    FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
    uint8_t* data_p = NULL;
    if(data->hfdcanx == FDCAN_Port1_set.hfdcanx)
    {
        if(data->State == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
        {
            if(HAL_FDCAN_GetRxMessage(data->hfdcanx, FDCAN_RX_FIFO0, &FDCAN_RxHeader, data_p) != HAL_OK)
            {

            }
        }

        if(data->State == FDCAN_IT_RX_FIFO0_FULL)
        {

        }
        Callback_Port1(FDCAN_RxHeader.Identifier, data_p, FDCAN_RxHeader.DataLength);
        return;
    }
    
    /*----------------------------------------------------------------------------------------------------*/

    else if(data->hfdcanx == FDCAN_Port2_set.hfdcanx)
    {
        if(data->State == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
        {
            if(HAL_FDCAN_GetRxMessage(data->hfdcanx, FDCAN_RX_FIFO0, &FDCAN_RxHeader, data_p) != HAL_OK)
            {

            }
        }

        if(data->State == FDCAN_IT_RX_FIFO0_FULL)
        {

        }
        Callback_Port2(FDCAN_RxHeader.Identifier, data_p, FDCAN_RxHeader.DataLength);
        return;
    }

    /*----------------------------------------------------------------------------------------------------*/

    else if(data->hfdcanx == FDCAN_Port3_set.hfdcanx)
    {
        if(data->State == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
        {
            if(HAL_FDCAN_GetRxMessage(data->hfdcanx, FDCAN_RX_FIFO0, &FDCAN_RxHeader, data_p) != HAL_OK)
            {

            }
        }

        if(data->State == FDCAN_IT_RX_FIFO0_FULL)
        {

        }
        Callback_Port3(FDCAN_RxHeader.Identifier, data_p, FDCAN_RxHeader.DataLength);
        return;
    }

}


/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/


__weak void mFDCAN_Class::Callback_Port1(uint32_t Id, uint8_t *data_p, uint8_t Len){/**/}
__weak void mFDCAN_Class::Callback_Port2(uint32_t Id, uint8_t *data_p, uint8_t Len){/**/}
__weak void mFDCAN_Class::Callback_Port3(uint32_t Id, uint8_t *data_p, uint8_t Len){/**/}

mFDCAN_Class mFDCAN;

extern "C"{


    /*----------------------------------------------------------------------------------------------------*/


    void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
    {
        mFDCAN.fdcan_TxCallBack.hfdcanx = hfdcan;
        mFDCAN.fdcan_TxCallBack.State = TxEventFifoITs;
        mFDCAN.TxCallback(&mFDCAN.fdcan_TxCallBack);
    }
    //Tx event callback


    /*----------------------------------------------------------------------------------------------------*/


    void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
    {
        mFDCAN.fdcan_RxCallBack_Fifo0.hfdcanx = hfdcan;
        mFDCAN.fdcan_RxCallBack_Fifo0.State = RxFifo0ITs;
        mFDCAN.RxCallback_Fifo0(&mFDCAN.fdcan_RxCallBack_Fifo0);
    }
    //Fifo0 Callback


    /*----------------------------------------------------------------------------------------------------*/
    
    
    void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
    {
        mFDCAN.fdcan_RxCallBack_Fifo1.hfdcanx = hfdcan;
        mFDCAN.fdcan_RxCallBack_Fifo1.State = RxFifo0ITs;
        mFDCAN.RxCallback_Fifo1(&mFDCAN.fdcan_RxCallBack_Fifo1);
    }
    //Fifo1 Callback


    /*----------------------------------------------------------------------------------------------------*/


    void HAL_FDCAN_TimeoutOccurredCallback(FDCAN_HandleTypeDef *hfdcan)
    {
        mFDCAN.Rx_TimeOut_flag = 1;
    }
    //timeout
    

    /*----------------------------------------------------------------------------------------------------*/


}