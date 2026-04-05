
#include "main_FW.hpp"

void mmain_FW_Class::setup()
{

    //gn10_can::drivers::DriverSTM32CAN fdcan_main;
    //gn10_can::CANBus can_bus(fdcan_main);
    //gn10_can::devices::

    fdcan_esc_setting.hfdcanx = &hfdcan2;
    fdcan_esc_setting.hfdcan_port = mFDCAN_template_Class::fdcan_ports::FDCAN2_Port;
    fdcan_esc_setting.fifo_num = mFDCAN_template_Class::Fifo_num_type::FIFO1;
    fdcan_esc_setting.hfdcan_frame = mFDCAN_template_Class::can_frame_type::classic_can;
    fdcan_esc_setting.RxTimeOutCycle_ms = 5;
    fdcan_esc_setting.bit_rate = mFDCAN_template_Class::bit_rate_type::_1Mbps_;
    if(mFDCAN.Init(&fdcan_esc_setting))
    {
        /*エラー処理を書く*/
    }

    HAL_TIM_Base_Start(&htim2); //10kHz
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);//Output PWM
    HAL_TIM_Base_Start(&htim6); //1kHz

    if(HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK)
    {
        /*エラーだよ ENC1*/
    }
    if(HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK)
    {
        /*エラーだよ ENC2*/
    }
    if(HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
    {
        /*エラーだよ ENC3*/
    }
    if(HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
    {
        /*エラーだよ ENC4*/
    }

    CAN_Data.Data_p = CAN_Data.Data;

    if(mFDCAN.Enable_timeout(mFDCAN_template_Class::fdcan_ports::FDCAN2_Port))
    {
        /*エラー*/
    }

    fdcan_txdata.FDCAN_Port = mFDCAN_template_Class::fdcan_ports::FDCAN1_Port;
    fdcan_txdata.Id = 0x00;
    fdcan_txdata.Len = 0x01;
    fdcan_txdata.data_p = NULL;
    if(mFDCAN.Send(&fdcan_txdata))
    {
        /*エラー処理を書く*/
    }
}

void mmain_FW_Class::loop()
{
    if(NVIC_1Hz)
    {
        NVIC_1Hz = 0;
    }

    if(NVIC_1kHz)
    {

        NVIC_1kHz = 0;
    }

    if(NVIC_10kHz)
    {

        NVIC_10kHz = 0;
    }

    if(NVIC_main_FDCAN)
    {

        NVIC_main_FDCAN = 0;
    }

    if(NVIC_esc_CAN)
    {

        NVIC_esc_CAN = 0;
    }
}

inline void mmain_FW_Class::Get_Encoder()
{

    Encoder_Count_Port1 = __HAL_TIM_GET_COUNTER(&htim8);

    Encoder_Count_Port2 = __HAL_TIM_GET_COUNTER(&htim1);

    Encoder_Count_Port3 = __HAL_TIM_GET_COUNTER(&htim3);

    Encoder_Count_Port4 = __HAL_TIM_GET_COUNTER(&htim4);

}

mmain_FW_Class mmain_FW;

void mFDCAN_Class::Callback_Port2(uint32_t Id, uint8_t *data_p, uint8_t Len)
{

    mmain_FW.CAN_Data.Id = Id;
    mmain_FW.CAN_Data.Data_p = data_p;
    mmain_FW.CAN_Data.Len = Len;
    mmain_FW.NVIC_esc_CAN = 1;
    
}

extern "C"
{

    void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
    {
        /**
         * 1 Hzの割り込み 
         */

        mmain_FW.Get_Encoder();
        mmain_FW.NVIC_1Hz = 1;
    }

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
    {
        if(htim == &htim2)
        {
            /**
             * 10 kHzの割り込み 
             */

            mmain_FW.Get_Encoder();
            mmain_FW.NVIC_1kHz = 1;
        }
        
        if(htim == &htim6)
        {
            /**
             * 1 kHzの割り込み 
             */

            mmain_FW.Get_Encoder();
            mmain_FW.NVIC_10kHz = 1;
        }
    }


}