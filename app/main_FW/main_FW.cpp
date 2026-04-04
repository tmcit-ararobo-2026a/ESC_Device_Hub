
#include "main_FW.hpp"

void mmain_FW_Class::setup(){
    fdcan_main_setting.hfdcanx = &hfdcan1;
    fdcan_main_setting.hfdcan_port = mFDCAN_template_Class::fdcan_ports::FDCAN1_Port;
    fdcan_main_setting.fifo_num = mFDCAN_template_Class::Fifo_num_type::FIFO0;
    fdcan_main_setting.hfdcan_frame = mFDCAN_template_Class::can_frame_type::fdcan;
    fdcan_main_setting.RxTimeOutCycle_ms = 5;
    fdcan_main_setting.bit_rate = mFDCAN_template_Class::bit_rate_type::_1Mbps_;
    if(mFDCAN.Init(&fdcan_main_setting))
    {
        /*エラー処理を書く*/
    }

    fdcan_esc_setting.hfdcanx = &hfdcan2;
    fdcan_esc_setting.hfdcan_port = mFDCAN_template_Class::fdcan_ports::FDCAN2_Port;
    fdcan_esc_setting.fifo_num = mFDCAN_template_Class::Fifo_num_type::FIFO1;
    fdcan_main_setting.hfdcan_frame = mFDCAN_template_Class::can_frame_type::classic_can;
    fdcan_esc_setting.RxTimeOutCycle_ms = 0;
    fdcan_esc_setting.bit_rate = mFDCAN_template_Class::bit_rate_type::_1Mbps_;
    if(mFDCAN.Init(&fdcan_esc_setting))
    {
        /*エラー処理を書く*/
    }

    mFDCAN.Enable_timeout(mFDCAN_template_Class::fdcan_ports::FDCAN1_Port);
    mFDCAN.Enable_timeout(mFDCAN_template_Class::fdcan_ports::FDCAN2_Port);

    fdcan_txdata.FDCAN_Port = mFDCAN_template_Class::fdcan_ports::FDCAN1_Port;
    fdcan_txdata.Id = 0x00;
    fdcan_txdata.Len = 0x01;
    fdcan_txdata.data_p = NULL;
    if(mFDCAN.Send(&fdcan_txdata))
    {
        /*エラー処理を書く*/
    }
}

void mmain_FW_Class::loop(){

}

void mFDCAN_Class::Callback_Port1(uint32_t Id, uint8_t *data_p, uint8_t Len){

}

void mFDCAN_Class::Callback_Port2(uint32_t Id, uint8_t *data_p, uint8_t Len){

}

void mFDCAN_Class::Callback_Port3(uint32_t Id, uint8_t *data_p, uint8_t Len){

}

mmain_FW_Class mmain_FW;
