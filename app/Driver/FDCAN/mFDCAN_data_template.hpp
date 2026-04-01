
#pragma once
#include <cstdint>

class mFDCAN_template_Class
{
public:
    enum class fdcan_ports : uint8_t {
        FDCAN1_Port,
        FDCAN3_Port,
        FDCAN2_Port,
    };
    /**
     * ユーザー指定のport
     * 使い方はhfdcan3をport1と指定して読みやすくする
     * typedefは下のfdcan_setting_Handle_TypeDefを使う
     */

    enum class can_frame_type : uint8_t{
        classic_can,
        fdcan,
    };
    /**
     * classic canと fdcanを選べる
     * どちらを使うかは.iocファイルになんて設定してあるかを参照
     */

    enum class bit_rate_type : uint8_t {
        _1Mbps_,
        _2Mbps_,
        _3Mbps_,
        _4Mbps_,
        _5Mbps_,
    };

    enum class Fifo_num_type : uint8_t {
        FIFO0,
        FIFO1,
    };

    typedef struct {
        FDCAN_HandleTypeDef* hfdcanx;  // hfdcan
        fdcan_ports hfdcan_port;       // user port
        Fifo_num_type fifo_num;        /*使うfifo基本fifo0かfifo1*/
        can_frame_type hfdcan_frame;   // can frame
        uint16_t RxTimeOutCycle_ms;    // 0:Disable | 1 = 0xFFFF enable value ms
        bit_rate_type bit_rate; /*通信速度*/
    } fdcan_setting_Handle_TypeDef;
    /**
     * hfdcanxにhfdcanを入れる
     * hfdcan_portにfdcan_portsの値を入れる
     * hfdcan_frameにFDCANかClassic CANなのかを入れる
     * TxCycleは何秒毎に送信するか値はms
     * RxTimeOutCycleは受信の際前回の受信から何秒たったらtimeoutにするかの値
     */

    typedef struct {
        fdcan_ports FDCAN_Port;
        uint32_t Id;
        uint8_t Len;
        uint8_t* data_p;/*64Byteで作れ*/
    } fdcan_TxData_Handle_TypeDef;

    typedef struct {
        struct {
            bool Config;
            bool Start;
            bool RxCallBack_Fifo0;
            bool RxCallBack_Fifo1;
            bool TxCallBack;
            bool TxFifoEmpty_CallBack;
            bool TimeOut_CallBack;
        } Init;  // In Init Function
        struct {
            bool User_TxFifo_full_Port;
            bool TxFifo_full_Port;
            bool over_Id_value;
            bool Add_New_TxMessage;
        } Send;  // In Send Function
        struct {
        } TxCallBack;  // In TxCallBack Function
        struct {
        } RxCallBack;  // In RxCallBack Function
    } fdcan_State_Handle_TypeDef;
    /**
     * error_flag
     * 1 = Error
     * 0 = No Error
     */

protected:
    typedef struct {
        uint8_t tx_events;
    } fdcan_stack_event;

    fdcan_stack_event FDCAN_Port1_Stack;
    fdcan_stack_event FDCAN_Port2_Stack;
    fdcan_stack_event FDCAN_Port3_Stack;

    typedef struct {
        FDCAN_HandleTypeDef* hfdcanx;
        can_frame_type hfdcan_frame;
    } fdcan_port_setting_Handle_TypeDef;

    fdcan_port_setting_Handle_TypeDef FDCAN_Port1_set;
    fdcan_port_setting_Handle_TypeDef FDCAN_Port2_set;
    fdcan_port_setting_Handle_TypeDef FDCAN_Port3_set;

    typedef struct {
        FDCAN_HandleTypeDef* hfdcanx;
        uint32_t State;
    } fdcan_CallBack_Handle_TypeDef;

    static constexpr uint32_t dlc_table[16] = {
        FDCAN_DLC_BYTES_0,
        FDCAN_DLC_BYTES_1,
        FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4,
        FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8,
        FDCAN_DLC_BYTES_12,
        FDCAN_DLC_BYTES_16,
        FDCAN_DLC_BYTES_20,
        FDCAN_DLC_BYTES_24,
        FDCAN_DLC_BYTES_32,
        FDCAN_DLC_BYTES_48,
        FDCAN_DLC_BYTES_64,
    };


#define Data_len(len) ((len >= 0) && (len <= 64)) ? (len <= 8 ? dlc_table[len] : (len <= 12 ? dlc_table[9] : \
                      (len <= 16 ? dlc_table[10] : (len <= 20 ? dlc_table[11] : (len <= 24 ? dlc_table[12] : \
                      (len <= 32 ? dlc_table[13] : (len <= 48 ? dlc_table[14] : dlc_table[15]))))))) : dlc_table[0]
};
