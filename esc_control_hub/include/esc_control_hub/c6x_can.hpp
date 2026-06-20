#pragma once
#include <cstdint>

#include "gn10_can/drivers/can_driver_interface.hpp"

namespace c6x0_can {

// M2006　M3508の制御フレーム（送信）
constexpr int SEND_CANID_0_3 = 0x200;
constexpr int SEND_CANID_4_7 = 0x1FF;

// c610電流変換定数
constexpr float C610_CURRENT_CONVERSION = 100.0f;

// c620電流変換用定数
constexpr float C620_CURRENT_CONVERSION = 819.2f;

// feedback data
struct C620Feedback {
    uint16_t angle;       // 値域:0~8191
    int16_t speed_rpm;    // 値域：-32768 ~ 32767 / -10000~10000 単位：rpm
    int16_t current;      // 値域：-16384~16384 / -10000~10000
    uint8_t temperature;  // 値域：0~255 単位：℃
    uint8_t reserved;     // 空きデータ
} __attribute__((__packed__));

class C6XCAN
{
private:
    gn10_can::drivers::ICANDriver& can_driver_;
    C620Feedback feedback_[8];

public:
    // コンストラクタ
    C6XCAN(gn10_can::drivers::ICANDriver& can_driver) : can_driver_(can_driver) {}

    /**
     * @brief CAN Callback
     *
     */
    void update();

    /**
     * @brief 電流値設定
     *
     * @param currents 電流値
     * @return true 送信成功
     * @return false 送信失敗
     */
    bool set_currents_1_3(int16_t currents[4]);

    /**
     * @brief 電流値設定
     *
     * @param currents 電流値
     * @return true 送信成功
     * @return false 送信失敗
     */
    bool set_currents_4_7(int16_t currents[4]);

    /**
     * @brief feedbackのangleを読み取るgetter関数。
     *
     * @param motor_id 読みたい角度のデータがあるモーター番号 値域:0~7
     *
     * @return 値域：0-8192 右の範囲内の角度
     */
    uint16_t get_feedback_angle(const uint8_t motor_id) const;

    /**
     * @brief feedbackのspeedを読み取るgetter関数。
     *
     * @param motor_id 読みたい回転速度のデータがあるモーター番号 値域:0~7
     *
     * @return 単位:rpm の回転速度
     * C620:-32768~32768/
     * C610:-10000~10000
     */
    int16_t get_feedback_speed(const uint8_t motor_id) const;

    /**
     * @brief feedbackのcurrentを読み取るgetter関数。
     *
     * @param motor_id 読みたいトルク電流値のデータがあるモーター番号 値域:0~7
     *
     * @return 値域：C620-16384~16384 / C610:-10000~10000
     */
    int16_t get_feedback_current(const uint8_t motor_id) const;

    /**
     * @brief feedbackのtemperatureを読み取るgetter関数。
     *
     * @param motor_id 読みたい摂氏温度のデータがあるモーター番号 値域:0~7
     *
     * @return 値域：0~255 単位:℃　の摂氏温度 C620のみ対応
     */
    uint8_t get_feedback_temperature(const uint8_t motor_id) const;
};
}  // namespace c6x0_can