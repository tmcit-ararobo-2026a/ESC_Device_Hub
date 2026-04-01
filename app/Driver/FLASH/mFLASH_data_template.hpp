
#pragma once
#include <cstdint>

class mFLASH_data_template_Class{
    public:
        enum class State : uint8_t{
            Standby,
            Writetting,
            Reading,
        };
};

/**
 * IO0 : SI
 * IO1 : SO
 * IO2 : WP#
 * IO3 : HOLD# or RESET#
 *
 * CE#はHIGHがスタンバイモード LOWが選択中
 * IO0 IO1 IO2 IO3 はSCKの立ち上がりで、命令、アドレス、データを書き込み
 * 立下りでデバイスのデータまたはステータスを読み取る
 */