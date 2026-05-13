
#include "driver_fdcan.hpp"

namespace gn10_can {
namespace drivers {

bool DriverSTM32FDCAN::init()
{
    if (main_silent_fdcan.init()) {
        return 1;
    }

    return 0;
}

bool DriverSTM32FDCAN::send(const FDCANFrame& frame)
{
    data_frame.id     = frame.id;
    data_frame.len    = frame.dlc;
    data_frame.data_p = const_cast<uint8_t*>(frame.data.data());

    if (main_silent_fdcan.SendMessage(&data_frame)) return 1;

    return 0;
}

bool DriverSTM32FDCAN::receive(FDCANFrame& out_frame)
{
    out_frame.id          = gn10_FDCAN_RxHeader.Identifier;
    out_frame.dlc         = gn10_FDCAN_RxHeader.DataLength;
    out_frame.is_extended = (gn10_FDCAN_RxHeader.IdType == FDCAN_EXTENDED_ID);

    for (uint8_t i = 0; i < out_frame.dlc; i++) {
        out_frame.data[i] = data[i];
    }

    return 0;
}

}  // namespace drivers
}  // namespace gn10_can

maidui3_xcan::xcan main_silent_fdcan(
    NULL,
    maidui3_xcan::fifo::FIFO0,
    maidui3_xcan::can_frame::FDCAN,
    maidui3_xcan::id_filter_type::Non_mask_id,
    0,
    0
);

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        if (HAL_FDCAN_GetRxMessage(
                main_silent_fdcan.setup_type.hxcan_,
                FDCAN_RX_FIFO0,
                &main_bus_fdcan.gn10_FDCAN_RxHeader,
                main_bus_fdcan.data
            ))
            return;
        main_fdcan_bus.update();
        return;
    }
}