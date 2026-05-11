
#include "driver_fdcan.hpp"

namespace gn10_can {
namespace drivers {

bool DriverSTM32FDCAN::init()
{
    return 0;
}

bool DriverSTM32FDCAN::send(const FDCANFrame& frame)
{
    return 0;
}

bool DriverSTM32FDCAN::receive(FDCANFrame& out_frame)
{
    return 0;
}

}  // namespace drivers
}  // namespace gn10_can

// void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
//{
//     if (RxFifo0ITs == FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
//     } else if (RxFifo0ITs == FDCAN_IT_RX_FIFO0_FULL) {
//     } else if (RxFifo0ITs == FDCAN_IT_RX_FIFO0_MESSAGE_LOST) {
//     } else {
//     }
// }