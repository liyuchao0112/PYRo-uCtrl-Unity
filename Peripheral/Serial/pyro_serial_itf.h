#ifndef __PYRO_SERIAL_ITF_H__
#define __PYRO_SERIAL_ITF_H__

#include "pyro_core_def.h"
#include "FreeRTOS.h"
#include <cstdint>
#include <functional>

namespace pyro
{

/**
 * @brief "字节流链路"抽象接口：UART 与 USB-CDC 的统一契约。
 *
 * 契约（驱动层必须对调用层兑现，调用层不得感知链路类型）：
 *  1) 一次 rx 回调 = 一个完整帧（由驱动层内部组帧保证，见 set_frame_config）；
 *  2) write() 返回 PYRO_BUSY 表示"没发出去，可重试"；
 *  3) "在线"判定由调用层超时逻辑负责，驱动层只提供物理收发能力。
 *
 * 成员范围：本接口只包含**抽象消费者（infantry2_autoaim_drv_t）通过
 * serial_itf_t* 实际调用的方法**。链路开启与参数配置不属于本契约：
 *   uart_drv_t    -> reset() + enable_rx_dma()
 *   usb_cdc_drv_t -> start()  + enable_rx()
 * 它们由各驱动自行提供，调用点在 pyro_init_thread.cpp 的 #ifdef 分派处。
 */
class serial_itf_t
{
  public:
    /**
     * @brief 接收回调。p 为字节流（若已 set_frame_config，则一次调用恰为一整帧）。
     * @note 调用上下文由实现决定：
     *       uart_drv_t    -> DMA/IDLE 中断（ISR）
     *       usb_cdc_drv_t -> TinyUSB 设备任务（tud_task）
     *       实现在任务上下文时会传入 woken = pdFALSE，回调内不得调用 portYIELD_FROM_ISR。
     * @warning p 指向驱动内部缓冲，**仅在本次回调期间有效**，回调方必须自行拷贝
     *          （现有调用层是 memcpy 进 MessageBuffer，满足该约束）。
     *          回调内也不得做耗时/阻塞操作：USB 实现下会拖慢整个 USB 栈。
     */
    using rx_event_func = std::function<bool(
        uint8_t *p, uint16_t size, BaseType_t &xHigherPriorityTaskWoken)>;

    virtual ~serial_itf_t() = default;

    /**
     * @brief 非阻塞写。
     * @note 只允许在任务上下文调用，**禁止在 ISR 中调用**（USB 实现内部会走 TinyUSB FIFO
     *       的 osal 互斥；跨任务调用是安全的）。
     * @return PYRO_OK=已入队；PYRO_BUSY=缓冲不足可重试；PYRO_ERROR=链路不可用（未挂载等）。
     */
    virtual status_t write(const uint8_t *p, uint16_t size) = 0;

    /** @brief 阻塞写（带超时）。 */
    virtual status_t write(const uint8_t *p, uint16_t size, uint32_t waittime) = 0;

    /** @brief 注册接收回调（owner 用于注销，通常传 this）。 */
    virtual void add_rx_event_callback(const rx_event_func &func,
                                       uint32_t owner) = 0;

    /** @brief 按 owner 注销接收回调。 */
    virtual status_t remove_rx_event_callback(uint32_t owner) = 0;

    /**
     * @brief 配置驱动层组帧：一次 rx 回调保证给到 [SOF + frame_len] 的完整帧。
     * @param sof       帧头字节（如 0xA5）
     * @param frame_len 整帧长度（含帧头与校验）
     * @note 传 sof = 0 关闭组帧（透传字节流），默认关闭。
     *       uart_drv_t 的实现为 no-op（保持现有 IDLE 语义不变）。
     */
    virtual void set_frame_config(uint8_t sof, uint16_t frame_len) = 0;
};

} // namespace pyro

#endif // __PYRO_SERIAL_ITF_H__
