#ifndef __PYRO_USB_CDC_DRV_H__
#define __PYRO_USB_CDC_DRV_H__

#include "pyro_serial_itf.h"
#include "pyro_frame_parser.h"
#include "pyro_task.h"
#include <vector>

namespace pyro
{

/**
 * @brief 基于 TinyUSB(CDC-ACM) 的虚拟串口驱动。
 *
 * 对调用层完全等价于 uart_drv_t —— 本类内部消化所有 USB 与 UART 的差异：
 *  - 链路开启用 start(serial)；接收开关用 enable_rx()/disable_rx()
 *    （USB 无波特率/字长/停止位/校验位，故不存在 reset 概念）
 *  - 设备身份：厂商/产品为编译期常量（pyro_usb_desc_config.h），
 *    序列号由调用层经 start(serial) 注入（必填，见该参数说明）
 *  - 接收在 USB 任务上下文回调，woken 恒为 pdFALSE
 *  - set_frame_config() 生效后，一次回调恰好给到一整帧（内部 frame_parser_t 组帧）
 *
 * @warning instance() 的静态局部变量初始化不是 ISR-safe：必须先由任务上下文调用
 *          start()/instance()，之后 tud_* 回调才会使用它。
 * @warning write() 只允许在任务上下文调用，禁止在 ISR 中调用。
 */
class usb_cdc_drv_t final : public serial_itf_t
{
  public:
    static usb_cdc_drv_t &instance();

    /**
     * @brief 启动 USB 设备栈（创建内部任务：tusb_init + tud_task 循环）。（不传参默认值）
     * @return PYRO_OK / PYRO_ALREADY_INIT / PYRO_PARAM_ERROR（序列号非法）/ PYRO_NO_MEMORY
     * @note 序列号在 tusb_init() 之前写入描述符表；运行期再改无效（主机已缓存枚举结果）。
     * @warning 空指针/空串会被拒绝 —— 产品名固定后，无序列号会使 Windows 按端口位置建实例。
     */
    status_t start();

    /**
     * @brief 启动 USB 设备栈（创建内部任务：tusb_init + tud_task 循环）。
     * @param serial 设备序列号（字符串描述符 index 3），**必填**：由应用层自行书写，
     *               可打印 ASCII（0x20~0x7E）、非空、长度 ≤ PYRO_USB_SERIAL_MAX_LEN(31)、
     *               且必须为**静态存储**（字符串字面量天然满足）。
     *               建议格式 "<机型>-<板别>"（例 "INFANTRY2-GIMBAL"），唯一性由调用者负责。
     * @return PYRO_OK / PYRO_ALREADY_INIT / PYRO_PARAM_ERROR（序列号非法）/ PYRO_NO_MEMORY
     * @note 序列号在 tusb_init() 之前写入描述符表；运行期再改无效（主机已缓存枚举结果）。
     * @warning 空指针/空串会被拒绝 —— 产品名固定后，无序列号会使 Windows 按端口位置建实例。
     */
    status_t start(const char *serial);

    /* ------------------ USB 专有接收控制 ------------------ */
    /** @brief 使能接收：置位内部开关，开始把收到的整帧分发给已注册回调。 */
    status_t enable_rx();

    /** @brief 停用接收（不再向回调分发，USB 设备栈继续运行）。 */
    status_t disable_rx();

    /* ------------------ serial_itf_t 实现 ------------------ */
    status_t write(const uint8_t *p, uint16_t size) override;
    status_t write(const uint8_t *p, uint16_t size, uint32_t waittime) override;
    void     add_rx_event_callback(const rx_event_func &func, uint32_t owner) override;
    status_t remove_rx_event_callback(uint32_t owner) override;
    void     set_frame_config(uint8_t sof, uint16_t frame_len) override;

    /* ------------------ 状态查询 ------------------ */
    [[nodiscard]] bool is_mounted() const;    // 已枚举并配置
    // 注意：write() 当前不依赖 DTR（放宽策略）。本接口供切换到"必须置 DTR"策略时使用。
    [[nodiscard]] bool is_connected() const;  // 已挂载且主机置了 DTR

    /* ------------------ 供 C 回调桥接 ------------------ */
    void on_cdc_rx();                        // tud_cdc_rx_cb -> 读 FIFO + 分发给已注册回调
    void on_mount();                         // tud_mount_cb
    void on_umount();                        // tud_umount_cb
    void on_line_state(bool dtr, bool rts);  // tud_cdc_line_state_cb

  private:
    usb_cdc_drv_t();
    ~usb_cdc_drv_t() override;

    /** @brief 内部 FreeRTOS 任务：先做硬件初始化，再跑 USB 栈。 */
    class usb_task_t final : public task_base_t
    {
      public:
        explicit usb_task_t(usb_cdc_drv_t *owner)
            : task_base_t("usb_task", 256, 512, priority_t::ABOVE_NORMAL),
              _owner(owner) {}

      protected:
        status_t init() override;     // 时钟 / PHY / GPIO / NVIC
        void run_loop() override;     // tusb_init() + while(1) tud_task()

      private:
        usb_cdc_drv_t *_owner;
    };

    /** @brief 可注册的接收回调上限（用于无分配的快照缓冲）。 */
    static constexpr uint8_t MAX_RX_CALLBACKS = 4;

    struct rx_cb_entry_t
    {
        uint32_t owner;
        rx_event_func func;
    };

    /** @brief 把一段字节流分发给已注册回调（按需组帧）。 */
    void dispatch(const uint8_t *p, uint16_t size);

    usb_task_t *_task{nullptr};
    std::vector<rx_cb_entry_t> _rx_cbs{};
    frame_parser_t _parser{};
    bool _rx_enabled{false};
};

} // namespace pyro

#endif // __PYRO_USB_CDC_DRV_H__
