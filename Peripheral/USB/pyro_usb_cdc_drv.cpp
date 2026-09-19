#include "pyro_usb_cdc_drv.h"

#include "tusb.h"
#include "stm32h7xx_hal.h"

// 自测回环档位（生产环境保持 0；需要自测时由 CMake 定义 1 或 2）
//   0 = 正常业务路径   1 = 字节回环   2 = 经驱动层组帧后再回环
#ifndef USB_CDC_LOOPBACK
#define USB_CDC_LOOPBACK 0
#endif

namespace pyro
{

/* ============================ Hardware Init ============================ */

status_t usb_cdc_drv_t::usb_task_t::init()
{
    // 说明：USB 的时钟源（HSI48 / PLL1Q / PLL3Q）、USB 电压检测器、OTG_HS 外设时钟、
    //       以及 NVIC 优先级与使能，全部由 CubeMX 生成并维护的 HAL_PCD_MspInit() 完成
    //       （在 CubeMX/Core/Src/usb_otg.c 的 USER CODE 区被显式调用）。
    //       这样时钟树变化（例如把 USB 时钟从 PLL1Q 换成 HSI48）时无需改本驱动。
    //       ⚠️ 切勿在此硬编码 RCC_USBCLKSOURCE_*，否则与 .ioc 不符会直接导致 USB 无法枚举。

    // DP/DM 引脚：CubeMX 的 gpio.c 未配置 USB 引脚，故在此配置。
    // 内置 FS PHY 固定使用 PA11(D-) / PA12(D+)，AF10。
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {};
    gpio.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF10_OTG1_HS;   // H723: == GPIO_AF10_OTG2_HS (0x0A)
    HAL_GPIO_Init(GPIOA, &gpio);

    // 强制中断优先级满足 FreeRTOS 约束（必须 <= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY）。
    // MspInit 里已设过一次（值为 5），这里再显式覆盖一次，以防后续 CubeMX 改动优先级。
    HAL_NVIC_SetPriority(OTG_HS_IRQn,
                         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);

    return PYRO_OK;
}

void usb_cdc_drv_t::usb_task_t::run_loop()
{
    // 必须在调度器启动后调用（ISR 中使用了 RTOS 队列 API）
    const tusb_rhport_init_t dev_init = {
        .role  = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,     // 内置 FS PHY
    };
    tusb_init(BOARD_TUD_RHPORT, &dev_init);

    while (true)
    {
        tud_task();                   // 事件处理；期间会回调 tud_cdc_rx_cb()
    }
}

/* ============================ Life Cycle =============================== */

usb_cdc_drv_t::usb_cdc_drv_t()
{
    // 预分配回调表容量：避免 add_rx_event_callback() 在临界区内触发内存分配
    _rx_cbs.reserve(MAX_RX_CALLBACKS);
}

usb_cdc_drv_t::~usb_cdc_drv_t()
{
    if (_task)
    {
        _task->stop();
        delete _task;
        _task = nullptr;
    }
}

usb_cdc_drv_t &usb_cdc_drv_t::instance()
{
    static usb_cdc_drv_t inst;
    return inst;
}

status_t usb_cdc_drv_t::start()
{
    if (_task)
        return PYRO_ALREADY_INIT;

    _task = new usb_task_t(this);
    if (!_task)
        return PYRO_NO_MEMORY;

    return _task->start();
}

/* ======================= serial_itf_t 实现 ============================= */

status_t usb_cdc_drv_t::write(const uint8_t *p, uint16_t size)
{
    if (!p || size == 0 || !tud_mounted())
        return PYRO_ERROR;

    // 注意：不严格依赖 DTR（部分上位机不置 DTR），可发与否由调用层超时逻辑兜底；
    //       因此"已枚举但无人打开端口"时数据会被静默丢弃。若需更强语义，
    //       可改为依赖 tud_cdc_connected()，并把
    //       CFG_TUD_CDC_TX_OVERWRITABLE_IF_NOT_CONNECTED 置 0（见 tusb_config.h）。
    if (tud_cdc_write_available() < size)
        return PYRO_BUSY;

    const uint32_t written = tud_cdc_write(p, size);
    tud_cdc_write_flush();
    return (written == size) ? PYRO_OK : PYRO_ERROR;
}

status_t usb_cdc_drv_t::write(const uint8_t *p, uint16_t size, uint32_t)
{
    return write(p, size);
}

/* ===================== USB 专有接收控制 ========================== */

status_t usb_cdc_drv_t::enable_rx()
{
#if USB_CDC_LOOPBACK == 2
    // 自测档位 2：用自瞄的帧参数开启组帧，从而验证 frame_parser_t 的切帧行为
    _parser.configure(0xA5, 29);
#endif
    _rx_enabled = true;
    return PYRO_OK;
}

status_t usb_cdc_drv_t::disable_rx()
{
    _rx_enabled = false;
    return PYRO_OK;
}

void usb_cdc_drv_t::add_rx_event_callback(const rx_event_func &func,
                                           uint32_t owner)
{
    rx_cb_entry_t e = {};
    e.owner = owner;
    e.func  = func;

    // 与 dispatch() 的遍历互斥，避免 vector 重新分配期间的并发访问（会破坏堆）
    taskENTER_CRITICAL();
    if (_rx_cbs.size() < MAX_RX_CALLBACKS)
        _rx_cbs.push_back(e);
    taskEXIT_CRITICAL();
}

status_t usb_cdc_drv_t::remove_rx_event_callback(uint32_t owner)
{
    status_t ret = PYRO_NOT_FOUND;

    taskENTER_CRITICAL();
    for (auto it = _rx_cbs.begin(); it != _rx_cbs.end(); ++it)
    {
        if (it->owner == owner)
        {
            _rx_cbs.erase(it);
            ret = PYRO_OK;
            break;
        }
    }
    taskEXIT_CRITICAL();

    return ret;
}

void usb_cdc_drv_t::set_frame_config(uint8_t sof, uint16_t frame_len)
{
    _parser.configure(sof, frame_len);
}

bool usb_cdc_drv_t::is_mounted() const
{
    return tud_mounted();
}

bool usb_cdc_drv_t::is_connected() const
{
    return tud_mounted() && tud_cdc_connected();
}

/* ============================ RX / 分发 =============================== */

void usb_cdc_drv_t::dispatch(const uint8_t *p, uint16_t size)
{
    if (!_rx_enabled)
        return;

    // 回调表快照：add_rx_event_callback() / remove_rx_event_callback() 可能由其它任务
    // （各模块的 init 任务）并发调用；若直接遍历 _rx_cbs，会在 vector 重新分配时导致
    // 迭代器失效甚至堆破坏。故在短临界区内取快照，回调本身在临界区外执行
    // （回调必须短小且不得阻塞，见 serial_itf_t 的接口约定）。
    rx_event_func cbs[MAX_RX_CALLBACKS];
    uint8_t n = 0;
    taskENTER_CRITICAL();
    for (auto &e : _rx_cbs)
    {
        if (n >= MAX_RX_CALLBACKS)
            break;
        cbs[n++] = e.func;
    }
    taskEXIT_CRITICAL();

    BaseType_t woken = pdFALSE;   // 任务上下文：无需 portYIELD

#if USB_CDC_LOOPBACK == 2
    // 自测档位 2：无接收方时回环（组帧开启则回环整帧），用于验证 USB 通路与切帧
    if (n == 0)
    {
        if (_parser.enabled())
        {
            _parser.feed(p, size,
                         [](const uint8_t *frame, uint16_t len) -> bool
                         {
                             loopback_write(frame, len);
                             return true;
                         });
        }
        else
        {
            loopback_write(p, size);
        }
        return;
    }
#endif

    if (n == 0)
        return;   // 尚无接收方（上层还没注册回调）：丢弃

    if (!_parser.enabled())
    {
        for (uint8_t i = 0; i < n; ++i)
        {
            if (cbs[i](const_cast<uint8_t *>(p), size, woken))
                break;
        }
        return;
    }

    _parser.feed(p, size,
                 [&](const uint8_t *frame, uint16_t len) -> bool
                 {
                     for (uint8_t i = 0; i < n; ++i)
                     {
                         if (cbs[i](const_cast<uint8_t *>(frame), len, woken))
                             return true;
                     }
                     return true;   // 继续切下一帧
                 });
}

void usb_cdc_drv_t::loopback_write(const uint8_t *p, uint16_t size)
{
    const uint32_t avail = tud_cdc_write_available();
    const uint32_t n = (size < avail) ? size : avail;
    if (n == 0)
        return;
    tud_cdc_write(p, n);
    tud_cdc_write_flush();
}

void usb_cdc_drv_t::on_cdc_rx()
{
    uint8_t buf[64];

    while (tud_cdc_available())
    {
        const uint32_t n = tud_cdc_read(buf, sizeof(buf));
        if (n == 0)
            break;

#if USB_CDC_LOOPBACK == 1
        // 档位 1：纯字节回环，只验证 USB 收发通路
        loopback_write(buf, (uint16_t) n);
#else
        // 档位 0（正常业务路径）与档位 2（经组帧后回环，见 dispatch 内部 #if）
        dispatch(buf, (uint16_t) n);
#endif
    }
}

void usb_cdc_drv_t::on_mount()
{
#if USB_CDC_LOOPBACK != 0
    static const char msg[] = "[USB] mounted\r\n";
    loopback_write(reinterpret_cast<const uint8_t *>(msg),
                   (uint16_t) (sizeof(msg) - 1));
#endif
}

void usb_cdc_drv_t::on_umount()
{
    _parser.reset();              // 丢弃半帧，重新同步
    tud_cdc_read_flush();
}

void usb_cdc_drv_t::on_line_state(bool dtr, bool rts)
{
    (void) rts;
#if USB_CDC_LOOPBACK != 0
    static const char dtr_on[]  = "[USB] dtr=1\r\n";
    static const char dtr_off[] = "[USB] dtr=0\r\n";
    const char *msg = dtr ? dtr_on : dtr_off;
    loopback_write(reinterpret_cast<const uint8_t *>(msg),
                   (uint16_t) (dtr ? (sizeof(dtr_on) - 1)
                                   : (sizeof(dtr_off) - 1)));
#else
    (void) dtr;
#endif
}

} // namespace pyro

/* ============================ C 桥接层 ================================ */
extern "C" {

void tud_cdc_rx_cb(uint8_t itf)
{
    (void) itf;
    pyro::usb_cdc_drv_t::instance().on_cdc_rx();
}

void tud_mount_cb(void)
{
    pyro::usb_cdc_drv_t::instance().on_mount();
}

void tud_umount_cb(void)
{
    pyro::usb_cdc_drv_t::instance().on_umount();
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void) remote_wakeup_en;
}

void tud_resume_cb(void)
{
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void) itf;
    pyro::usb_cdc_drv_t::instance().on_line_state(dtr, rts);
}

/**
 * @brief 供 CubeMX 生成的 OTG_HS_IRQHandler 转发的桥接函数。
 * @note 中断向量的定义保留在 CubeMX/Core/Src/stm32h7xx_it.c 中（其 USER CODE 区调用本函数），
 *       这样 CubeMX 重新生成代码不会造成符号丢失或重复。
 *       编号必须与 tusb_init()/BOARD_TUD_RHPORT 一致。
 */
void pyro_usb_irq_handler(void)
{
    tusb_int_handler(BOARD_TUD_RHPORT, true);
}

} // extern "C"
