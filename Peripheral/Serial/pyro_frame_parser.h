#ifndef __PYRO_FRAME_PARSER_H__
#define __PYRO_FRAME_PARSER_H__

#include <cstdint>

namespace pyro
{

/**
 * @brief 定长帧切分器（字节流 -> 完整帧）。
 *
 * 用法：把每个字节喂入 feed()，内部扫描 SOF，凑满 frame_len 字节即回调一次 on_frame。
 *
 * 特性：
 *  - 纯数据、无锁、无动态分配；单帧缓冲固定 MAX_FRAME_LEN。
 *  - 只在同一上下文被访问（UART ISR 或 USB 任务，二者互斥），故无需加锁。
 *  - 不做内容校验（CRC 由上层业务校验），本类只负责定界。
 */
class frame_parser_t
{
  public:
    static constexpr uint16_t MAX_FRAME_LEN = 64; // 常见业务帧约 29B，留足余量

    /** @brief 置位 SOF 与帧长。frame_len 必须 <= MAX_FRAME_LEN。 */
    void configure(uint8_t sof, uint16_t frame_len)
    {
        _sof = sof;
        _len = (frame_len == 0 || frame_len > MAX_FRAME_LEN) ? 0 : frame_len;
        _idx = 0;
    }

    /** @brief 丢弃半帧、重新同步（掉线/拔插时调用）。 */
    void reset() { _idx = 0; }

    /** @brief 是否已配置帧长。 */
    [[nodiscard]] bool enabled() const { return _len != 0; }

    /**
     * @brief 喂入一段字节流，每凑满一帧调用一次 on_frame(frame, len)。
     * @param on_frame 返回 false 可提前中止本次喂入。
     * @return 本次产生的完整帧数量。
     */
    template <typename F>
    uint16_t feed(const uint8_t *p, uint16_t size, F &&on_frame)
    {
        uint16_t produced = 0;
        for (uint16_t i = 0; i < size; ++i)
        {
            const uint8_t b = p[i];
            if (_idx == 0 && b != _sof)
                continue;              // 帧外字节：丢弃
            _buf[_idx++] = b;
            if (_idx == _len)
            {
                _idx = 0;
                ++produced;
                if (!on_frame(_buf, _len))
                    break;
            }
        }
        return produced;
    }

  private:
    uint8_t  _sof{0x00};    // 未 configure() 时 _len = 0（组帧未启用），默认值不参与切帧
    uint16_t _len{0};
    uint16_t _idx{0};
    uint8_t  _buf[MAX_FRAME_LEN]{};
};

} // namespace pyro

#endif // __PYRO_FRAME_PARSER_H__
