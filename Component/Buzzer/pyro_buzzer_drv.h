#ifndef __PYRO_BUZZER_DRV_H__
#define __PYRO_BUZZER_DRV_H__

/**
 * @file pyro_buzzer_drv.h
 * @brief 蜂鸣器驱动（复用 pwm_drv_t；无队列设计，新播放覆盖式、暂停静音）。
 */

#include "pyro_core_def.h"
#include "pyro_pwm_drv.h"
#include "pyro_task.h"
#include "pyro_mutex.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <cstdint>

namespace pyro {

// 配置常量
constexpr uint32_t BUZZER_MAX_MELODY_LEN = 64;   // 旋律最大音符数
constexpr uint16_t BUZZER_DEFAULT_FREQ   = 4000; // beep 默认频率 Hz

// 音符枚举（半音序号：C=0, C#=1, ..., A=9, ..., B=11）
enum class note_t : uint8_t {
    C, CS, D, DS, E, F, FS, G, GS, A, AS, B
};

// 节奏型一步
struct rhythm_step_t {
    note_t   note;
    uint8_t  octave;
    uint16_t beats;   // 拍数（受 tempo 控制）
    bool     rest;    // 休止符
};

// 常用音符常量（C4 八度）
constexpr uint16_t NOTE_C4 = 262, NOTE_D4 = 294, NOTE_E4 = 330, NOTE_F4 = 349,
                   NOTE_G4 = 392, NOTE_A4 = 440, NOTE_B4 = 494;

/**
 * @brief 蜂鸣器驱动（播放器）。复用 pwm_drv_t，新播放覆盖式、暂停完全静音。
 */
class buzzer_drv_t {
  public:
    static buzzer_drv_t& get_instance();   // 单例

    buzzer_drv_t(const buzzer_drv_t&)            = delete;
    buzzer_drv_t& operator=(const buzzer_drv_t&) = delete;
    ~buzzer_drv_t() = default;

    // 基础（阻塞：入队 + 等待完成）
    status_t beep(uint32_t frequency, uint32_t duration_ms);
    status_t beep(uint32_t duration_ms);
    status_t start();
    status_t stop();
    bool is_playing() const;

    // 音符
    status_t play_note(uint16_t frequency, uint32_t duration_ms);
    status_t play_note(note_t note, uint8_t octave, uint32_t duration_ms);

    // 旋律
    status_t play_melody(const uint16_t notes[], const uint32_t durations[], uint32_t len);
    status_t play_melody_blocking(const uint16_t notes[], const uint32_t durations[], uint32_t len);
    status_t play_melody_async(const uint16_t notes[], const uint32_t durations[], uint32_t len);

    // 节奏（阻塞）
    status_t play_rhythm(const rhythm_step_t pattern[], uint32_t len);

    // RTTTL
    status_t play_rtttl(bool is_blocking, const char* rtttl);

    // 音量/静音
    status_t set_volume(uint8_t percent);   // 0-100 → duty 0~1
    uint8_t  get_volume() const;
    status_t mute();
    status_t unmute();

    // 速度
    void set_tempo(uint16_t bpm);
    uint16_t get_tempo() const;

    // 播放控制
    status_t pause();
    status_t resume();
    void set_loop(bool enable);

    // 工具
    static uint32_t note_to_freq(note_t note, uint8_t octave);

  private:
    buzzer_drv_t();                             // 私有：内部绑定 bsp_pwm

    // 提交新播放内容（覆盖式）并唤醒后台；wait_complete=true 时阻塞到播完
    status_t submit_play(const uint16_t notes[], const uint32_t durations[], uint32_t len,
                         bool wait_complete);

    bool take_pending();                   // 加锁读取待播内容；有则拷入 _run_* 并返回 true
    status_t store_play(const uint16_t notes[], const uint32_t durations[], uint32_t len);  // 加锁写入待播内容（覆盖），释放被覆盖的等待者
    void notify_done();                    // 定向唤醒当前阻塞等待者（播完/停止）
    void check_pending();                  // 有待播内容则覆盖并开始
    void update_playback();                // 逐音符推进

    static note_t rtttl_note(char c, bool sharp);   // RTTTL 音名（C..B + #）→ note_t

    class player_task_t : public task_base_t {
      public:
        explicit player_task_t(buzzer_drv_t* owner);
        void notify();                     // 唤醒后台任务（任务通知）
        bool is_self() const;              // 是否在后台任务上下文
      protected:
        status_t init() override;
        void run_loop() override;
      private:
        buzzer_drv_t* _owner;
    };

    pwm_drv_t* _pwm;
    player_task_t _task;
    pyro::mutex_t _mutex;                  // 保护输入区共享数据

    // 输入区：调用线程写 / 后台读（notes/durations/len/pending 用锁保护）
    struct play_state_t {
        bool pending = false;
        uint16_t notes[BUZZER_MAX_MELODY_LEN];
        uint16_t durations[BUZZER_MAX_MELODY_LEN];
        uint32_t len = 0;
        volatile bool paused    = false;
        volatile bool loop      = false;
        volatile bool need_stop = false;
    } _state;

    // 运行区：后台任务独占
    uint16_t _run_notes[BUZZER_MAX_MELODY_LEN];
    uint16_t _run_durations[BUZZER_MAX_MELODY_LEN];
    uint32_t _run_len = 0;
    bool     _playing = false;
    bool     _need_start = false;
    uint8_t  _idx = 0;
    uint32_t _note_start_tick = 0;

    // 当前阻塞等待者及其专用信号量（单等待者假设；_mutex 保护）
    struct waiter_t {
        TaskHandle_t task = nullptr;
        SemaphoreHandle_t sem = nullptr;
    } _waiter;

    uint8_t  _volume = 50;
    bool     _muted = false;
    volatile uint16_t _tempo_bpm = 120;
    volatile uint16_t _beat_ms = 125;     // 120BPM → 四分音符 125ms
};

} // namespace pyro

#endif // __PYRO_BUZZER_DRV_H__