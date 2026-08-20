#include "pyro_buzzer_drv.h"
#include "pyro_bsp_pwm.h"

#include <cctype>    // isdigit, toupper
#include <cstdio>    // sscanf
#include <cstring>   // memcpy, strchr
#include <cmath>     // pow, lround

namespace pyro {

uint32_t buzzer_drv_t::note_to_freq(note_t note, uint8_t octave) {
    const int32_t semi = (int32_t)note - 9 + (int32_t)(octave - 4) * 12;
    return (uint32_t)std::lround(440.0 * std::pow(2.0, semi / 12.0));
}

// RTTTL 音名（C D E F G A B + # 升半音）→ note_t
note_t buzzer_drv_t::rtttl_note(char c, bool sharp) {
    switch (c) {
        case 'C': return sharp ? note_t::CS : note_t::C;
        case 'D': return sharp ? note_t::DS : note_t::D;
        case 'E': return note_t::E;
        case 'F': return sharp ? note_t::FS : note_t::F;
        case 'G': return sharp ? note_t::GS : note_t::G;
        case 'A': return sharp ? note_t::AS : note_t::A;
        case 'B': return note_t::B;
        default:  return note_t::C;
    }
}

buzzer_drv_t& buzzer_drv_t::get_instance() {
    static buzzer_drv_t instance;
    return instance;
}

buzzer_drv_t::buzzer_drv_t()
    : _pwm(bsp_pwm::get_pwm(bsp_pwm::pwm_tim12_ch2)), _task(this) {
    _task.start();
}

status_t buzzer_drv_t::store_play(const uint16_t notes[], const uint32_t durations[], uint32_t len) {
    pyro::scoped_mutex_t lock(_mutex);
    if (_waiter.sem != nullptr) {            // 有旧等待者被覆盖 → 定向唤醒它
        xSemaphoreGive(_waiter.sem);
        _waiter.task = nullptr;
        _waiter.sem  = nullptr;
    }
    _state.pending = true;                   // 覆盖：最新内容直接写入
    _state.len = len;
    for (uint32_t i = 0; i < len; i++) {
        _state.notes[i] = notes[i];
        _state.durations[i] = (uint16_t)durations[i];
    }
    return PYRO_OK;
}

void buzzer_drv_t::notify_done() {
    pyro::scoped_mutex_t lock(_mutex);
    if (_waiter.sem != nullptr) {
        xSemaphoreGive(_waiter.sem);
        _waiter.task = nullptr;
        _waiter.sem  = nullptr;
    }
}

status_t buzzer_drv_t::submit_play(const uint16_t notes[], const uint32_t durations[],
                                   uint32_t len, bool wait_complete) {
    if (len > BUZZER_MAX_MELODY_LEN) return PYRO_PARAM_ERROR;

    // 后台任务内调用：直接同步播放，避免等自己完成而死锁
    if (_task.is_self()) {
        for (uint32_t i = 0; i < len; i++) {
            if (notes[i] != 0u) { _pwm->set_frequency(notes[i]); _pwm->start(); }
            vTaskDelay(pdMS_TO_TICKS(durations[i]));
            _pwm->stop();
        }
        return PYRO_OK;
    }

    const status_t r = store_play(notes, durations, len);  // 加锁写入（覆盖），并释放被覆盖的等待者
    if (r != PYRO_OK) return r;
    _task.notify();

    if (wait_complete) {                     // 阻塞：用专用信号量等"播完或被覆盖"
        SemaphoreHandle_t my_sem = xSemaphoreCreateBinary();
        if (my_sem == nullptr) return PYRO_NO_MEMORY;
        {
            pyro::scoped_mutex_t lock(_mutex);
            _waiter.task = xTaskGetCurrentTaskHandle();
            _waiter.sem  = my_sem;
        }
        xSemaphoreTake(my_sem, portMAX_DELAY);
        vSemaphoreDelete(my_sem);
    }
    return PYRO_OK;
}

status_t buzzer_drv_t::beep(uint32_t frequency, uint32_t duration_ms) {
    uint16_t n[1] = { (uint16_t)frequency };
    uint32_t d[1] = { duration_ms };
    return submit_play(n, d, 1, true);
}

status_t buzzer_drv_t::beep(uint32_t duration_ms) {
    return beep(BUZZER_DEFAULT_FREQ, duration_ms);
}

status_t buzzer_drv_t::play_note(uint16_t frequency, uint32_t duration_ms) {
    return beep(frequency, duration_ms);
}

status_t buzzer_drv_t::play_note(note_t note, uint8_t octave, uint32_t duration_ms) {
    return beep(note_to_freq(note, octave), duration_ms);
}

status_t buzzer_drv_t::play_melody(const uint16_t notes[], const uint32_t durations[], uint32_t len) {
    return play_melody_blocking(notes, durations, len);
}

status_t buzzer_drv_t::play_melody_blocking(const uint16_t notes[], const uint32_t durations[], uint32_t len) {
    return submit_play(notes, durations, len, true);
}

status_t buzzer_drv_t::play_melody_async(const uint16_t notes[], const uint32_t durations[], uint32_t len) {
    return submit_play(notes, durations, len, false);
}

status_t buzzer_drv_t::play_rhythm(const rhythm_step_t pattern[], uint32_t len) {
    if (len > BUZZER_MAX_MELODY_LEN)
        return PYRO_PARAM_ERROR;
    uint16_t notes[BUZZER_MAX_MELODY_LEN];
    uint32_t durs[BUZZER_MAX_MELODY_LEN];
    for (uint32_t i = 0; i < len; i++) {
        notes[i] = pattern[i].rest ? 0u : note_to_freq(pattern[i].note, pattern[i].octave);
        durs[i]  = (uint32_t)(pattern[i].beats * _beat_ms);
    }
    return submit_play(notes, durs, len, true);
}

// ===== RTTTL 铃声（非阻塞） =====
status_t buzzer_drv_t::play_rtttl(const char* rtttl) {
    if (rtttl == nullptr) return PYRO_PARAM_ERROR;

    // 1) 定位控制区与音符序列（两个 ':' 分隔）
    const char* ctrl  = strchr(rtttl, ':');
    if (!ctrl) return PYRO_PARAM_ERROR;
    const char* notes = strchr(ctrl + 1, ':');
    if (!notes) return PYRO_PARAM_ERROR;
    notes++;                                  // 跳过第二个 ':'

    unsigned dd = 4, oo = 5, bb = 120;
    sscanf(ctrl + 1, "d=%u,o=%u,b=%u", &dd, &oo, &bb);
    if (bb == 0) bb = 120;
    const uint32_t beat_ms = 60000u / bb;

    // 3) 逐音符解析 [时值][音名][#][.]
    uint16_t freq[BUZZER_MAX_MELODY_LEN];
    uint32_t dur[BUZZER_MAX_MELODY_LEN];
    uint32_t n = 0;
    const char* p = notes;
    while (*p && n < BUZZER_MAX_MELODY_LEN) {
        while (*p == ',' || *p == ' ') p++;   // 跳过分隔
        if (!*p) break;

        uint8_t d_i = (uint8_t)dd;            // 时值（可被数字覆盖）
        if (isdigit((unsigned char)*p)) {
            d_i = 0;
            while (isdigit((unsigned char)*p)) { d_i = (uint8_t)(d_i * 10 + (*p - '0')); p++; }
        }
        if (d_i == 0u) d_i = 4u;              // 防护

        char nc = (char)toupper((unsigned char)*p);
        p++;                                  // 跳过音名字母
        if (nc == 'P') { freq[n] = 0u; }      // 休止符：静音计时
        else {
            bool sharp = false;
            if (*p == '#') { sharp = true; p++; }
            freq[n] = note_to_freq(rtttl_note(nc, sharp), (uint8_t)oo);
        }
        bool dot = false;
        if(*p == '.') {
            dot = true;
            p++;
        }

        uint32_t ms = beat_ms * 4u / d_i;
        if(dot) 
            ms = ms * 3u / 2u;
        dur[n] = ms;
        n++;

        while(*p && *p != ',')
            p++;
    }

    return submit_play(freq, dur, n, false);
}

status_t buzzer_drv_t::stop() {
    _state.need_stop = true;
    _task.notify();
    return PYRO_OK;
}

status_t buzzer_drv_t::pause() {
    _state.paused = true;
    _task.notify();
    return PYRO_OK;
}

status_t buzzer_drv_t::resume() {
    _state.paused = false;
    _task.notify();
    return PYRO_OK;
}

void buzzer_drv_t::set_loop(bool enable) {
    _state.loop = enable;
    _task.notify();
}

status_t buzzer_drv_t::start() {
    return _pwm->start();
}

bool buzzer_drv_t::is_playing() const {
    return _playing;
}

// ===== 音量/速度 =====
status_t buzzer_drv_t::set_volume(uint8_t percent)
{
    if (percent > 100u) return PYRO_PARAM_ERROR;
    _volume = percent;
    return _muted ? PYRO_OK : _pwm->set_duty_cycle((float)_volume / 100.0f);
}

uint8_t buzzer_drv_t::get_volume() const { return _volume; }

status_t buzzer_drv_t::mute() {
    _muted = true;
    return _pwm->set_duty_cycle(0.0f);
}

status_t buzzer_drv_t::unmute() {
    _muted = false;
    return _pwm->set_duty_cycle((float)_volume / 100.0f);
}

void buzzer_drv_t::set_tempo(uint16_t bpm) {
    _tempo_bpm = (bpm == 0u) ? 120u : bpm;
    _beat_ms   = 60000 / _tempo_bpm / 4;
}

uint16_t buzzer_drv_t::get_tempo() const {
    return _tempo_bpm;
}

bool buzzer_drv_t::take_pending() {
    pyro::scoped_mutex_t lock(_mutex);
    if (!_state.pending) return false;
    _state.pending = false;
    _run_len = _state.len;
    memcpy(_run_notes, _state.notes, sizeof(_run_notes));
    memcpy(_run_durations, _state.durations, sizeof(_run_durations));
    return true;
}

void buzzer_drv_t::check_pending() {
    if (!take_pending())
        return;
    _idx = 0;
    _playing = true;
    _need_start = true;
    _pwm->stop();
}

void buzzer_drv_t::update_playback() {
    if (!_playing)
        return;
    
    const uint32_t now = xTaskGetTickCount();

    if (_need_start) {
        if (_run_notes[_idx] != 0u) {
            _pwm->set_frequency(_run_notes[_idx]);
            _pwm->start();
        }
        _note_start_tick = now;
        _need_start = false;
        return;
    }

    if (now - _note_start_tick < _run_durations[_idx]) return;

    _pwm->stop();
    if(++_idx >= _run_len) {
        if (_state.loop)
            _idx = 0;
        else {
            _playing = false;
            notify_done();                   // 播完：定向唤醒等待者
            return;
        }
    }
    if(_run_notes[_idx] != 0u) {
        _pwm->set_frequency(_run_notes[_idx]);
        _pwm->start();
    }
    _note_start_tick = now;
}

buzzer_drv_t::player_task_t::player_task_t(buzzer_drv_t* owner)
    : task_base_t("buzzer_task", 256, 512, priority_t::LOW), _owner(owner) {}

void buzzer_drv_t::player_task_t::notify() {
    if (_loop_task_handle != nullptr) xTaskNotifyGive(_loop_task_handle);
}

bool buzzer_drv_t::player_task_t::is_self() const {
    return (_loop_task_handle == xTaskGetCurrentTaskHandle());
}

status_t buzzer_drv_t::player_task_t::init() {
    return PYRO_OK;
}

void buzzer_drv_t::player_task_t::run_loop() {
    while (1) {
        if (_owner->_state.paused) {           // 暂停：完全静音，阻塞等恢复
            _owner->_pwm->stop();
            _owner->_playing = false;
            _owner->_need_start = true;
            if (_owner->_state.need_stop) {    // 暂停中也能响应 stop
                _owner->_state.need_stop = false;
                _owner->notify_done();         // 停止：定向唤醒等待者
            }
            xTaskNotifyWait(0, 0, nullptr, portMAX_DELAY);
            continue;
        }
        if (_owner->_state.need_stop) {        // 停止请求
            _owner->_state.need_stop = false;
            _owner->_pwm->stop();
            _owner->_playing = false;
            _owner->notify_done();             // 停止：定向唤醒等待者
        }
        _owner->check_pending();               // 有新播放内容则覆盖播放
        _owner->update_playback();             // 逐音符推进
        xTaskNotifyWait(0, 0, nullptr, 10);    // 等通知或 10ms 超时
    }
}

} // namespace pyro