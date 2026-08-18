#include "pyro_pwm_drv.h"

namespace pyro {

status_t pwm_drv_t::init() {
    __HAL_TIM_SET_PRESCALER(_htim, _psc - 1);
    __HAL_TIM_SET_AUTORELOAD(_htim, _arr - 1);
    __HAL_TIM_SET_COMPARE(_htim, _channel, _cmp);
    return PYRO_OK;
}

status_t pwm_drv_t::deinit() {
    HAL_TIM_PWM_Stop(_htim, _channel);
    _running = false;
    return PYRO_OK;
}

status_t pwm_drv_t::start() {
    HAL_TIM_PWM_Start(_htim, _channel);
    _running = true;
    return PYRO_OK;
}

status_t pwm_drv_t::stop() {
    HAL_TIM_PWM_Stop(_htim, _channel);
    _running = false;
    return PYRO_OK;
}

bool pwm_drv_t::is_running() const {
    return _running;
}

status_t pwm_drv_t::set_frequency(uint32_t f) {
    if(f == 0 || f > TIM_CLK_HZ)
        return PYRO_PARAM_ERROR;
    _freq = f;
    update();
    return PYRO_OK;
}

uint32_t pwm_drv_t::get_frequency() const {
    return _freq;
}

status_t pwm_drv_t::set_duty_cycle(float duty) {
    if(duty < 0 || duty > 1)
        return PYRO_PARAM_ERROR;
    _duty = duty;
    update();
    return PYRO_OK;
}

float pwm_drv_t::get_duty_cycle() const {
    return _duty;
}

pwm_drv_t::pwm_drv_t(TIM_HandleTypeDef *htim, uint32_t channel)
    : _htim(htim), _channel(channel), _freq(4000), _duty(0.5){}

status_t pwm_drv_t::update() {
    _arr = TIM_CLK_HZ / _psc / _freq;
    _cmp = _duty * _arr;
    __HAL_TIM_SET_AUTORELOAD(_htim, _arr - 1);
    __HAL_TIM_SET_COMPARE(_htim, _channel, _cmp);
    return PYRO_OK;
}

} // namespace pyro