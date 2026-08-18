#ifndef __PYRO_PWM_DRV_H__
#define __PYRO_PWM_DRV_H__

#include "pyro_core_def.h"
#include "tim.h"
#include <cstdint>

namespace pyro {
    
constexpr uint32_t TIM_CLK_HZ{240000000};

class pwm_drv_t {
    friend class bsp_pwm;
    
  public:
    pwm_drv_t() = delete;
    pwm_drv_t(const pwm_drv_t&) = delete;
    pwm_drv_t& operator=(const pwm_drv_t&) = delete;

    status_t init();
    status_t deinit();
    status_t start();
    status_t stop();
    bool is_running() const;

    status_t set_frequency(uint32_t f);
    uint32_t get_frequency() const;
    status_t set_duty_cycle(float duty);
    float get_duty_cycle() const;

  private:
    explicit pwm_drv_t(TIM_HandleTypeDef *htim, uint32_t channel);

    status_t update();

    TIM_HandleTypeDef *_htim;
    uint32_t _channel;
    uint32_t _psc{60}, _arr{1000};
    uint32_t _cmp{500};

    bool _running;
    uint32_t _freq{4000};
    float _duty{0.5};
};

} // namespace pyro

#endif // __PYRO_PWM_DRV_H__