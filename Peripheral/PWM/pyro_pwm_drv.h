#ifndef __PYRO_PWM_DRV_H__
#define __PYRO_PWM_DRV_H__

#include "pyro_core_def.h"
#include "tim.h"

namespace pyro {

class pwm_drv_t {
    friend class bsp_pwm;
  public:
    status_t init();
    status_t deinit();
    status_t start();
    status_t stop();
    bool is_running() const;

    status_t set_frequency(uint32_t f);
    uint32_t get_frequency() const;
    status_t set_duty_cycle(uint8_t duty);
    uint8_t get_duty_cycle() const;

  private:
    explicit pwm_drv_t()
};

} // namespace pyro

#endif