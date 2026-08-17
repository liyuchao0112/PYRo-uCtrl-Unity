#ifndef __PYRO_BSP_PWM_H__
#define __PYRO_BSP_PWM_H__

#include "pyro_pwm_drv.h"

namespace pyro {

class bsp_pwm {
  public:
    enum which_pwm {
        pwm_tim12_ch2
    };

    static pwm_drv_t& get_tim3_ch4();

    static pwm_drv_t* get_pwm(which_pwm which);

    static status_t init_all();       // 触发各实例首次构造并 init
};

} // namespace pyro

#endif // __PYRO_BSP_PWM_H__