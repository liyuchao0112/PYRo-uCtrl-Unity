#include "pyro_bsp_pwm.h"
#include "tim.h"
#include "pyro_core_def.h"

namespace pyro {

pwm_drv_t& bsp_pwm::get_tim12_ch2() {
    static pwm_drv_t instance(&htim12, TIM_CHANNEL_2);
    return instance;
}

pwm_drv_t* bsp_pwm::get_pwm(which_pwm which) {
    switch(which) {
      case pwm_tim12_ch2:
        return &get_tim12_ch2();
      default:
        return nullptr;
    }
}

status_t bsp_pwm::init_all() {
    CHECK_PYRO_RET(get_tim12_ch2().init());
    return PYRO_OK;
}

} // namespace pyro