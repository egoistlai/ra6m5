/***********************************************************************************************************************
 * Includes
 **********************************************************************************************************************/
#include "drv_gpio_ec11.h"

/**********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
#define DRV_GPIO_EC11_USE_LVGL  (1)

#if DRV_GPIO_EC11_USE_LVGL == 1
#include "lvgl.h"
#endif

/**********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private function prototypes
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Private global variables
 **********************************************************************************************************************/
static uint32_t ec11_key_press_tick = 0;

static bool ec11_key_state = false;


/***********************************************************************************************************************
 * Functions
 **********************************************************************************************************************/

fsp_err_t drv_gpio_ec11_init(void)
{
    fsp_err_t err;

    // external irq 1
    err = g_external_irq1_key.p_api->open(g_external_irq1_key.p_ctrl,
                                          g_external_irq1_key.p_cfg);
    if(FSP_SUCCESS != err)
    {
        return err;
    }
    err = g_external_irq1_key.p_api->enable(g_external_irq1_key.p_ctrl);
    if(FSP_SUCCESS != err)
    {
        return err;
    }

    return err;
}

uint32_t drv_uart_get_pin_press_tick(bsp_io_port_pin_t pin)
{
    uint32_t pin_press_tick = 0;

    switch(pin){
        case EC11_PIN_KEY:
            pin_press_tick = ec11_key_press_tick;
            break;
        default:
            pin_press_tick = 0;
            break;
    }

    return pin_press_tick;
}

bool drv_uart_get_pin_state(bsp_io_port_pin_t pin)
{
    return ec11_key_state;
}


void drv_uart_set_pin_state(bsp_io_port_pin_t pin, bool state)
{
    ec11_key_state = state;
}


void external_irq1_key_callback(external_irq_callback_args_t * p_args)
{
    if(p_args->channel == 1)
    {
        ec11_key_state = true;
#if DRV_GPIO_EC11_USE_LVGL == 0
        ec11_key_press_tick = 20;
#else
        ec11_key_press_tick = lv_tick_get() + 20;;
#endif
    }
}


/***********************************************************************************************************************
 * Private Functions
 **********************************************************************************************************************/
