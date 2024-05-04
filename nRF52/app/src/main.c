#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_gpio.h>

#include "components/bluetooth/bluetooth_core.h"
#include "uart/uart_stm.h"

LOG_MODULE_REGISTER(o_invader_main, LOG_LEVEL_DBG);

#define LED_1 NRF_GPIO_PIN_MAP(0,5)
#define LED_2 NRF_GPIO_PIN_MAP(0,4)

#define SLEEP_TIME_MS 1000

int main(void)
{
//    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
//    NRF_TIMER1->TASKS_CLEAR = 1;
//    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
//    NRF_TIMER1->TASKS_START = 1;
//    NRF_CLOCK->TASKS_HFCLKSTART = 1;
//    int counter = 1000000;
//    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0 && counter-- > 0);
////    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
//    NRF_TIMER1->TASKS_CAPTURE[0] = 1;
//
//    printk("HF Clock has started. Startup time: %d uS\n", NRF_TIMER1->CC[0]);
//    printk("All up and running!\n");
//
//
//
    LOG_INF("Starting up...\n");
    bt_core_initialize();

    nrf_gpio_cfg_output(LED_1);
    nrf_gpio_cfg_output(LED_2);

    uart_stm_init();

    LOG_INF("All up and running!\n");


    while(1) {
//        nrf_gpio_pin_toggle(LED_1);
//        nrf_gpio_pin_toggle(LED_2);
////        LOG_INF("TOGGLED\n");
        k_sleep(K_MSEC(SLEEP_TIME_MS));
    }

}