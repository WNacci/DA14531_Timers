#include <da1458x_config_basic.h>
#include <da1458x_config_advanced.h>
#include <user_config.h>
#include <rwip_config.h> // SW configuration

#include <user_app.h>
#include <custs1.h>
#include <custs1_task.h>
#include <gpio.h>
#include <user_custs1_def.h>
#include <user_periph_setup.h>
#include <ble_handlers.h>
#include <debug.h>

//Timer Section#################
#include "app_api.h"
#include "arch_system.h"
#include "timer1.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
 
// Timer1 counting configuration structure
static timer1_count_options_t timer1_config =
{	
	/*Timer1 input clock*/
	.input_clk 	= 	(tim1_clk_src_t)INPUT_CLK,
	
	/*Timer1 free run off*/
	.free_run 	= 	FREE_RUN,
	
	/*Timer1 IRQ mask*/
	.irq_mask	= 	INTERRUPT_MASK_TMR,
	
	/*Timer1 count direction*/
	.count_dir	= 	COUNT_DIRECTION,
	
	/*Timer1 reload value*/
	.reload_val	= 	RELOAD_VALUE
};



/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
// Retained variables

uint8_t timer1_cnt_ovf                          __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
bool led_state                                  __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY
//extern rcx_time_data_t rcx_time_data;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/


static void timer1_overflow(void);

//void user_on_connection(uint8_t connection_idx, struct gapc_connection_req_ind const *param)
//{
//    default_app_on_connection(connection_idx, param);
//}

//void user_on_disconnect( struct gapc_disconnect_ind const *param )
//{
//    default_app_on_disconnect(param);
//}
//
//void user_app_adv_start(void)
//{
//
//    default_advertise_operation();  
//}

void timer1_initialize(void)
{
    /*Enable PD_TIM power domain*/
	SetBits16(PMU_CTRL_REG, TIM_SLEEP, 0);                  // Enable the PD_TIM
    timer1_count_config(&timer1_config, timer1_overflow);   // Set the capture counting configurations for the timer

    printf("Timer 1 is initialized \n\r") ;
}

static void user_timer_wakeup_ble(void)
{
    printf("BLE is awake via timer 1\n\r");
}

static void toggle_led(void)
{
    if(led_state)
    {
        GPIO_SetInactive(GPIO_LED_PORT, GPIO_LED_PIN);
        led_state = false;
    }
    else
    {
        GPIO_SetActive(GPIO_LED_PORT, GPIO_LED_PIN);
        led_state = true;
    }
}

static void timer1_overflow(void)
{
    if (timer1_cnt_ovf == TIMER1_OVF_COUNTER)
    {
        toggle_led();
        
        timer1_cnt_ovf = 0;
        /* Wake up the device and print out the measured time */
        arch_ble_force_wakeup();                // Force the BLE to wake up
        app_easy_wakeup_set(user_timer_wakeup_ble);
        app_easy_wakeup();                      // Send the message to print
    }
    
    timer1_cnt_ovf++;
}

//##############################

/*
This function is called as a callback by system arch, after peripheral init, and app init, but before giving away to the kernel and main loop
*/
void app_on_init(void)
{
    // To keep compatibility call default handler
    default_app_on_init();

    extern uint32_t __StackTop;
    extern uint32_t __HeapBase;
    extern uint32_t __HeapLimit;

    uint32_t initial_sp = (uint32_t)&__StackTop;
    uint32_t heap_base = (uint32_t)&__HeapBase;
    uint32_t heap_limit = (uint32_t)&__HeapLimit;

    printf("app_on_init()\r\n");
    printf("Compiled: %s %s\r\n", __DATE__, __TIME__);
    printf("stack: 0x%08lX\r\n", initial_sp);
    printf("heap: 0x%08lX (0x%04X)\r\n", heap_base, (uint16_t)heap_limit);
	printf("Advertized Name: %s\n",USER_DEVICE_NAME);

	//Timer stuff
    timer1_initialize();
	timer1_clear_all_events();
    timer1_enable_irq();
    timer1_start();
}

void app_resume_from_sleep(void)
{
}

arch_main_loop_callback_ret_t app_on_system_powered(void)
{
    wdg_reload(1);
    return KEEP_POWERED;
}

sleep_mode_t app_validate_sleep(sleep_mode_t sleep_mode)
{
    /* Block sleep. */
    return mode_active;
}

void app_going_to_sleep(sleep_mode_t sleep_mode)
{
}
