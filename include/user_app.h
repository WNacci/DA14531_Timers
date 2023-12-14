#ifndef USER_APP_H_
#define USER_APP_H_

#include <stdint.h>

#include <arch_api.h>
#include <ke_msg.h>
#include <arch.h>

void app_on_init(void);
void app_resume_from_sleep(void);
arch_main_loop_callback_ret_t app_on_system_powered(void);
sleep_mode_t app_validate_sleep(sleep_mode_t sleep_mode);
void app_going_to_sleep(sleep_mode_t sleep_mode);


//####Timer stuff##############
//LED
#define GPIO_LED_PORT           GPIO_PORT_0
#define GPIO_LED_PIN            GPIO_PIN_9

//Config

#define TIMER1_ON_SYSCLK                1
#define TIMER1_ON_LPCLK                 0
/* Define settings for Timer 0 */
#define TIMER0_DIV_BY_1                 0
#define TIMER0_DIV_BY_2                 1
#define TIMER0_DIV_BY_4                 2
#define TIMER0_DIV_BY_8                 3

/* Generic timer settings */
#define FREE_RUN                        TIM1_FREE_RUN_ON    
#define COUNT_DIRECTION                 TIM1_CNT_DIR_UP
#define RELOAD_VALUE                    TIM1_RELOAD_MAX

/* Generic event Settings */
#define STAMP_TYPE                      TIM1_EVENT_STAMP_CNT        
#define EVENT_PIN                       (tim1_event_gpio_pin_t)(GPIO_SIGNAL_PIN + 1)

/* Define settings for Timer 1 */
#define TIMER1_ON_SYSCLK                1
#define TIMER1_ON_LPCLK                 0

#define INPUT_CLK               TIMER1_ON_LPCLK
#define INTERRUPT_MASK_TMR      TIM1_IRQ_MASK_OFF
#define TIMER1_OVF_COUNTER      7                   // Using the LP clock, timer1 overflows at around 140ms * 7 is approximatelly 1
//####################################
/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwble_config.h"
#include "app_task.h"                  // application task
#include "gapc_task.h"                 // gap functions and messages
#include "gapm_task.h"                 // gap functions and messages
#include "app.h"                       // application definitions
#include "co_error.h"                  // error code definitions
#include "arch_api.h"
#include "app.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
 enum pulse_meas_sm_state{
     HIGH_WAIT_LOW_ST,
     LOW_WAIT_HIGH_ST,
 };
 
 enum pulse_meas_sm_events{
     RISING_EDGE_EVT,
     FALLING_EDGE_EVT,
 };
 
 struct freq_meas_ready{
    uint16_t captrure_val;
};
 
enum
{
  FREQ_MEAS_READY_MSG = APP_MSG_UTIL_API_LAST_MES + 1,
};


/*
 * DEFINES
 ****************************************************************************************
 */
 
#define DEBUG_CURSORS

#if defined (DEBUG_CURSORS)

#define RISING_EDGE                     GPIO_PIN_6
#define FALLING_EDGE                    GPIO_PIN_7
#define TIMER_OVFL                      GPIO_PIN_9
#define REPORT_TIME                     GPIO_PIN_8

#define TOGGLE_CURSOR(pin)              SetWord16(PAD_LATCH_REG, 1);                    \
                                        SetWord16(P00_MODE_REG + (pin << 1), 0x300);    \
                                        SetWord16(P0_SET_DATA_REG, 1 << pin);           \
                                        SetWord16(P0_RESET_DATA_REG, 1 << pin);
#else
#define TOGGLE_CURSOR(pin)
#endif
 
/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
 
 /**
 ****************************************************************************************
 * @brief Get the current value of the timer
 * @return the current value of the timer
 ****************************************************************************************
 */
__STATIC_FORCEINLINE uint16_t user_current_timer1_value(void)
{
    return GetBits32(TIMER1_STATUS_REG, TIMER1_TIMER_VALUE);
}

/**
 ****************************************************************************************
 * @brief Get the captured value of the event 1 edge
 * @return the value of the timer when a falling edge occurred
 ****************************************************************************************
 */
__STATIC_FORCEINLINE uint16_t timer1_get_event_ch1_capture_value(void)
{
    return GetBits32(TIMER1_CAPCNT1_VALUE_REG, TIMER1_CAPCNT1_VALUE);
}

/**
 ****************************************************************************************
 * @brief Get the captured value of the event 2 edge
 * @return the value of the timer when a rising edge occurred
 ****************************************************************************************
 */
__STATIC_FORCEINLINE uint16_t timer1_get_event_ch2_capture_value(void)
{
    return GetBits32(TIMER1_CAPCNT2_VALUE_REG, TIMER1_CAPCNT2_VALUE);
}

/**
 ****************************************************************************************
 * @brief Enable/Disable the timer 1 overflow interrupt
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void timer1_enable_overflow_intr(bool enable)
{
    if (enable) {
        SetBits32(TIMER1_CTRL_REG, TIMER1_IRQ_EN, 1);
    }
    else
    {
        SetBits32(TIMER1_CTRL_REG, TIMER1_IRQ_EN, 0);
        // Clear a pending timer event
        SetWord16(TIMER1_CLR_EVENT_REG, TIMER1_CLR_TIMER_EVENT);
    }
}

/**
 ****************************************************************************************
 * @brief Clear all timer 1 events
 ****************************************************************************************
 */
__STATIC_FORCEINLINE void timer1_clear_all_events(void)
{
    SetWord16(TIMER1_CLR_EVENT_REG, TIMER1_CLR_IN1_EVENT | TIMER1_CLR_IN2_EVENT | TIMER1_CLR_TIMER_EVENT);
}
 
 /**
 ****************************************************************************************
 * @brief Application initialization function.
 ****************************************************************************************
*/
void user_app_init(void);

/**
 ****************************************************************************************
 * @brief Connection function.
 * @param[in] connection_idx Connection Id index
 * @param[in] param Pointer to GAPC_CONNECTION_REQ_IND message
 ****************************************************************************************
*/
void user_on_connection(uint8_t connection_idx, struct gapc_connection_req_ind const *param);

/**
 ****************************************************************************************
 * @brief Disconnection function.
 * @param[in] param Pointer to GAPC_DISCONNECT_IND message
 ****************************************************************************************
*/
void user_on_disconnect( struct gapc_disconnect_ind const *param );

/**
 ****************************************************************************************
 * @brief User start advertising function.
 ****************************************************************************************
*/
void user_app_adv_start(void);

/**
 ****************************************************************************************
 * @brief Handles the messages that are not handled by the SDK internal mechanisms.
 * @param[in] msgid   Id of the message received.
 * @param[in] param   Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id  ID of the sending task instance.
 * @return void
 ****************************************************************************************
*/
void user_catch_rest_hndl(ke_msg_id_t const msgid,
                          void const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id);

/**
 ****************************************************************************************
 * @brief Initializes the timer 1 counting
 * @details Enabled the Timer domain and configures the timer
 ****************************************************************************************
*/
void timer1_initialize(void);

/**
 ****************************************************************************************
 * @brief Initializes the pulse measure functionality
 * @details It starts the timer 1 and sets the rising and falling events to the
 *          corresponding channel.
 ****************************************************************************************
*/
#if (ENABLE_PULSE_MEASURING)
void initialize_pulse_length_measure(void);
#endif

/**
 ****************************************************************************************
 * @brief Initializes the frequency counting functionality
 * @details It starts the timer 1 and sets event for the frequency measurement.
 ****************************************************************************************
*/
#if (ENABLE_FREQ_COUNTING)
void start_frequency_counting(void);

void stop_frequency_counting(void);
#endif
//###############

#endif // USER_APP_H_
