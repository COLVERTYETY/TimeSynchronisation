/*! ----------------------------------------------------------------------------
 * @file    deca_mutex.c
 * @brief   IRQ interface / mutex implementation
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 */

#include <dw3000_device_api.h>
#include <dw3000_port.h>
#include <driver/mcpwm_cap.h>
// ---------------------------------------------------------------------------
//
// NB: The purpose of this file is to provide for microprocessor interrupt enable/disable, this is used for
//     controlling mutual exclusion from critical sections in the code where interrupts and background
//     processing may interact.  The code using this is kept to a minimum and the disabling time is also
//     kept to a minimum, so blanket interrupt disable may be the easiest way to provide this.  But at a
//     minimum those interrupts coming from the decawave device should be disabled/re-enabled by this activity.
//
//     In porting this to a particular microprocessor, the implementer may choose to use #defines in the
//     deca_irq.h include file to map these calls transparently to the target system.  Alternatively the
//     appropriate code may be embedded in the functions provided below.
//
//     This mutex dependent on HW port.
//	   If HW port uses EXT_IRQ line to receive ready/busy status from DW1000 then mutex should use this signal
//     If HW port not use EXT_IRQ line (i.e. SW polling) then no necessary for decamutex(on/off)
//
//	   For critical section use this mutex instead
//	   __save_intstate()
//     __restore_intstate()
// ---------------------------------------------------------------------------


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexon()
 *
 * Description: This function should disable interrupts. This is called at the start of a critical section
 * It returns the irq state before disable, this value is used to re-enable in decamutexoff call
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
// portMUX_TYPE deca_mutex = portMUX_INITIALIZER_UNLOCKED;
// Track whether DW3000 interrupt is currently enabled.
extern volatile bool irq_enabled;
extern mcpwm_cap_channel_handle_t capture_channel;
extern uint8_t _irq;

// decamutexon() - returns previous IRQ state
decaIrqStatus_t decamutexon(void)
{
    
    // portENTER_CRITICAL(&deca_mutex);
    decaIrqStatus_t state = irq_enabled;  // Remember current state
    if (irq_enabled) {
        // Disable DW3000 interrupt line
        // gpio_intr_disable(static_cast<gpio_num_t>(_irq));
        mcpwm_capture_channel_disable(capture_channel);
        irq_enabled = false;
    }
    // portEXIT_CRITICAL(&deca_mutex);


    return state;  // Return the old state
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: decamutexoff()
 *
 * Description: This function should re-enable interrupts, or at least restore their state as returned(&saved) by decamutexon
 * This is called at the end of a critical section
 *
 * Note: The body of this function is defined in deca_mutex.c and is platform specific
 *
 * input parameters:
 * @param s - the state of the DW1000 interrupt as returned by decamutexon
 *
 * output parameters
 *
 * returns the state of the DW1000 interrupt
 */
// decamutexoff() - restore state
void decamutexoff(decaIrqStatus_t state)
{
    // Only re‐enable if it was previously enabled
    // (i.e. decamutexon() had to turn it off)
    if (state) {
        // portENTER_CRITICAL(&deca_mutex);
        if (!irq_enabled) {
            // gpio_intr_enable((gpio_num_t)_irq);
            mcpwm_capture_channel_enable(capture_channel);
            irq_enabled = true;
        }
        // portEXIT_CRITICAL(&deca_mutex);
    }
}
