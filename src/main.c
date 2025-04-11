/*
 * Copyright (c) 2018 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <lvgl_input_device.h>
#include <zephyr/sys/printk.h>


#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif

#include "adcconfig.h"



// ADC Config

const nrfx_timer_t timer_instance = NRFX_TIMER_INSTANCE(TIMER_INSTANCE_NUMBER);
static int16_t saadc_sample_buffer[2][SAADC_BUFFER_SIZE];
static uint32_t saadc_current_buffer = 0;

uint32_t adcval=0;

static lv_style_t style_indic;

static void configure_timer(void)
{
    nrfx_err_t err;

    /* STEP 3.3 - Declaring timer config and intialize nrfx_timer instance. */
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(1000000);
    err = nrfx_timer_init(&timer_instance, &timer_config, NULL);
    if (err != NRFX_SUCCESS) {
        printk("nrfx_timer_init error: %08x", err);
        return;
    }
    uint32_t timer_ticks = nrfx_timer_us_to_ticks(&timer_instance, SAADC_SAMPLE_INTERVAL_US);
    nrfx_timer_extended_compare(&timer_instance, NRF_TIMER_CC_CHANNEL0, timer_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

}

static void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    nrfx_err_t err;
    switch (p_event->type)
    {
        case NRFX_SAADC_EVT_READY:
        
           /* STEP 5.1 - Buffer is ready, timer (and sampling) can be started. */
           nrfx_timer_enable(&timer_instance);

            break;                        
            
        case NRFX_SAADC_EVT_BUF_REQ:
        
            /* STEP 5.2 - Set up the next available buffer. Alternate between buffer 0 and 1 */
            err = nrfx_saadc_buffer_set(saadc_sample_buffer[(saadc_current_buffer++)%2], SAADC_BUFFER_SIZE);
            if (err != NRFX_SUCCESS) {
                printk("nrfx_saadc_buffer_set error: %08x", err);
                return;
            }

            break;

        case NRFX_SAADC_EVT_DONE:

            /* STEP 5.3 - Buffer has been filled. Do something with the data and proceed */
            int64_t average = 0;
            int16_t max = INT16_MIN;
            int16_t min = INT16_MAX;
            int16_t current_value;
            for (int i = 0; i < p_event->data.done.size; i++) {
                current_value = ((int16_t *)(p_event->data.done.p_buffer))[i];
                adcval=current_value;
                /*
                average += current_value;
                if (current_value > max) {
                    max = current_value;
                }
                if (current_value < min) {
                    min = current_value;
                } */
            }
            /*
            average = average / p_event->data.done.size;
            printk("SAADC buffer at 0x%x filled with %d samples\r\n", (uint32_t)p_event->data.done.p_buffer,
                p_event->data.done.size);
            printk("AVG=%d, MIN=%d, MAX=%d\r\n", (int16_t)average, min, max);   */
            break;

        default:
            printk("Unhandled SAADC evt %d", p_event->type);
            break;
    }
}


static void configure_saadc(void)
{
    nrfx_err_t err;

    /* STEP 4.4 - Connect ADC interrupt to nrfx interrupt handler */
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
    DT_IRQ(DT_NODELABEL(adc), priority),
    nrfx_isr, nrfx_saadc_irq_handler, 0);

    
    /* STEP 4.5 - Initialize the nrfx_SAADC driver */
    err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        printk("nrfx_saadc_init error: %08x", err);
        return;
    }
    
    /* STEP 4.6 - Declare the struct to hold the configuration for the SAADC channel used to sample the battery voltage */
    #define SAADC_INPUT_PIN NRF_SAADC_INPUT_AIN0
    static nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(SAADC_INPUT_PIN, 0);

    /* STEP 4.7 - Change gain config in default config and apply channel configuration */
    channel.channel_config.gain = NRF_SAADC_GAIN1_6;
    err = nrfx_saadc_channels_config(&channel, 1);
    if (err != NRFX_SUCCESS) {
        printk("nrfx_saadc_channels_config error: %08x", err);
        return;
    }

    /* STEP 4.8 - Configure channel 0 in advanced mode with event handler (non-blocking mode) */
    nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    err = nrfx_saadc_advanced_mode_set(BIT(0),
                                        NRF_SAADC_RESOLUTION_12BIT,
                                        &saadc_adv_config,
                                        saadc_event_handler);
    if (err != NRFX_SUCCESS) {
        printk("nrfx_saadc_advanced_mode_set error: %08x", err);
        return;
    }
                                            
    /* STEP 4.9 - Configure two buffers to make use of double-buffering feature of SAADC */
    err = nrfx_saadc_buffer_set(saadc_sample_buffer[0], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS) {
        printk("nrfx_saadc_buffer_set error: %08x", err);
        return;
    }
    err = nrfx_saadc_buffer_set(saadc_sample_buffer[1], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS) {
        printk("nrfx_saadc_buffer_set error: %08x", err);
        return;
    }

    /* STEP 4.10 - Trigger the SAADC. This will not start sampling, but will prepare buffer for sampling triggered through PPI */
    err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        printk("nrfx_saadc_mode_trigger error: %08x", err);
        return;
    }

}

static void configure_ppi(void)
{
    nrfx_err_t err;
    /* STEP 6.1 - Declare variables used to hold the (D)PPI channel number */
    uint8_t m_saadc_sample_ppi_channel;
    uint8_t m_saadc_start_ppi_channel;

    /* STEP 6.2 - Trigger task sample from timer */
    err = nrfx_gppi_channel_alloc(&m_saadc_sample_ppi_channel);
    if (err != NRFX_SUCCESS) {
        printk("nrfx_gppi_channel_alloc error: %08x", err);
        return;
    }
    
    err = nrfx_gppi_channel_alloc(&m_saadc_start_ppi_channel);
    if (err != NRFX_SUCCESS) {
        printk("nrfx_gppi_channel_alloc error: %08x", err);
        return;
    }


    /* STEP 6.3 - Trigger task sample from timer */
    nrfx_gppi_channel_endpoints_setup(m_saadc_sample_ppi_channel,
        nrfx_timer_compare_event_address_get(&timer_instance,
                             NRF_TIMER_CC_CHANNEL0),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));

    /* STEP 6.4 - Trigger task start from end event */
    nrfx_gppi_channel_endpoints_setup(m_saadc_start_ppi_channel,
        nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

    /* STEP 6.5 - Enable both (D)PPI channels */ 
    nrfx_gppi_channels_enable(BIT(m_saadc_sample_ppi_channel));
    nrfx_gppi_channels_enable(BIT(m_saadc_start_ppi_channel));
}





#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

#define LV_TICK_CUSTOM 0

static uint32_t count;

#ifdef CONFIG_GPIO
static struct gpio_dt_spec button_gpio = GPIO_DT_SPEC_GET_OR(
		DT_ALIAS(sw0), gpios, {0});
static struct gpio_callback button_callback;

static void button_isr_callback(const struct device *port,
				struct gpio_callback *cb,
				uint32_t pins)
{
    static bool color = true;
    printk("Button pressed!\r\n");
    if (color) lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_RED));
    else lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_DEEP_PURPLE));
    color = !color;
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	count = 0;
}
#endif /* CONFIG_GPIO */

#ifdef CONFIG_LV_Z_ENCODER_INPUT
static const struct device *lvgl_encoder =
	DEVICE_DT_GET(DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_lvgl_encoder_input));
#endif /* CONFIG_LV_Z_ENCODER_INPUT */

static void lv_btn_click_callback(lv_event_t *e)
{
	ARG_UNUSED(e);

	count = 0;
}






int main(void)
{

	configure_timer();
    configure_saadc();  
    configure_ppi();

	
	lv_init();
	char count_str[11] = {0};
	const struct device *display_dev;
	lv_obj_t *hello_world_label;
	lv_obj_t *count_label;
	lv_obj_t * bar1;

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		printk("Device not ready, aborting test");
		return 0;
	}
	else printk("Device DT GET pass");

#ifdef CONFIG_GPIO
	if (gpio_is_ready_dt(&button_gpio)) {
		int err;

		err = gpio_pin_configure_dt(&button_gpio, GPIO_INPUT);
		if (err) {
			printk("failed to configure button gpio: %d", err);
			return 0;
		}
		else printk("Button set");

		gpio_init_callback(&button_callback, button_isr_callback,
				   BIT(button_gpio.pin));

		err = gpio_add_callback(button_gpio.port, &button_callback);
		if (err) {
			printk("failed to add button callback: %d", err);
			return 0;
		}
		else printk("button callback pass");

		err = gpio_pin_interrupt_configure_dt(&button_gpio,
						      GPIO_INT_EDGE_TO_ACTIVE);
		if (err) {
			printk("failed to enable button callback: %d", err);
			return 0;
		}
		else printk("button enable");
	}
#endif /* CONFIG_GPIO */

#ifdef CONFIG_LV_Z_ENCODER_INPUT
	lv_obj_t *arc;
	lv_group_t *arc_group;

	arc = lv_arc_create(lv_scr_act());
	lv_obj_align(arc, LV_ALIGN_CENTER, 0, 0);
	lv_obj_set_size(arc, 150, 150);

	arc_group = lv_group_create();
	lv_group_add_obj(arc_group, arc);
	lv_indev_set_group(lvgl_input_get_indev(lvgl_encoder), arc_group);
#endif /* CONFIG_LV_Z_ENCODER_INPUT */

	if (IS_ENABLED(CONFIG_LV_Z_POINTER_KSCAN) || IS_ENABLED(CONFIG_LV_Z_POINTER_INPUT)) {
		lv_obj_t *hello_world_button;

		hello_world_button = lv_btn_create(lv_scr_act());
		lv_obj_align(hello_world_button, LV_ALIGN_CENTER, 0, 0);
		lv_obj_add_event_cb(hello_world_button, lv_btn_click_callback, LV_EVENT_CLICKED,
						NULL);
		hello_world_label = lv_label_create(hello_world_button);
	} else {
		hello_world_label = lv_label_create(lv_scr_act());
	}

	lv_label_set_text(hello_world_label, "FFT VISUALIZER");
	lv_obj_align(hello_world_label, LV_ALIGN_TOP_LEFT, 0, 0);
	printk("FFT visulizer set\r\n");

	/*bar1 = lv_bar_create(lv_scr_act());
	lv_obj_set_size(bar1, 100, 10);
	lv_obj_align(bar1, LV_ALIGN_TOP_MID, 0, -30);
	lv_bar_set_value(bar1, 70, LV_ANIM_OFF); */


    lv_style_init(&style_indic);
    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
    lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_DEEP_PURPLE));
    //lv_style_set_bg_grad_color(&style_indic, lv_palette_main(LV_PALETTE_BLUE));
    //lv_style_set_bg_grad_dir(&style_indic, LV_GRAD_DIR_VER);


	bar1 = lv_bar_create(lv_scr_act());
	lv_obj_add_style(bar1, &style_indic, LV_PART_INDICATOR);
    lv_obj_set_size(bar1, 10, 150);
    lv_bar_set_range(bar1, 0, 3000);
    lv_bar_set_value(bar1, 0, LV_ANIM_OFF);
    lv_obj_align(bar1, LV_ALIGN_CENTER, 0, 30);


	lv_task_handler();
	display_blanking_off(display_dev);
	uint8_t count2=0;
	while (1) {
		lv_bar_set_value(bar1, adcval, LV_ANIM_OFF);
		lv_task_handler();
		if (count2<=100)count2++;
		else count2 = 0;
		k_sleep(K_MSEC(1));
	}
}
