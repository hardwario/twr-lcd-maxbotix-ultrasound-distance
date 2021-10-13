#include <application.h>

/*

Maxbotix connection

Sensor Module pin       Maxbotix pin
-------------------------------------
A (pulse in)            2 (pulse out)
GND                     GND
VCC                     V+

*/


// LED instance
twr_led_t led;

// Button instance
twr_button_t button;

twr_gfx_t *gfx;

bool sleep = false;

#define GRAPH_WIDTH_TIME (10 * 1000)
#define DISTANCE_MEASURE_PERIOD (200)

TWR_DATA_STREAM_FLOAT_BUFFER(distance_stream_buffer, (GRAPH_WIDTH_TIME / DISTANCE_MEASURE_PERIOD))
twr_data_stream_t distance_stream;

void button_event_handler(twr_button_t *self, twr_button_event_t event, void *event_param)
{
    if (event == TWR_BUTTON_EVENT_PRESS)
    {
        twr_led_pulse(&led, 100);

        sleep = !sleep;

        if (sleep)
        {
            twr_module_sensor_set_vdd(false);
        }
        else
        {
            twr_module_sensor_set_vdd(true);
            twr_scheduler_plan_now(0);
        }
    }
}

void application_init(void)
{
    // Initialize logging
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);

    // Initialize LED
    twr_led_init(&led, TWR_GPIO_LED, false, false);
    twr_led_set_mode(&led, TWR_LED_MODE_ON);

    twr_module_lcd_init();
    gfx = twr_module_lcd_get_gfx();

    // Initialize button
    twr_button_init(&button, TWR_GPIO_BUTTON, TWR_GPIO_PULL_DOWN, false);
    twr_button_set_event_handler(&button, button_event_handler, NULL);

    twr_module_sensor_init();
    twr_module_sensor_set_vdd(true);

    twr_gpio_init(TWR_GPIO_P4);
    twr_gpio_set_mode(TWR_GPIO_P4, TWR_GPIO_MODE_INPUT);

    twr_data_stream_init(&distance_stream, 1, &distance_stream_buffer);
}

void graph(twr_gfx_t *gfx, int x0, int y0, int x1, int y1, twr_data_stream_t *data_stream, int time_step, const char *format)
{
    int w, h;
    char str[32];
    int width = x1 - x0;
    int height = y1 - y0;
    float max_value = 0;
    float min_value = 0;

    twr_data_stream_get_max(data_stream, &max_value);

    twr_data_stream_get_min(data_stream, &min_value);

    if (min_value > 0)
    {
        min_value = 0;
    }

    max_value = ceilf(max_value / 5) * 5;

    twr_module_lcd_set_font(&twr_font_ubuntu_11);

    int number_of_samples = twr_data_stream_get_number_of_samples(data_stream);

    int end_time = - number_of_samples * time_step / 1000;

    h = 10;

    float range = fabsf(max_value) + fabsf(min_value);
    float fh = height - h - 2;

    snprintf(str, sizeof(str), "%ds", end_time);
    w = twr_gfx_calc_string_width(gfx, str) + 8;

    int lines = width / w;
    int y_time = y1 - h - 2;
    int y_zero = range > 0 ? y_time - ((fabsf(min_value) / range) * fh) : y_time;
    int tmp;

    for (int i = 0, time_step = end_time / lines, w_step = width / lines; i < lines; i++)
    {
        snprintf(str, sizeof(str), "%ds", time_step * i);

        w = twr_gfx_calc_string_width(gfx, str);

        tmp = width - w_step * i;

        twr_gfx_draw_string(gfx, tmp - w, y1 - h, str, 1);

        twr_gfx_draw_line(gfx, tmp - 2, y_zero - 2, tmp - 2, y_zero + 2, 1);

        twr_gfx_draw_line(gfx, tmp - 2, y0, tmp - 2, y0 + 2, 1);

        if (y_time != y_zero)
        {
            twr_gfx_draw_line(gfx, tmp - 2, y_time - 2, tmp - 2, y_time, 1);
        }
    }

    twr_gfx_draw_line(gfx, x0, y_zero, x1, y_zero, 1);

    if (y_time != y_zero)
    {
        twr_gfx_draw_line(gfx, x0, y_time, y1, y_time, 1);

        snprintf(str, sizeof(str), format, min_value);

        twr_gfx_draw_string(gfx, x0, y_time - 10, str, 1);
    }

    twr_gfx_draw_line(gfx, x0, y0, x1, y0, 1);

    snprintf(str, sizeof(str), format, max_value);

    twr_gfx_draw_string(gfx, x0, y0, str, 1);

    twr_gfx_draw_string(gfx, x0, y_zero - 10, "0", 1);

    if (range == 0)
    {
        return;
    }

    int length = twr_data_stream_get_length(data_stream);
    float value;

    int x_zero = x1 - 2;
    float fy;

    int dx = width / (number_of_samples - 1);
    int point_x = x_zero + dx;
    int point_y;
    int last_x;
    int last_y;

    min_value = fabsf(min_value);

    for (int i = 1; i <= length; i++)
    {
        if (twr_data_stream_get_nth(data_stream, -i, &value))
        {
            fy = (value + min_value) / range;

            point_y = y_time - (fy * fh);
            point_x -= dx;

            if (i == 1)
            {
                last_y = point_y;
                last_x = point_x;
            }

            twr_gfx_draw_line(gfx, point_x, point_y, last_x, last_y, 1);

            last_y = point_y;
            last_x = point_x;

        }
    }
}


void application_task(void)
{
    if (!twr_gfx_display_is_ready(gfx))
    {
            twr_scheduler_plan_current_from_now(50);
    }

    if (sleep)
    {
        twr_gfx_clear(gfx);
        twr_gfx_set_font(gfx, &twr_font_ubuntu_33);
        twr_gfx_printf(gfx, 5, 10, true, "Sleep");
        twr_gfx_update(gfx);

        return;
    }

    twr_system_pll_enable();

    // Wait while signal is low
    while (twr_gpio_get_input(TWR_GPIO_P4));

    // Wait until signal is high - rising edge
    while (!twr_gpio_get_input(TWR_GPIO_P4));

    twr_timer_start();

    // Wait while signal is low
    while (twr_gpio_get_input(TWR_GPIO_P4));

    uint32_t microseconds = twr_timer_get_microseconds();

    twr_timer_stop();

    twr_gfx_clear(gfx);
    float centimeters = microseconds / 58.0f;

    twr_gfx_set_font(gfx, &twr_font_ubuntu_33);
    twr_gfx_printf(gfx, 5, 10, true, "%.1f", centimeters);

    twr_gfx_set_font(gfx, &twr_font_ubuntu_24);
    twr_gfx_printf(gfx, 90, 15, true, "cm");

    twr_data_stream_feed(&distance_stream, &centimeters);
    graph(gfx, 0, 40, 127, 127, &distance_stream, DISTANCE_MEASURE_PERIOD, "%.1f");

    twr_gfx_update(gfx);

    twr_log_debug("%d cm", centimeters);

    twr_system_pll_disable();
    twr_scheduler_plan_current_from_now(DISTANCE_MEASURE_PERIOD);
}
