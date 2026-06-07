/*This project implements an embedded closed-loop position control system for a modified servo motor using an ESP32 and FreeRTOS.

The original servo controller was removed, allowing direct access to the DC motor and feedback potentiometer. The potentiometer is connected to the ESP32 

ADC inputs and used as the position feedback sensor, while the motor is driven through an H-bridge using bidirectional PWM control.

The system features a configurable PID controller with anti-windup protection and derivative filtering. Control parameters and setpoint can be adjusted 

either locally through analog potentiometers or remotely through a SCADA/LabVIEW interface via UART communication.

An SSD1306 OLED display provides real-time monitoring of system variables including position, setpoint, PID gains, error, and PWM output. 

The firmware is built on FreeRTOS and uses multiple concurrent tasks for ADC acquisition, PID computation, UART communication, OLED updates, 

and user input handling.

Key Features:

* Closed-loop position control
* PID controller with tunable gains
* Bidirectional H-bridge motor control
* Real-time parameter adjustment
* UART communication with SCADA/LabVIEW
* SSD1306 OLED interface
* FreeRTOS multitasking architecture
* Disturbance injection for control system testing
* ESP-IDF based implementation
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

// -------------------------------------------------------------------
// Pines y constantes
// -------------------------------------------------------------------
#define MOTOR_FWD 2
#define MOTOR_BWD 4
#define CONT_PIN 16
#define SWITCH_PIN 5

#define POT_SETPOINT ADC_CHANNEL_5 // GPIO33
#define POT_KP ADC_CHANNEL_6       // GPIO34
#define POT_KI ADC_CHANNEL_7       // GPIO35
#define POT_KD ADC_CHANNEL_0       // GPIO36
#define ADC_POSICION ADC_CHANNEL_4 // GPIO32

#define DUTY_MAX 1023
#define TIMER_MS 100
#define UART_PORT UART_NUM_0
#define BUF_SIZE 128
#define TASK_MEMORY 3072

#define SETPOINT_MAX 255.0f
#define KP_MAX 50.0f
#define KI_MAX 25.0f
#define KD_MAX 5.0f

// -------------------------------------------------------------------
// OLED SSD1306 — I2C
// -------------------------------------------------------------------
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA 21
#define I2C_MASTER_SCL 22
#define I2C_MASTER_FREQ 400000
#define OLED_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_PAGES 8 // 64px / 8

static uint8_t oled_buf[OLED_WIDTH * OLED_PAGES];

static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // ' ' 32
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // '!' 33
    {0x00, 0x00, 0x00, 0x00, 0x00}, // '"' 34
    {0x00, 0x00, 0x00, 0x00, 0x00}, // '#' 35
    {0x00, 0x00, 0x00, 0x00, 0x00}, // '$' 36
    {0x00, 0x00, 0x00, 0x00, 0x00}, // '%' 37
    {0x00, 0x00, 0x00, 0x00, 0x00}, // '&' 38
    {0x00, 0x00, 0x00, 0x00, 0x00}, // ''' 39
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // '(' 40
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // ')' 41
    {0x00, 0x00, 0x00, 0x00, 0x00}, // '*' 42
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // '+' 43
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ',' 44
    {0x08, 0x08, 0x08, 0x08, 0x08}, // '-' 45
    {0x00, 0x60, 0x60, 0x00, 0x00}, // '.' 46
    {0x20, 0x10, 0x08, 0x04, 0x02}, // '/' 47
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // '0'
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // '1'
    {0x42, 0x61, 0x51, 0x49, 0x46}, // '2'
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // '4'
    {0x27, 0x45, 0x45, 0x45, 0x39}, // '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // '6'
    {0x01, 0x71, 0x09, 0x05, 0x03}, // '7'
    {0x36, 0x49, 0x49, 0x49, 0x36}, // '8'
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // '9'
    {0x00, 0x36, 0x36, 0x00, 0x00}, // ':'
    {0x00, 0x00, 0x00, 0x00, 0x00}, // ';'
    {0x00, 0x00, 0x00, 0x00, 0x00}, // '<'
    {0x00, 0x00, 0x00, 0x00, 0x00}, // '='
    {0x00, 0x00, 0x00, 0x00, 0x00}, // '>'
    {0x00, 0x00, 0x00, 0x00, 0x00}, // '?'
    {0x00, 0x00, 0x00, 0x00, 0x00}, // '@'
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 'A'
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 'B'
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 'C'
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 'D'
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 'E'
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 'F'
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 'G'
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 'H'
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 'I'
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 'J'
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 'K'
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 'L'
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 'M'
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 'N'
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 'O'
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 'P'
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 'Q'
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 'R'
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 'S'
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 'T'
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 'U'
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 'V'
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 'W'
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 'X'
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 'Y'
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 'Z'
};

// -------------------------------------------------------------------
// I2C + OLED — funciones base INIT
// -------------------------------------------------------------------
static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

static void oled_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    i2c_master_write_to_device(I2C_MASTER_NUM, OLED_ADDR, buf, 2, pdMS_TO_TICKS(10));
}

//REFRESQUEO DE LA PANTALLA 

static void oled_flush(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; page++)
    {
        oled_cmd(0xB0 + page);
        oled_cmd(0x00);
        oled_cmd(0x10);
        uint8_t buf[OLED_WIDTH + 1];
        buf[0] = 0x40;
        memcpy(buf + 1, oled_buf + page * OLED_WIDTH, OLED_WIDTH);
        i2c_master_write_to_device(I2C_MASTER_NUM, OLED_ADDR, buf,
                                   sizeof(buf), pdMS_TO_TICKS(20));
    }
}

// INICIALISACION DE LA PANTALLA 

static void oled_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(200)); // más tiempo de espera al arranque

    uint8_t cmds[] = {
        0xAE, // display off
        0xD5,
        0x80, // set display clock divide
        0xA8,
        0x3F, // set multiplex 63
        0xD3,
        0x00, // set display offset 0
        0x40, // set start line 0
        0x8D,
        0x14, // charge pump ON
        0x20,
        0x00, // memory mode horizontal
        0xA1, // segment remap
        0xC8, // com scan direction
        0xDA,
        0x12, // com pins config
        0x81,
        0xCF, // set contrast
        0xD9,
        0xF1, // pre-charge period
        0xDB,
        0x40, // vcomh deselect level
        0xA4, // display from RAM
        0xA6, // normal display (no inverse)
        0xAF, // display ON
    };

    for (int i = 0; i < sizeof(cmds); i++)
        oled_cmd(cmds[i]);

    memset(oled_buf, 0, sizeof(oled_buf));
    oled_flush();
}

//LIMPIAR PANTALLA
  
static void oled_clear(void)
{
    memset(oled_buf, 0, sizeof(oled_buf));
}

// GRAVAS PANTALLA 

static void oled_draw_char(uint8_t x, uint8_t page, char c)
{
    if (c < 32 || c > 'Z')
        c = ' ';
    const uint8_t *glyph = font5x7[c - 32];
    for (int i = 0; i < 5; i++)
    {
        if (x + i < OLED_WIDTH)
            oled_buf[page * OLED_WIDTH + x + i] = glyph[i];
    }
    if (x + 5 < OLED_WIDTH)
        oled_buf[page * OLED_WIDTH + x + 5] = 0x00;
}

// Scanner I2C — imprime qué dirección responde
void i2c_scan(void)
{
    printf("Escaneando I2C...\n");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK)
            printf("Dispositivo encontrado en: 0x%02X\n", addr);
    }
}

static void oled_draw_str(uint8_t x, uint8_t page, const char *str)
{
    while (*str)
    {
        oled_draw_char(x, page, *str++);
        x += 6;
        if (x >= OLED_WIDTH)
            break;
    }
}

// -------------------------------------------------------------------
// Estructura queue
// -------------------------------------------------------------------
typedef struct
{
    uint32_t setpoint;
    float kp;
    float ki;
    float kd;
} pid_params_t;

// -------------------------------------------------------------------
// Variables globales
// -------------------------------------------------------------------
volatile float kp = 4.0f;
volatile float ki = 4.5f;
volatile float kd = 0.2f;
volatile uint32_t setpoint = 0;
volatile uint32_t velosity = 0U;
volatile uint32_t pwm_asignate = 0U;
volatile int32_t pid_output = 0;
volatile uint8_t RPM = 0;
volatile int32_t error = 0;
volatile int32_t perturbacion = 0; // Almacena el valor del dial de LabVIEW (-1023 a 1023 o 0 a 100)

TimerHandle_t pwm_timer;
QueueHandle_t pid_queue;

static adc_oneshot_unit_handle_t adc1_handle;

// -------------------------------------------------------------------
// Prototipos
// -------------------------------------------------------------------
static void PID_TASK(void *pvParameters);
static void ADC_TASK(void *pvParameters);
static void POT_TASK(void *pvParameters);
static void uart_task(void *pvParameters);
static void OLED_TASK(void *pvParameters);
static void IRAM_ATTR gpio_isr_handler(void *arg);

// -------------------------------------------------------------------
// UART
// -------------------------------------------------------------------
static void init_uart(void)
{
    uart_config_t uart_con = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_delete(UART_PORT);
    esp_err_t err = uart_driver_install(UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    if (err == ESP_OK)
    {
        uart_param_config(UART_PORT, &uart_con);
        uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
}

static void uart_task(void *pvParameters)
{
    uint8_t *buf = (uint8_t *)malloc(BUF_SIZE);
    char line[BUF_SIZE];
    int pos = 0;
    while (1)
    {
        int len = uart_read_bytes(UART_PORT, buf, BUF_SIZE - 1, pdMS_TO_TICKS(10));
        for (int i = 0; i < len; i++)
        {
            char c = (char)buf[i];
            if (c == '\n' || c == '\r')
            {
                if (pos > 0)
                {
                    line[pos] = '\0';
                    if (gpio_get_level(SWITCH_PIN) == 0) // Modo SCADA
                    {
                        // Creamos una estructura usando los valores globales de ESTE instante
                        pid_params_t p = {setpoint, kp, ki, kd};
                        // ... dentro del lazo de la uart_task, en la sección de Modo SCADA:
                        if (strncmp(line, "S:", 2) == 0)
                        {
                            p.setpoint = atoi(line + 2);
                        }
                        else if (strncmp(line, "P:", 2) == 0)
                        {
                            p.kp = atof(line + 2);
                        }
                        else if (strncmp(line, "I:", 2) == 0)
                        {
                            p.ki = atof(line + 2);
                        }
                        else if (strncmp(line, "D:", 2) == 0)
                        {
                            p.kd = atof(line + 2);
                        }
                        else if (strncmp(line, "U:", 2) == 0) // <--- NUEVO: 'U' de Perturbación
                        {
                            perturbacion = atoi(line + 2); // Guarda directamente el valor del dial
                        }

                        // Actualizamos las variables globales al instante para que el próximo comando no las pise
                        setpoint = p.setpoint;
                        kp = p.kp;
                        ki = p.ki;
                        kd = p.kd;

                        xQueueOverwrite(pid_queue, &p);
                    }
                    pos = 0;
                }
            }
            else if (pos < (int)sizeof(line) - 2)
                line[pos++] = c;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(buf);
}

// -------------------------------------------------------------------
// PWM
// -------------------------------------------------------------------
void vTimerCallBack(TimerHandle_t xTimer)
{
    if (pid_output >= 0)
    {
        // Adelante: FWD con PWM, BWD en 0
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)pid_output);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    }
    else
    {
        // Atrás: FWD en 0, BWD con PWM (valor absoluto)
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (uint32_t)(-pid_output));
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void init_pwm(void)
{
    // Timer compartido para ambos canales
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    // Canal 0 → adelante (GPIO2)
    ledc_channel_config_t fwd_cfg = {
        .gpio_num = MOTOR_FWD,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&fwd_cfg);

    // Canal 1 → atrás (GPIO4)
    ledc_channel_config_t bwd_cfg = {
        .gpio_num = MOTOR_BWD,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&bwd_cfg);
}

// -------------------------------------------------------------------
// ADC
// -------------------------------------------------------------------
static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_POSICION, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_SETPOINT, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_KP, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_KI, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_KD, &chan_cfg));
    gpio_set_pull_mode(GPIO_NUM_32, GPIO_FLOATING);
    gpio_set_pull_mode(GPIO_NUM_33, GPIO_FLOATING);
    gpio_set_pull_mode(GPIO_NUM_34, GPIO_FLOATING);
    gpio_set_pull_mode(GPIO_NUM_35, GPIO_FLOATING);
    gpio_set_pull_mode(GPIO_NUM_36, GPIO_FLOATING);
}

static uint32_t adc_promedio(adc_channel_t canal, int muestras)
{
    int raw = 0;
    int32_t suma = 0;
    adc_oneshot_read(adc1_handle, canal, &raw);
    vTaskDelay(pdMS_TO_TICKS(2));
    for (int i = 0; i < muestras; i++)
    {
        adc_oneshot_read(adc1_handle, canal, &raw);
        suma += raw;
    }
    return (uint32_t)(suma / muestras);
}

// -------------------------------------------------------------------
// MAIN
// -------------------------------------------------------------------
void app_main(void)
{
    init_pwm();
    init_uart();
    init_adc();
    i2c_init();
    i2c_scan();
    oled_init();

    gpio_config_t sw_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << SWITCH_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&sw_conf);

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CONT_PIN),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CONT_PIN, gpio_isr_handler, (void *)CONT_PIN);

    pid_queue = xQueueCreate(1, sizeof(pid_params_t));

    pwm_timer = xTimerCreate("PWM_TIMER", pdMS_TO_TICKS(TIMER_MS),
                             pdTRUE, (void *)0, vTimerCallBack);
    xTimerStart(pwm_timer, 0);

    xTaskCreate(uart_task, "uart_task", TASK_MEMORY, NULL, 6, NULL);
    xTaskCreate(PID_TASK, "PID_TASK", TASK_MEMORY, NULL, 5, NULL);
    xTaskCreate(ADC_TASK, "ADC_TASK", TASK_MEMORY, NULL, 4, NULL);
    xTaskCreate(POT_TASK, "POT_TASK", TASK_MEMORY, NULL, 3, NULL);
    xTaskCreate(OLED_TASK, "OLED_TASK", TASK_MEMORY, NULL, 2, NULL);

    uint8_t cont = 0;
    while (1)
    {
        cont++;
        vTaskDelay(pdMS_TO_TICKS(500));
        if (cont >= 250)
            cont = 0;
    }
}

void IRAM_ATTR gpio_isr_handler(void *arg) { velosity++; }

// -------------------------------------------------------------------
// POT TASK
// -------------------------------------------------------------------
static void POT_TASK(void *pvParameters)
{
    uint32_t raw_sp = 0;
    uint8_t last_switch = 2;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(150));

        uint8_t sw = gpio_get_level(SWITCH_PIN);

        // Detectar cambio de modo
        if (sw != last_switch)
        {
            xQueueReset(pid_queue);
            last_switch = sw;

            if (sw == 0)
            {
                // Entrando a modo SCADA — resetear PID con valores seguros
                pid_params_t p = {0, kp, ki, kd};
                xQueueOverwrite(pid_queue, &p);
                printf("Modo SCADA activo\n");
            }
            else
            {
                printf("Modo POTS activo\n");
            }
        }

        if (sw == 1)
        {
            uint32_t adc_read = adc_promedio(POT_SETPOINT, 8);
            if (adc_read > 0)
                raw_sp = adc_read;

            uint32_t raw_kp = adc_promedio(POT_KP, 8);
            uint32_t raw_ki = adc_promedio(POT_KI, 8);
            uint32_t raw_kd = adc_promedio(POT_KD, 8);

            pid_params_t p = {
                .setpoint = (uint32_t)((raw_sp / 4095.0f) * SETPOINT_MAX),
                .kp = (raw_kp / 4095.0f) * KP_MAX,
                .ki = (raw_ki / 4095.0f) * KI_MAX,
                .kd = (raw_kd / 4095.0f) * KD_MAX,
            };
            xQueueOverwrite(pid_queue, &p);
        }
    }
}

// -------------------------------------------------------------------
// PID TASK
// -------------------------------------------------------------------
static void PID_TASK(void *pvParameters)
{
    int32_t error_prev = 0;
    float integral = 0;
    float derivada = 0;
    const float dt = 0.1f;
    pid_params_t p;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz control de posición

        if (xQueueReceive(pid_queue, &p, 0) == pdTRUE)
        {
            setpoint = p.setpoint;
            kp = p.kp;
            ki = p.ki;
            kd = p.kd;
        }

        if (setpoint == 0)
        {
            pid_output = 0;
            integral = 0;
        }
        else
        {
            error = (int32_t)setpoint - (int32_t)RPM;
            integral += error * dt;

            // Anti-windup
            if (integral > 512)
                integral = 512;
            if (integral < -512)
                integral = -512;

            derivada = 0.7f * derivada + 0.3f * ((error - error_prev) / dt);

           

            // Salida puede ser negativa → motor gira atrás
            int32_t output = (int32_t)((kp * error) + (ki * integral) + (kd * derivada));
            output += perturbacion;

            // Saturar entre -DUTY_MAX y +DUTY_MAX
            if (output > DUTY_MAX)
                output = DUTY_MAX;
            if (output < -DUTY_MAX)
                output = -DUTY_MAX;

            pid_output = output;
           
        }

        error_prev = error;

        uint8_t pwm_pct = (uint8_t)(((float)abs(pid_output) / (float)DUTY_MAX) * 100.0f);
        const char *dir = pid_output > 0 ? "FWD" : (pid_output < 0 ? "BWD" : "STOP");
        printf("POS:%d,SP:%lu,PWM:%d,DIR:%s,KP:%.2f,KI:%.2f,KD:%.2f\n",
               RPM, setpoint, pwm_pct, dir, kp, ki, kd);
    }
}
// -------------------------------------------------------------------
// ADC TASK
// -------------------------------------------------------------------
static void ADC_TASK(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(20));

        uint32_t raw = adc_promedio(ADC_POSICION, 8);

        // Por ahora mapeo directo 0-4095 → 0-255
        // Aquí metes tu linealización cuando la tengas:
        float pos = (raw / 4095.0f) * 255.0f;

        RPM = (uint8_t)pos;
    }
}

// -------------------------------------------------------------------
// OLED TASK (Corregida y Optimizada)
// -------------------------------------------------------------------
static void OLED_TASK(void *pvParameters)
{
    char line[32];
    uint8_t pwm_pct;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(200)); // Refresco de 5Hz, ideal para no saturar el I2C

        oled_clear();

        // Calculamos el porcentaje real basado en la salida del PID absoluto
        pwm_pct = (uint8_t)(((float)abs(pid_output) / (float)DUTY_MAX) * 100.0f);

        if (gpio_get_level(SWITCH_PIN) == 1)
        {
            // Modo POTS — muestra ganancias
            oled_draw_str(0, 0, "MODE: POTS");

            snprintf(line, sizeof(line), "POS:%3d  SP:%3lu", RPM, setpoint);
            oled_draw_str(0, 1, line);

            snprintf(line, sizeof(line), "KP: %.2f", kp);
            oled_draw_str(0, 2, line);

            snprintf(line, sizeof(line), "KI: %.2f", ki);
            oled_draw_str(0, 3, line);

            snprintf(line, sizeof(line), "KD: %.2f", kd);
            oled_draw_str(0, 4, line);

            // Cambiado a pwm_pct para ver el % real. Eliminado el '%' final para evitar fallos de fuente
            snprintf(line, sizeof(line), "PWM: %3d PCT", pwm_pct);
            oled_draw_str(0, 5, line);
        }
        else
        {
            // Modo LabVIEW — muestra estado SCADA
            oled_draw_str(0, 0, "MODE: SCADA");

            snprintf(line, sizeof(line), "POS:%3d  SP:%3lu", RPM, setpoint);
            oled_draw_str(0, 1, line);

            snprintf(line, sizeof(line), "PWM: %3d PCT", pwm_pct);
            oled_draw_str(0, 2, line);

            snprintf(line, sizeof(line), "KP:%.1f KI:%.1f", kp, ki);
            oled_draw_str(0, 3, line);

            snprintf(line, sizeof(line), "KD: %.2f", kd);
            oled_draw_str(0, 4, line);
        }

        snprintf(line, sizeof(line), "ERR: %ld", error);
        oled_draw_str(0, 6, line);
        oled_flush();
    }
}
