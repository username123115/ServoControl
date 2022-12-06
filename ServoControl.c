#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "time.h"

#define captureSize 432 //should be big enough at sampling rate to capture the entirety of the control signal
#define samplingRate 190000 //38000 * 5
#define capturePin 13
#define vS 14
#define decode 1
#define servoA 12
#define servoB 11

// ------- I2C pain --------

#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define POWER 13
#define PWR_MGMT_1 0x6B //107
#define GYRO_CONFIG 0x1B //27
#define ACCEL_CONFIG 0x1C //28
#define MEASURE_START 0x3B //Accel measurements start
#define MPU6050 0x68
#define LED_PIN 6
#define GYRO_TASK_TIME 100000

int writeRegister(i2c_inst_t * i2c, 
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes);

int readRegister(i2c_inst_t * i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes);

// ------- I2C pain --------


void dmaHandler();
static inline void setupDma(PIO, uint);
static inline void setupPIO(PIO, uint, uint, bool, float);
uint16_t convertValue(float*);

uint32_t buffer[captureSize];
uint16_t wrapValue = 50000;
uint dmaChan;
PIO pio;
uint sm;
uint servoPins[] = {12, 11};
uint totalServos = 2;

float servoValues[] = {80, 80}; //two servos to control, 0-180
float pwmDiv = 50.f;
bool forwardDrive = true;

bool updateValues = false;

int main()
{
    stdio_init_all();
    dmaChan = dma_claim_unused_channel(true);
    float clkdiv = clock_get_hz(clk_sys) / (samplingRate);

    irq_set_exclusive_handler(DMA_IRQ_0, dmaHandler);
    irq_set_enabled(DMA_IRQ_0, true);
    printf("%d\n", irq_is_enabled(DMA_IRQ_0));

    pio = pio0;
    sm = pio_claim_unused_sm(pio, true);

    //setup power to sensor
    gpio_init(vS);
    gpio_set_dir(vS, true);
    gpio_put(vS, true);
    //for interfacing with sensor
    gpio_pull_up(capturePin);

    setupDma(pio, sm);
    setupPIO(pio, sm, capturePin, false, clkdiv);
    
    //--------- SERVO ----------
    gpio_set_function(servoA, GPIO_FUNC_PWM);
    gpio_set_function(servoB, GPIO_FUNC_PWM);
    pwm_config servoConf = pwm_get_default_config();
    pwm_config_set_wrap(&servoConf, wrapValue - 1);
    pwm_config_set_clkdiv(&servoConf, pwmDiv);
    for (int i = 0; i < totalServos; i++)
    {
        pwm_init(pwm_gpio_to_slice_num(servoPins[i]), &servoConf, true);
        pwm_set_gpio_level(servoPins[i], convertValue(&servoValues[i]));
    }
    // pwm_init(pwm_gpio_to_slice_num(servoA), &servoConf, true);
    // pwm_set_gpio_level(servoA, convertValue(servoValues[0]));
    //--------- SERVO ----------

    uint32_t timeNow = time_us_32(); //this can last an hour?
    
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, true);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    uint8_t powerManagementSettings = 0x00;
    uint8_t gyroConfigSettings = 0x00;
    uint8_t accelConfigSettings = 0x00;

    writeRegister(I2C_PORT, MPU6050, PWR_MGMT_1, &powerManagementSettings, 1);
    writeRegister(I2C_PORT, MPU6050, GYRO_CONFIG, &gyroConfigSettings, 1);
    writeRegister(I2C_PORT, MPU6050, ACCEL_CONFIG, &accelConfigSettings, 1);

    uint8_t accelMeasurements[] = {0, 0};
    uint8_t regAddress = 0x47;
    uint16_t final = 0;
    bool neg = false;

    //--------- more I2C pain --------


    while (true)
    {
        // printf("%u\n", pio_sm_get_blocking(pio, sm));
        if ((time_us_32() - timeNow) > GYRO_TASK_TIME)
        {
            // printf("bruh\n");
            readRegister(I2C_PORT, MPU6050, regAddress, (uint8_t*)&accelMeasurements, 2);
            final = (accelMeasurements[0] << 8) + accelMeasurements[1];
            neg = false;
            if ((accelMeasurements[0] >> 7) == 1)
            {
                neg = true;
            }
            if (neg)
            {
                final = ((~final) & 0xFFFF) + 1;
            }
        if (neg && final > 256)
        {
            printf("right\n");
            gpio_put(LED_PIN, true);
        }
        if (!neg && final > 256)
        {
            printf("left\n");
            gpio_put(LED_PIN, false);
        }


            timeNow += GYRO_TASK_TIME;
        }

        if (updateValues)
        {
            for (int i = 0; i < totalServos; i++)
            {
                pwm_init(pwm_gpio_to_slice_num(servoPins[i]), &servoConf, true);
                pwm_set_gpio_level(servoPins[i], convertValue(&servoValues[i]));
            }
            updateValues = false;
        }
        // printf("%d\n", dma_channel_is_busy(dmaChan));
    }
    return 0;
}
uint16_t convertValue(float *angle)
{
    if (*angle < 0) {*angle = 0;}
    if (*angle > 180) {*angle = 180;}
    float portion = *angle / 180;
    uint16_t result = ((1 + portion) / 20) * wrapValue;
    printf("%u\n", result);
    return result;
}

void dmaHandler()
{
    // printf("Received, proccessing ready\n");
    bool state = true;
    bool currentState;
    uint32_t stateWidth = 0;
    uint32_t tempWidth;
    uint32_t extractedData = 0;
    for (int i = 0; i < captureSize; i++)
    {
        for (int j = 0; j < 32; j++) //iterate from lsb of data
        {
            currentState = !((bool) ((buffer[i] >> j) && 1u)); //1 is off 0 is on
            if (currentState == state)
            {
                stateWidth += 1;
            }
            if (currentState != state)
            {
                if (!decode)
                {
                    printf("State: %s, Width: %d\n", state ? "True": "False", stateWidth);
                }
                if (decode)
                {
                    if (state)
                    {
                        tempWidth = stateWidth; // the on time divided by off time will determine if the signal is on or off
                    }
                    if (!state) //it is false so divide true by false
                    {
                        float temp = (float) tempWidth / stateWidth;
                        // printf("%2.3f\n", temp);
                        if (temp < 1.2)
                        {
                            extractedData *= 2;
                            if (temp < 0.5)
                            {
                                extractedData += 1;
                            }
                        }

                    }
                }

                stateWidth = 1;
                state = !state;
            }
        }
    }
    if ((stateWidth != 1) && (!decode))
    {
        printf("State: %s, Width: %d\n", state ? "True": "False", stateWidth);
    }
    // printf("Transfer done\n");

    if (decode)
    {
        char n2 = extractedData & (char) 255;
        char n1 = (extractedData >> 8) & (char) 255;
        if (n1 + n2 == 255)
        {
            // printf("%u\n", extractedData);
            printf("Raw: %u, Command: %u, Repeat: %d\n", extractedData, n1 >> 1, n1 & 1u);
            updateValues = true;
            switch (n1 >> 1)
            {
            case 76: //up
                servoValues[0] = 120; //original 120 moter a = left
                servoValues[1] = 80;
                break;
            case 102:
                servoValues[0] = 85;
                servoValues[1] = 20;
                break;
            case 42:
                servoValues[0] = 85;
                servoValues[1] = 85;
                break;
            default:
                updateValues = false;
                break;
            }
        }
    }


    dma_channel_acknowledge_irq0(dmaChan);
    pio_sm_exec(pio, sm, pio_encode_wait_pin(false, 0));
    dma_channel_set_write_addr(dmaChan, &buffer, true);
    pio_sm_clear_fifos(pio, sm);
    return;
}

static inline void setupPIO(PIO pio, uint sm, uint pin, bool trigger, float clkdiv)
{
    uint16_t instr = pio_encode_in(pio_pins, 1);
    struct pio_program instructions = {
        .instructions = &instr,
        .length = 1,
        .origin = -1
    };
    uint offset = pio_add_program(pio, &instructions);

    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset);

    sm_config_set_in_pins(&c, pin);
    sm_config_set_in_shift(&c, true, true, 32);  //set autopush to true
    sm_config_set_clkdiv(&c, clkdiv);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_exec(pio, sm, pio_encode_wait_pin(trigger, 0));
    pio_sm_set_enabled(pio, sm, true);

}

static inline void setupDma(PIO pio, uint sm)
{
    dma_channel_config dc = dma_channel_get_default_config(dmaChan);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_dreq(&dc, pio_get_dreq(pio, sm, false));
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);

    dma_channel_set_irq0_enabled(dmaChan, true);
    dma_channel_configure(dmaChan,
        &dc,
        &buffer,
        &pio->rxf[sm],
        captureSize,
        true
    );

}

int writeRegister(i2c_inst_t * i2c, 
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes)
{
    if (nbytes < 0)
    {
        return 0;
    }
    int bytesWritten = 0;
    uint8_t * msg = malloc(sizeof(uint8_t) * (nbytes + 1));
    msg[0] = reg;
    memcpy(msg + 1, buf, nbytes);
    bytesWritten = i2c_write_blocking(i2c, addr, msg, nbytes + 1, false);

    free(msg);
    msg = NULL;
    return bytesWritten;
}
int readRegister(i2c_inst_t * i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes)
{
    if (nbytes < 1)
    {
        return 0;
    }
    int bytesRead = 0;
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    bytesRead = i2c_read_blocking(i2c, addr, buf, nbytes, false);
    return bytesRead;
}

// % c-sdk {
//     static inline void initCapture(Pio pio, uint sm, uint offset, uint pin, float div)
//     {
        
//     }

// %}