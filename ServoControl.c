#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#define captureSize 432
#define capturePin 13
#define vS 14
#define decode 1

void dmaHandler();
static inline void setupDma(PIO, uint);
static inline void setupPIO(PIO, uint, uint, bool, float);

uint32_t buffer[captureSize];
uint dmaChan;
PIO pio;
uint sm;

int main()
{
    stdio_init_all();
    dmaChan = dma_claim_unused_channel(true);
    float clkdiv = clock_get_hz(clk_sys) / (38000 * 5);

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

    while (true)
    {
        // printf("%u\n", pio_sm_get_blocking(pio, sm));
        tight_loop_contents();
        // printf("%d\n", dma_channel_is_busy(dmaChan));
    }
    return 0;
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

// % c-sdk {
//     static inline void initCapture(Pio pio, uint sm, uint offset, uint pin, float div)
//     {
        
//     }

// %}