#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "6502ROM.h"
#include "hardware/gpio.h"
#include "e6821.h"

#define lengthof(x) sizeof(x)/sizeof(x[0])


#define RW_PIN (16)
#define CLK_PIN (17)
#define IRQ_PIN (18)
#define NMI_PIN (19)
#define RST_PIN (20)
#define ADDR_PAGE_PIN (21)
#define BUTTON_PIN (22)
/** address range from 0-7 */
#define ADDR_PIN_MASK (0x000000FF)
/** data range from 8-15 */
#define DATA_PIN_MASK (0x0000FF00)

// #define DEBUG_MODE

#ifdef DEBUG_MODE
#define HALF_CLK_CYCLE_MS (250)
#else
#define HALF_CLK_CYCLE_MS (1)
#endif


static void terminal_putchar(uint8_t data ,void* ctx);

static uint8_t memory[0x10000] = {0};

int main()
{
    stdio_init_all();

    /** init hardware */
    {
        /** set address and data to inputs */
        gpio_init_mask(ADDR_PIN_MASK|DATA_PIN_MASK);
        gpio_set_dir_masked(ADDR_PIN_MASK|DATA_PIN_MASK,0);
        /** set rw to input */
        gpio_init(RW_PIN);
        gpio_set_dir(RW_PIN,false);
        /** set CLK to 0 */
        gpio_init(CLK_PIN);
        gpio_set_dir(CLK_PIN,true);
        gpio_put(CLK_PIN,0);
        /** set IRQ to 1 */
        gpio_init(IRQ_PIN);
        gpio_set_dir(IRQ_PIN,true);
        gpio_put(IRQ_PIN,1);
        /** set NMI to 1 */
        gpio_init(NMI_PIN);
        gpio_set_dir(NMI_PIN,true);
        gpio_put(NMI_PIN,1);
        /** set RST to 1 */
        gpio_init(RST_PIN);
        gpio_set_dir(RST_PIN,true);
        gpio_put(RST_PIN,1);
        /** set address page to 0 */
        gpio_init(ADDR_PAGE_PIN);
        gpio_set_dir(ADDR_PAGE_PIN,true);
        gpio_put(ADDR_PAGE_PIN,0);
        /** set button pin to input pull up */
        gpio_init(BUTTON_PIN);
        gpio_set_dir(BUTTON_PIN,false);
        gpio_pull_up(BUTTON_PIN);
    }
    /** wait for user input to keep going */
    // getchar();
    printf("6502 CPU starting...\n");
reset:
#ifdef DEBUG_MODE
    /** to avoid multiple resets in debug mode */
    sleep_ms(1000);
#endif
    /** reset system */
    {
        /** reset memory */
        memset(memory,0,sizeof(memory));
        /** load ROM */
        for(int i=0;i<lengthof(ROM_BLOCKS);i++)
        {
            const rom_block_t* rom_block = &ROM_BLOCKS[i];
            memcpy(memory+rom_block->offset,rom_block->data,rom_block->length);
        }
        /** reset 6821 */
        e6821_reset();
        e6821_set_device_hook(E6821_PORT_B,terminal_putchar,NULL);
        /** RST -> low */
        gpio_put(RST_PIN,0);
        sleep_ms(HALF_CLK_CYCLE_MS);
        /** wait for at least 2 clock cycles */
        for(int i=0;i<3;i++)
        {
            gpio_put(CLK_PIN,1);
            sleep_ms(HALF_CLK_CYCLE_MS);
            gpio_put(CLK_PIN,0);
            sleep_ms(HALF_CLK_CYCLE_MS);
        }
        /** RST -> high */
        gpio_put(RST_PIN,1);
        sleep_ms(HALF_CLK_CYCLE_MS);
    }
    
    /** run clock cycles */
    for(;;)
    {
        /** check stdin input */
        int c = getchar_timeout_us(0);
        if(c >= 0)
        {
            /** convert to higher case */
            if(c>='a'&&c<='z')
                c = c - ('a'-'A');
            /** keyboard interface always set bit 7 to 1 */
            e6821_input_from_device(E6821_PORT_A,0x80|c);
            e6821_set_irq(E6821_PORT_A,E6821_IRQ_LINE_1);
        }
        /** reset data pin to input */
        {
            gpio_set_dir_in_masked(DATA_PIN_MASK);
        }
        /** check button */
        if(gpio_get(BUTTON_PIN) == 0)
            goto reset;
        /** read address & r/w */
        uint16_t addr = 0;
        bool is_read = false;
        {
            uint32_t v = gpio_get_all();
            gpio_put(ADDR_PAGE_PIN,1);  /** switch page at once */
            addr = v & 0xFF;
            is_read = (v & (1<<RW_PIN)) != 0;
            /** @todo replace NOPs to achieve minimum delay */
            sleep_us(1);    /** wait for address page switching, should be faster */
            v = gpio_get_all();
            addr = addr | ((v&0xFF) << 8);
            gpio_put(ADDR_PAGE_PIN,0);  /** switch back address page */
        }
#ifdef DEBUG_MODE
        printf("Address:%04x,is_read:%d\n",addr,is_read);
#endif
        /** handle memory read */
        if(is_read)
        {
            gpio_set_dir_out_masked(DATA_PIN_MASK);
            if((addr & 0xF000) == 0xD000)
            {
                /** PIA */
                gpio_put_masked(DATA_PIN_MASK,((uint32_t)e6821_read(addr&0b11))<<8);
            }
            else
            {
                gpio_put_masked(DATA_PIN_MASK,((uint32_t)memory[addr])<<8);
            }
        }
        /** send clk pos edge */
        gpio_put(CLK_PIN,1);
        sleep_ms(HALF_CLK_CYCLE_MS);
        /** handle memory write */
        if(!is_read)
        {
            uint32_t v = gpio_get_all();
            uint8_t data = (v>>8) & 0xFF;
#ifdef DEBUG_MODE
            printf("Write %02x to %04x\n",data,addr);
#endif
            if((addr & 0xF000) == 0xD000)
            {
                /** PIA */
                e6821_write(addr&0b11,data);
            }
            else if(!(addr & 0x8000))
            {
                /** RAM */
                memory[addr] = data;
            }
        }
        /** send clk neg edge */
        gpio_put(CLK_PIN,0);
        sleep_ms(HALF_CLK_CYCLE_MS);
    }

    return 0;
}

static void terminal_putchar(uint8_t data ,void* ctx)
{
    data &= 0x3F;
    int c = (data==0x0D)? '\n' :(data & 0x20)? data : data+0x40;
#ifndef DEBUG_MODE
    printf("%c",c);
#else
    printf("Terminal: data:%02x, char:%c\n",data,c);
#endif
    /** write back to inform the cpu */
    e6821_input_from_device(E6821_PORT_B,0x00);
}
