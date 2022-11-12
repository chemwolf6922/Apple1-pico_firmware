#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "6502ROM.h"
#include "hardware/gpio.h"
#include "pico/multicore.h" 

#define lengthof(x) sizeof(x)/sizeof(x[0])

/**
 * Hardware define
 */
#define RW_PIN (16)
#define CLK_PIN (17)
#define IRQ_PIN (18)
#define NMI_PIN (19)
#define RST_PIN (20)
#define ADDR_PAGE_PIN (21)
#define BUTTON_PIN (22)
/** address range from 0-7 */
#define ADDR_PIN_MASK (0x000000FFu)
/** data range from 8-15 */
#define DATA_PIN_MASK (0x0000FF00u)

#define HALF_CLK_CYCLE_US (1)

/**
 * Inter core messages
 */
#define CORE1_DATA_MASK     (0x000000FFu)
#define CORE1_HAS_ACK       (0x00000100u)
#define CORE1_HAS_DATA      (0x00000200u)

/**
 * 6821 fast implementation
 */
typedef enum
{
    E6821_PORT_A,
    E6821_PORT_B
} e6821_port_t;

typedef enum
{
    E6821_IRQ_LINE_1,
    E6821_IRQ_LINE_2
} e6821_irq_line_t;

typedef struct
{
    uint8_t data_A;
    uint8_t direction_A;
    uint8_t contorl_A;
    uint8_t data_B;
    uint8_t direction_B;
    uint8_t contorl_B;
} e6821_reg_dump_t;

typedef void(*e6821_output_to_device_t)(uint8_t data ,void* ctx);

/** 6821_fast implementation */
typedef struct
{
    uint8_t data_A;
    uint8_t direction_A;
    union
    {
        struct {
            uint8_t C1_enable:1;
            uint8_t C1_edge:1;
            uint8_t DDR_access:1;
            uint8_t C2_b3:1;
            uint8_t C2_b4:1;
            uint8_t C2_output:1;
            uint8_t IRQ2:1;
            uint8_t IRQ1:1;
        } __attribute__((packed)) bits;
        uint8_t byte; 
    } control_A;

    uint8_t data_B;
    uint8_t direction_B;
    union
    {
        struct {
            uint8_t C1_enable:1;
            uint8_t C1_edge:1;
            uint8_t DDR_access:1;
            uint8_t C2_b3:1;
            uint8_t C2_b4:1;
            uint8_t C2_output:1;
            uint8_t IRQ2:1;
            uint8_t IRQ1:1;
        } __attribute__((packed)) bits;
        uint8_t byte; 
    } control_B;

    struct
    {
        e6821_output_to_device_t write_to_device_A;
        void* ctx_A;
        e6821_output_to_device_t write_to_device_B;
        void* ctx_B;
    } callbacks;
} e6821_t;

static e6821_t e6821 = {0};

static inline __attribute__((always_inline)) void e6821_reset()
{
    memset(&e6821,0,sizeof(e6821));
}

static inline __attribute__((always_inline)) void e6821_set_device_hook(e6821_port_t port, e6821_output_to_device_t write_to_device, void* ctx)
{
    if(port == E6821_PORT_A)
    {
        e6821.callbacks.write_to_device_A = write_to_device;
        e6821.callbacks.ctx_A = ctx;
    }
    else if(port == E6821_PORT_B)
    {
        e6821.callbacks.write_to_device_B = write_to_device;
        e6821.callbacks.ctx_B = ctx;
    }
}

/**
 * @param rs bit1: rs1, bit0: rs0
 * @param data data
 */
static inline __attribute__((always_inline)) void e6821_write(int rs, uint8_t data)
{
    switch (rs)
    {
    case 0b00:{
        if(e6821.control_A.bits.DDR_access)
        {
            /** data A */
            e6821.data_A = (data & e6821.direction_A) | (e6821.data_A & (~e6821.direction_A));
            if(e6821.callbacks.write_to_device_A)
            {
                e6821.callbacks.write_to_device_A(data,e6821.callbacks.ctx_A);
            }
        }
        else
        {
            /** data direction A, does not matter */
            e6821.direction_A = data;
        }
    } break;
    case 0b01:{
        e6821.control_A.byte = data;
    } break;
    case 0b10:{
        if(e6821.control_B.bits.DDR_access)
        {
            /** data B */
            e6821.data_B = (data & e6821.direction_B) | (e6821.data_B & (~e6821.direction_B));
            if(e6821.callbacks.write_to_device_B)
            {
                e6821.callbacks.write_to_device_B(data,e6821.callbacks.ctx_B);
            }
        }
        else
        {
            /** data direction B, does not matter */
            e6821.direction_B = data;
        }
    } break;
    case 0b11:{
        e6821.control_B.byte = data;
    } break;
    default:
        break;
    }
}

/**
 * @param rs bit1: rs1, bit0: rs0
 */
static inline __attribute__((always_inline)) uint8_t e6821_read(int rs)
{
    switch (rs)
    {
    case 0b00:{
        if(e6821.control_A.bits.DDR_access)
        {
            /** data A */
            e6821.control_A.bits.IRQ1 = 0;
            e6821.control_A.bits.IRQ2 = 0;
            return e6821.data_A;
        }
        else
        {
            /** data direction A, does not matter */
            return e6821.direction_A;
        }
    } break;
    case 0b01:{
        /** control A */
        return e6821.control_A.byte;
    } break;
    case 0b10:{
        if(e6821.control_B.bits.DDR_access)
        {
            /** data B */
            e6821.control_B.bits.IRQ1 = 0;
            e6821.control_B.bits.IRQ2 = 0;
            return e6821.data_B;
        }
        else
        {
            /** data direction B, does not matter */
            return e6821.direction_B;
        }
    } break;
    case 0b11:{
        /** control B */
        return e6821.control_B.byte;
    } break;
    default:
        break;
    }
}

static inline __attribute__((always_inline)) void e6821_input_from_device(e6821_port_t port, uint8_t data)
{
    if(port == E6821_PORT_A)
    {
        e6821.data_A = (data & (~e6821.direction_A)) | (e6821.data_A & e6821.direction_A);
    }
    else if(port == E6821_PORT_B)
    {
        e6821.data_B = (data & (~e6821.direction_B)) | (e6821.data_B & e6821.direction_B);
    }
}

static inline __attribute__((always_inline)) void e6821_set_irq(e6821_port_t port, e6821_irq_line_t line)
{
    if(port == E6821_PORT_A)
    {
        if(line == E6821_IRQ_LINE_1)
        {
            e6821.control_A.bits.IRQ1 = 1;
        }
        else if(line == E6821_IRQ_LINE_2 && (e6821.control_A.bits.C2_output==0))
        {
            e6821.control_A.bits.IRQ2 = 1;
        }
    }
    else if(port == E6821_PORT_B)
    {
        if(line == E6821_IRQ_LINE_1)
        {
            e6821.control_B.bits.IRQ1 = 1;
        }
        else if(line == E6821_IRQ_LINE_2 && (e6821.control_B.bits.C2_output==0))
        {
            e6821.control_B.bits.IRQ2 = 1;
        }
    }
}

/**
 * Simple (higher resolution) sleep function 
 */
static inline __attribute__((always_inline)) void SLEEP_WAIT_FOR_PAGE_CHANGE()
{
    /** 11 nops + 9 op cycles ~150ns */
    __asm volatile (
        "nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"
        "nop\n\t"
    );
}

static inline __attribute__((always_inline)) void SLEEP_HALF_CYCLE()
{
    /** 67 nops ~500ns */
    __asm volatile (
        "nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"
        "nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"
        "nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"
        "nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"
        "nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"
        "nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"
        "nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"
    );
}

/**
 * function pre define
 */
static void terminal_putchar(uint8_t data ,void* ctx);
static void memory_task(void);

/**
 * memory
 */
static uint8_t memory[0x10000] = {0};


/** task running on core 0 */
int __not_in_flash_func(main)()
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
        gpio_set_slew_rate(ADDR_PAGE_PIN,GPIO_SLEW_RATE_FAST);
        gpio_put(ADDR_PAGE_PIN,0);
        /** set button pin to input pull up */
        gpio_init(BUTTON_PIN);
        gpio_set_dir(BUTTON_PIN,false);
        gpio_pull_up(BUTTON_PIN);
    }
    printf("6502 CPU starting...\n");

    /** start memory task on core 1 */
    multicore_launch_core1(memory_task);
    /** handle IO */
    for(;;)
    {
        /** check message from core 1 (terminal output) */
        bool has_ack = false;
        if(sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS)
        {
            uint32_t core1_data =  sio_hw->fifo_rd;
            core1_data &= 0x3F;
            int c_out = (core1_data==0x0D)?'\n':(core1_data&0x20)?core1_data:core1_data+0x40;
            printf("%c",c_out);
            has_ack = true;
        }
        /** check stdin input */
        int c_in = getchar_timeout_us(0);
        if(c_in >= 0)
        {
            /** convert to higher case */
            if(c_in>='a'&&c_in<='z')
                c_in = c_in - ('a'-'A');
            /** keyboard interface always set bit 7 to 1 */
            c_in = c_in | 0x80;
        }
        /** send message to core 1 if any */
        if(has_ack || (c_in >= 0))
        {
            /** has message to send */
            uint32_t message = 0;
            if(has_ack)
                message |= CORE1_HAS_ACK;
            if(c_in>=0)
                message |= CORE1_HAS_DATA | (c_in & CORE1_DATA_MASK);
            sio_hw->fifo_wr = message;
            __asm volatile ("sev");
        }
    }

    return 0;
}



/** 
 * task running on core 1
 *  - RAM
 *  - ROM
 *  - PIA logic
 */
static void __not_in_flash_func(memory_task)(void)
{
reset:
    /** reset system */
    {
        /** reset data pin to input */
        gpio_set_dir_in_masked(DATA_PIN_MASK);
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
        SLEEP_HALF_CYCLE();
        /** wait for at least 2 clock cycles */
        for(int i=0;i<3;i++)
        {
            gpio_put(CLK_PIN,1);
            SLEEP_HALF_CYCLE();
            gpio_put(CLK_PIN,0);
            SLEEP_HALF_CYCLE();
        }
        /** RST -> high */
        gpio_put(RST_PIN,1);
        SLEEP_HALF_CYCLE();
    }
    
    /** run clock cycles */
    for(;;)
    {
        /** read first addr page and settings */
        uint32_t v = gpio_get_all();
        gpio_set_mask(1<<ADDR_PAGE_PIN); /** switch page ASAP */
        /** check message from core 0 */
        if(__builtin_expect(sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS,0))
        {
            uint32_t core0_data = sio_hw->fifo_rd;
            if(core0_data & CORE1_HAS_DATA)
            {
                e6821_input_from_device(E6821_PORT_A,core0_data&CORE1_DATA_MASK);
                e6821_set_irq(E6821_PORT_A,E6821_IRQ_LINE_1);
            }
            else
            {
                /** CORE1_HAS_ACK */
                e6821_input_from_device(E6821_PORT_B,0x00);
            }  
        }
        /** check button */
        if(__builtin_expect(!(v&(1u<<BUTTON_PIN)),0))
            goto reset;
        /** get address & r/w */
        uint16_t addr = 0;
        uint32_t is_read = 0;
        addr = v & 0xFF;
        is_read = v & (1<<RW_PIN);
        /** at least 9 cycles past since page switching */
        /** wait for address page switching 74HC157@2V 145ns, @4.5V 29ns, @3.3V ?ns */
        SLEEP_WAIT_FOR_PAGE_CHANGE(); 
        /** read second addr page */   
        v = gpio_get_all();
        addr = addr | ((v&0xFF) << 8);
        /** handle memory read */
        if(is_read)
        {
            gpio_set_dir_out_masked(DATA_PIN_MASK);
            if(__builtin_expect((addr & 0xF000) == 0xD000,0))
            {
                /** PIA */
                gpio_put_masked(DATA_PIN_MASK,((uint32_t)e6821_read(addr&0b11))<<8);
            }
            else
            {
                gpio_put_masked(DATA_PIN_MASK,((uint32_t)memory[addr])<<8);
            }
        }
        /** put operations between data write and clk change for data to be valid */
        /** switch back address page */
        gpio_clr_mask(1<<ADDR_PAGE_PIN); 
        /** send clk pos edge */
        gpio_set_mask(1<<CLK_PIN);
        SLEEP_HALF_CYCLE();
        /** handle memory write */
        if(is_read==0)
        {
            v = gpio_get_all();
            uint8_t data = (v>>8) & 0xFF;
            if(__builtin_expect((addr & 0xF000) == 0xD000,0))
            {
                /** PIA */
                e6821_write(addr&0b11,data);
            }
            else if(__builtin_expect(addr < 0x8000,1))
            {
                /** RAM */
                memory[addr] = data;
            }
        }
        /** send clk neg edge */
        gpio_clr_mask(1<<CLK_PIN);
        SLEEP_HALF_CYCLE();
        /** reset data pin to input */
        gpio_set_dir_in_masked(DATA_PIN_MASK);
    }
}

/**
 * Callback from core 1
 */
static void __not_in_flash_func(terminal_putchar)(uint8_t data ,void* ctx)
{
    /** send data to core 0 */
    sio_hw->fifo_wr = (uint32_t)data;
    __asm volatile ("sev");
}

