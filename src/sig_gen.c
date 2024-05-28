////////////////////////////////////////////////////////////////////////
//      Raspberry Pi Pico Oscilloscope Demo/Training Utility V1.1     //
//      Created by Paul Wasserman pwproj24@gmail.com                  //
//      This utility utilizes routines from the                       //
//      Raspberry Pi Pico examples which are Copyrighted (c)          //
//      2023 Raspberry Pi (Trading) Ltd.                              //
////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/multicore.h"
#include "pico/rand.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"


#include "sin_tbl.h"        // Sine Wave Table 
#include "fwr_sin_tbl.h"    // Fully Rectified Sine Wave Table
#include "manchester_encoding.pio.h"

////////////////////////////////////////////////////////////////////////
////////////////////// Debug Print Functions ///////////////////////////
////////////////////////////////////////////////////////////////////////
int debug_printing = 0;
uint16_t loop_count = 0;
void print_spi_regs(int spi_num)
{
    if(spi_num == 1)
    {
        printf("SPI1 - CR0 %4x CR1 %4x SR %4x\n", 
            *(&spi_get_hw(spi1)->cr0),
            *(&spi_get_hw(spi1)->cr1),
            *(&spi_get_hw(spi1)->sr)     );
    }
    if(spi_num == 0)
    {
        printf("SPI0 - CR0 %4x CR1 %4x SR %4x\n", 
            *(&spi_get_hw(spi0)->cr0),
            *(&spi_get_hw(spi0)->cr1),
            *(&spi_get_hw(spi0)->sr)     );
    }
}

void print_dma_regs(char *stg , uint32_t channel)
{
    printf("%s", stg);
    uint32_t * ba = (uint32_t *)(0x50000000 + channel*0x40);
    printf("%x ", *(ba) );
    printf("%x ", *(ba+1) );
    printf("%x ", *(ba+2) );
    printf("%x ", *(ba+3) );
    printf("\n");
}


////////////////////////////////////////////////////////////////////////
//////////// Initialize the system clock to 120 Mhz ////////////////////
////////////////////////////////////////////////////////////////////////
void init_system_clock()
{
    set_sys_clock_khz(120000, true); // Which slows the clock a little
                                     // But allows 10 MHz for PWM to be
                                     // Exact divider
}

////////////////////////////////////////////////////////////////////////
//////////////////// Initialize the LED ////////////////////////////////
////////////////////////////////////////////////////////////////////////
const uint LED_PIN = PICO_DEFAULT_LED_PIN;
void init_LED()
{   
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

////////////////////////////////////////////////////////////////////////
/////////////////// Power Supply Initialization ////////////////////////
////////////////////////////////////////////////////////////////////////
void reduce_power_supply_ripple()
{
    gpio_init(23);               // GPIO23 sets the Operating mode of the
    gpio_set_dir(23, GPIO_OUT);  // power supply. A 1 => PWM mode which
    gpio_put(23, 1);             // improves ripple but might lower
}                                // efficiency at light loads

                               
////////////////////////////////////////////////////////////////////////
///////// Initialize the Master SPI peripheral and its DMA /////////////                              
////////////////////////////////////////////////////////////////////////
int spi_enabled = 0;
#define Master_TX_DMA_BUFF_SIZE 16 // Bytes
uint8_t master_txbuf[Master_TX_DMA_BUFF_SIZE];
uint master_dma_tx;
dma_channel_config tx_dcc;

void init_SPI_and_its_DMA()
{    
    
    // SPI0  which will be the master (Used to transmit data) 
    // Enable SPI at 1 MHz and connect to GPIOs 
    spi_init(spi0, 1000000); 
    gpio_set_function(5, GPIO_FUNC_SPI);    // GP5 - pin 7 will be chip select
    gpio_set_function(7, GPIO_FUNC_SPI);    // GP7 - pin 10 will be TX MOSI
    gpio_set_function(6, GPIO_FUNC_SPI);    // GP6 - pin 9 will be CLK
    gpio_set_function(8, GPIO_FUNC_SPI);    // GP8 - pin 11 will be RX MISO (Not really used here)

    if(debug_printing) printf("TX buffer  %x\n", master_txbuf);
    
    ////////////////////////////////////////////////////////////////////
      
    master_dma_tx = dma_claim_unused_channel(true);
    tx_dcc = dma_channel_get_default_config(master_dma_tx);
    channel_config_set_transfer_data_size(&tx_dcc, DMA_SIZE_8);
    int tx_dreq = spi_get_dreq(spi0, true); // True for TX
    
    if(debug_printing) printf("TX DREQ = %d TX_DCC = %x\n", tx_dreq, &tx_dcc);
    
    channel_config_set_dreq(&tx_dcc, tx_dreq);  
} 

////////////////////////////////////////////////////////////////////////
//////////////// Second Core/Coprocessor Operations ////////////////////
///////////// Supports PWM4 Statistical Signal Generation //////////////
////////////////////////////////////////////////////////////////////////
uint32_t core1_stack[128];
uint16_t core1_cmd = 0;
uint16_t core1_data1 = 0;
uint16_t core1_data2 = 300;

void core1_main()
{
    uint64_t sum;
    
    // Perform Initialization
    // Initialize the random number generation hardware
    get_rand_32();
    while(1)
    {
        if( core1_cmd != 0 )
        {
            // Process the command
            
            // Currently the only command is to
            // Generate a normally distributed random sample
            // and a uniformly distributed random sample
            
            // Use Central Limit Theorum to build an approximately
            // normally distributed sample
            sum = 0;
            for(int k=0; k<6; k++)
            {
                sum += get_rand_32();
            }
            sum = sum / 6;
            // Store Normal sample in data1
            // The 44 center the 512 wide distribution in the PWM 0-599
            // space.   600-512 = 88     /2 -> 44
            core1_data1 =  (sum >> (32-9)) + 44;
            // Store Uniform sample in data2
            core1_data2 = (get_rand_32() & 0x1ff) + 44;
            core1_cmd = 0; // Ack by setting command back to 0
        }
        sleep_ms(1); 
    }
}


////////////////////////////////////////////////////////////////////////    
///////////////////////// PWM4 DAC Signal Generator ////////////////////
////////////////////////////////////////////////////////////////////////

// PWM4 Used to make a DAC with an RC filter.
// Refer to - Stephen Woodward 2017 EDN article on PWM
// DAC Ripple Reduction for details.


// GP15 Pin 20    -----VVVVVVV-------
//                       1K         |
//                                  |
//                               -------
//                     0.051uf   -------
//                                  |
//                                  |          
// GP14 Pin 19   -----VVVVVVV-------O----------  GP26 Pin 31 ADC0
//                      1K          |
//                                  |
//                               -------
//                     0.015uf   -------
//                                  |
//                                  |
// GND Pin 18  ----------------------

uint slice_num_3;     // For PWM4

#define PWM_RAMP 0
int ramp_value = 0;
int ramp_repeat = 0;


#define PWM_SAW 1
int saw_value = 0;
int saw_delta = 1;    // May be +1 or -1


#define PWM_STAIRCASE 2
int staircase_value = 0;
int staircase_repeat = 0;


#define PWM_SINE 3
int sine_value = 0;
uint16_t stv;


#define PWM_FWR_SINE 4
int fwr_sine_repeat = 0;
uint16_t fwrtv;


#define PWM_GAUSSIAN_STAIRCASE 5
uint16_t gaussian_staircase_value = 0;
uint16_t gaussian_staircase_repeat = 0;
uint16_t gaussian_counter = 0;


#define PWM_UNIFORM_STAIRCASE 6
int uniform_staircase_value = 0;
int uniform_staircase_repeat = 0;
unsigned int uniform_counter = 0;
int gsv = 0;
int usv = 0;

#define PWM_DDS_SINE 7
float dds_freq = 1000.0f;
uint32_t dds_inc = (4294967296.0*1000.0/200000.0);
uint32_t dds_acc = 0;
int ddsv = 0;


int  pwm_sig_genID = PWM_RAMP;

// Note this ISR will be executed every 5 microseconds
// It must execute in less time than that.
static void __isr __time_critical_func(pwm_wrap_isr)()
{
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(14));
    switch( pwm_sig_genID )
    {
        case PWM_RAMP:
            ramp_repeat++;
			if(ramp_repeat < 4) break;
            ramp_repeat = 0;
            pwm_set_both_levels (slice_num_3, ramp_value, ramp_value);
            ramp_value++;
            if( ramp_value > 600) ramp_value = 0;
            break;
            // 200,000/(601*4) -> 83.19 Hz
            
        case PWM_SAW:            
            pwm_set_both_levels (slice_num_3, saw_value, saw_value);
            saw_value += saw_delta;
            if( saw_value == 0 )
            {
                saw_delta = 1;
                break;
            }
            if( saw_value == 600 )
                saw_delta = -1;
            break;            
            // 200,000/(600*2)) ->  166.66 Hz
            
        case PWM_STAIRCASE: // Eleven step staircase
            staircase_repeat++;
            if(staircase_repeat < 240) break;
            pwm_set_both_levels (slice_num_3, staircase_value , staircase_value );
            staircase_repeat = 0;
            staircase_value += 60;
            if( staircase_value > 600) staircase_value = 0;
            break;
            // 200,000/(240*11) -> 75.75 Hz
            
        case PWM_SINE:
            stv = sin_tbl[sine_value];
            pwm_set_both_levels (slice_num_3, stv , stv );
            sine_value += 1;
            if( sine_value == 512 )
                sine_value = 0;
            break;
            // 200,000/(512)) ->  390.625 Hz
            
        case PWM_FWR_SINE:
            fwrtv = fwr_sin_tbl[sine_value];
            pwm_set_both_levels (slice_num_3, fwrtv , fwrtv );
            fwr_sine_repeat++;
            if(fwr_sine_repeat < 4) break;
            fwr_sine_repeat = 0;
            sine_value += 1;
            if( sine_value == 512 )
                sine_value = 0;
            break;
            // 200,000/(256*4)) ->  195.31 Hz
                        
        case PWM_GAUSSIAN_STAIRCASE: // Eleven step staircase with one segment
                                     // Being normally distributed
            pwm_set_both_levels (slice_num_3, gsv , gsv);
            if(++gaussian_staircase_repeat < 240) break;
            if( gaussian_staircase_value == 300 )
            {
                gsv = core1_data1;
                core1_cmd = 1; // Request another set of random data
            }
            else
            {
                gsv = gaussian_staircase_value;
            }
            gaussian_staircase_repeat = 0;
            gaussian_staircase_value += 60;
            if( gaussian_staircase_value > 600) 
            {
                gaussian_staircase_value = 0;
            }
            break;
            // 200,000/(240*11) -> 75.75 Hz
            
        case PWM_UNIFORM_STAIRCASE: // Eleven step staircase with one segment
                                    // Being uniformly distributed
            pwm_set_both_levels (slice_num_3, usv , usv);
            if(++uniform_staircase_repeat < 240) break;
            if( uniform_staircase_value == 300 )
            {
                usv = core1_data2;
                core1_cmd = 1; // Request another set of random data
            }
            else
            {
                usv = uniform_staircase_value;
            }
            uniform_staircase_repeat = 0;
            uniform_staircase_value += 60;
            if( uniform_staircase_value > 600) 
            {
                uniform_staircase_value = 0;
            }
            break;   
            // 200,000/(240*11) -> 75.75 Hz


        case PWM_DDS_SINE:
            pwm_set_both_levels (slice_num_3, ddsv, ddsv);            
            dds_acc += dds_inc; 
            ddsv = sin_tbl[dds_acc>>23];
            break;

        default:
            break;
    }
}


////////////////////////////////////////////////////////////////////////
///////////// Initialize Four PWM Signal Generators ////////////////////
////////////////////////////////////////////////////////////////////////
#define NUM_OF_DUTY_CYCLES 5
int GP2_duty_cycle[NUM_OF_DUTY_CYCLES] = {
    600, // 50%
    300, // 25%
    120, // 10%
    60,  //  5%
    12   //  1%
};
int GP2_duty_cycle_index = 0;


uint slice_num_1;   // For PWM2

void init_PWM()
{
    // PWMs 1 and 3 generate 50% duty cycle square waves at 
    // 10Mhz and 1Khz.
    // PWM2 generates 100kHz with duty cycles of 50%, 25%, 10%, 5%, 1%
    // PWM5 generates a 50% duty cycle square wave at 60 Mhz
    
    // They are set and forget.  I.E. no further interaction are needed
    // (other then to change the duty cycle when initiated by the user)
    
    ////////////////////////////////////////////////////////////////////
    gpio_set_function(0, GPIO_FUNC_PWM);    // GP0 - pin 1 will be 
                                            // 10 Mhz square wave
    gpio_set_drive_strength(0, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(0, GPIO_SLEW_RATE_FAST);
    uint slice_num_0 = pwm_gpio_to_slice_num(0);
    pwm_set_wrap(slice_num_0, 11);          // Set period of 12 cycles 
    pwm_set_chan_level(slice_num_0, PWM_CHAN_A, 6); // 50% Duty cycle
    pwm_set_enabled(slice_num_0, true);


    ////////////////////////////////////////////////////////////////////    
    gpio_set_function(2, GPIO_FUNC_PWM);    // GP2 - pin 4 will be 
                                            // 100 Khz square wave
    slice_num_1 = pwm_gpio_to_slice_num(2);
    pwm_set_wrap(slice_num_1, 1199);        // Set period of 1200 cycles  
    pwm_set_chan_level(slice_num_1, PWM_CHAN_A, GP2_duty_cycle[GP2_duty_cycle_index]);
                                           // initially 50% Duty cycle
    pwm_set_enabled(slice_num_1, true);
    
    ////////////////////////////////////////////////////////////////////   
    gpio_set_function(4, GPIO_FUNC_PWM);    // GP4 - pin 6 will be 
                                            // 1 Khz square wave
    uint slice_num_2 = pwm_gpio_to_slice_num(4);
    pwm_set_clkdiv (slice_num_2, 100.f);    // Pre Divide by 100
    pwm_set_wrap(slice_num_2, 1199);        // Set period of 1200 cycles  
    pwm_set_chan_level(slice_num_2, PWM_CHAN_A, 600); // 50% Duty cycle
    pwm_set_enabled(slice_num_2, true);


    ////////////////////////////////////////////////////////////////////
    // The forth PWM will be used to create various time variable signals
    // It will generate an interrupt after each PWM wrap. This interrupt
    // will be used to change the duty cycle and thus build various signals
    ////////////////////////////////////////////////////////////////////

    gpio_set_function(15, GPIO_FUNC_PWM);   // GP15 - pin 20
    gpio_set_drive_strength(15, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(15, GPIO_SLEW_RATE_FAST);

    gpio_set_function(14, GPIO_FUNC_PWM);   // GP14 - pin 19   
    gpio_set_drive_strength(14, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(14, GPIO_SLEW_RATE_FAST);
     
    slice_num_3 = pwm_gpio_to_slice_num(14);
    pwm_set_wrap(slice_num_3, 599);        // 200 KHz  120Mhz/600
    // Initial duty cycle value which will be changes via sw interrupt 
    pwm_set_both_levels (slice_num_3, 300 , 300 ); // 50% Duty cycle
    pwm_set_enabled(slice_num_3, true);
    
    // The low pass filtered PWM DAC output will be wired to
    // ADC0.  This provides a spot to read back the DAC output if desired
    // More importantly it provides an easy access pin to connect a scope
    // probe to.
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);
    
    ////////////////////////////////////////////////////////////////////
    pwm_set_output_polarity (slice_num_3, false, true); 
    pwm_clear_irq(slice_num_3);
    pwm_set_irq_enabled(slice_num_3, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_isr);
    irq_set_enabled(PWM_IRQ_WRAP, true);


    ////////////////////////////////////////////////////////////////////
    // The fifth PWM will generate a 60 Mhz square wave 50% duty cycle.
    // It is also set and forget.
     ////////////////////////////////////////////////////////////////////
    gpio_set_function(10, GPIO_FUNC_PWM);   // GP10 - pin 14 will be
                                            // 60 Mhz square wave
    gpio_set_drive_strength(10, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(10, GPIO_SLEW_RATE_FAST);
    uint slice_num_4 = pwm_gpio_to_slice_num(10);
    pwm_set_wrap(slice_num_4, 1);          // Set period of 2 cycles 
    pwm_set_chan_level(slice_num_4, PWM_CHAN_A, 1); // 50% Duty cycle
    pwm_set_enabled(slice_num_4, true);
}


////////////////////////////////////////////////////////////////////////
////////////////// Initialize a UART and its DMA ///////////////////////
////////////////////////////////////////////////////////////////////////
int uart_enabled = 0;
char uart_tx_message[] = "The quick brown fox jumped over the lazy dog";
int uart_message_size;


void init_UART()
{
    
    uart_message_size = sizeof(uart_tx_message) - 1; // Sizeof includes the null
                                                     // Which does not get transmitted

 
    gpio_set_function(16, GPIO_FUNC_UART); // GP16 - pin 21 will be UART0_TX
    
    uart_init(uart0, 115200);
    uart_set_hw_flow(uart0, false, false);
    //                     Data bits  Stop bits
    uart_set_format(uart0, 8,         2,       UART_PARITY_EVEN);
}



////////////////////////////////////////////////////////////////////////
///////////////// Initialize I2C Master and Slave //////////////////////
////////////////////////////////////////////////////////////////////////
int i2c_tx_enabled = 0;
static const uint I2C_SLAVE_ADDRESS = 0x17;
static const uint I2C_BAUDRATE = 100000;    // 100 kHz
// Slave I2C is I2C0
static const uint I2C_SLAVE_SDA_PIN = 20;   // GP20 - pin 26 -----
static const uint I2C_SLAVE_SCL_PIN = 21;   // GP21 - pin 27 ---  |
// Master I2C is I2C1                                           | |
static const uint I2C_MASTER_SCL_PIN = 19;  // GP19 - pin 25 ---  |
static const uint I2C_MASTER_SDA_PIN = 18;  // GP18 - pin 24 -----


// The slave implements a 256 byte memory. To write a series of bytes,
// the master first writes the memory address, followed by the data. 
// The address is automatically incremented for each byte transferred, 
// looping back to 0 upon reaching the end. Reading is done
// sequentially from the current memory address.
static struct
{
    uint8_t mem[256];
    uint8_t mem_address;
    bool mem_address_written;
} context;

// Our handler is called from the I2C ISR, so it must complete quickly. 
// Blocking calls/printing to stdio may interfere with interrupt handling.
static void __isr __time_critical_func(i2c_slave_handler)(i2c_inst_t *i2c,
                                                  i2c_slave_event_t event)
{
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        if (!context.mem_address_written) {
            // writes always start with the memory address
            context.mem_address = i2c_read_byte_raw(i2c);
            context.mem_address_written = true;
        } else {
            // save into memory
            context.mem[context.mem_address] = i2c_read_byte_raw(i2c);
            context.mem_address++;
        }
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        // load from memory
        i2c_write_byte_raw(i2c, context.mem[context.mem_address]);
        context.mem_address++;
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        context.mem_address_written = false;
        break;
    default:
        break;
    }
}

char no_snp_msg[32];
static void i2c_setup_slave_and_master() 
{
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
    
    gpio_init(I2C_MASTER_SDA_PIN);
    gpio_set_function(I2C_MASTER_SDA_PIN, GPIO_FUNC_I2C);
    // pull-ups are already active on slave side, this is just a fail-safe in case the wiring is faulty
    gpio_pull_up(I2C_MASTER_SDA_PIN);

    gpio_init(I2C_MASTER_SCL_PIN);
    gpio_set_function(I2C_MASTER_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_MASTER_SCL_PIN);

    i2c_init(i2c1, I2C_BAUDRATE);
    
    for(int i=0;i<32;i++)
        no_snp_msg[i] = 0;
    strcpy(no_snp_msg, "Hello, I2C slave! - 0x00");
}


static uint8_t mem_address = 0;

static void i2c_run_master() 
{
    // char msg[32];
    // snprintf(msg, sizeof(msg), "Hello, I2C slave! - 0x%02X", mem_address);
    
    // Made faster and removed snprintf which is thought to be blocking
    // and thus could interfere with the tight ISR timing for PWM4.
    
    switch (mem_address>>4)
    {
        case 0:      no_snp_msg[22] = '0';  break;            
        case 2:      no_snp_msg[22] = '2';  break;     
        case 4:      no_snp_msg[22] = '4';  break; 
        case 6:      no_snp_msg[22] = '6';  break;   
        case 8:      no_snp_msg[22] = '8';  break;     
        case 10:     no_snp_msg[22] = 'A';  break;     
        case 12:     no_snp_msg[22] = 'C';  break;     
        case 14:     no_snp_msg[22] = 'E';  break;     
    }

    uint8_t msg_len = 24;

    uint8_t buf[32];
    buf[0] = mem_address;
    memcpy(buf + 1, no_snp_msg, msg_len);
    // write message at mem_address
    if(debug_printing) printf("Write at 0x%02X: '%s'\n", mem_address, no_snp_msg);
    int count = i2c_write_blocking(i2c1, I2C_SLAVE_ADDRESS, buf, 1 + msg_len, false);
    if (count < 0) {
        puts("Couldn't write to slave, please check your wiring!");
        return;
    }
    hard_assert(count == 1 + msg_len);

    // seek to mem_address
    count = i2c_write_blocking(i2c1, I2C_SLAVE_ADDRESS, buf, 1, true);
    hard_assert(count == 1);
    // partial read
    uint8_t split = 5;
    count = i2c_read_blocking(i2c1, I2C_SLAVE_ADDRESS, buf, split, true);
    hard_assert(count == split);
    buf[count] = '\0';
    if(debug_printing) printf("Read  at 0x%02X: '%s'\n", mem_address, buf);
    hard_assert(memcmp(buf, no_snp_msg, split) == 0);
    // read the remaining bytes, continuing from last address
    count = i2c_read_blocking(i2c1, I2C_SLAVE_ADDRESS, buf, msg_len - split, false);
    hard_assert(count == msg_len - split);
    buf[count] = '\0';
    if(debug_printing) printf("Read  at 0x%02X: '%s'\n", mem_address + split, buf);
    hard_assert(memcmp(buf, no_snp_msg + split, msg_len - split) == 0);
    if(debug_printing) puts("");
 
    mem_address = (mem_address + 32) % 256;
}

////////////////////////////////////////////////////////////////////////
///////////////// PIO Manchester Encoding TX ///////////////////////////
////////////////////////////////////////////////////////////////////////
int manchester_tx_enabled = 0;
const uint pin_tx = 22;  // GP22 - pin 29
PIO pio = pio0;
uint sm_tx = 0;
static void manchester_encoder_setup()
{ 
    uint offset_tx = pio_add_program(pio, &manchester_tx_program);
    manchester_tx_program_init(pio, sm_tx, offset_tx, pin_tx, 1.f);
    pio_sm_set_enabled(pio, sm_tx, false);
}



////////////////////////////////////////////////////////////////////////
////////////  The main program which executes on core0  ////////////////
////////////////////////////////////////////////////////////////////////
char c;
int UI_on = 1;
int UI_off_count=0;
void do_UI(char lc);
int overclocking = 0;


int main() 
{
    init_system_clock();
    
    stdio_init_all();
    
    // Start Core1 "Coprocessor"
    multicore_launch_core1_with_stack(core1_main, &core1_stack[64], 256);
    
    sleep_ms(5000); // Time to set up the Terminal emulator if desired
    reduce_power_supply_ripple();    
    printf("\n\n\n\n\n\n\n");
    printf("**************************************************************\n");
    printf("* Welcome to the RP Pico Oscilloscope Demo/Training Utility. *\n");
    printf("* Version 1.1                                                *\n");
    printf("* Type h to view the help information.                       *\n");
    printf("**************************************************************\n");
    
    /////////////// Additional Hardware Initialization  ////////////////
    init_LED();
    init_SPI_and_its_DMA();
    init_PWM();
    init_UART();
    i2c_setup_slave_and_master();
    manchester_encoder_setup();
                          
                         
    // Initalize SPI Transmit buffer values
    for(int i=4; i< Master_TX_DMA_BUFF_SIZE; i++)
    {
        master_txbuf[i] = 0x30 + i;
    }
    master_txbuf[0] = 'R';
    master_txbuf[1] = 'P';
    
    ///////////////////  Main Signal Generation Loop ///////////////////
    while (true) 
    {
        loop_count++;
        if(debug_printing) printf("Loop %d\n", loop_count);
        
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        
        ///////////////////////////// SPI //////////////////////////////
        if( spi_enabled )
        { 
        
            master_txbuf[2] = loop_count >> 8;
            master_txbuf[3] = loop_count & 0xff;                       
        
            // Setup SPI Master transaction
            dma_channel_configure(master_dma_tx, &tx_dcc,
                          &spi_get_hw(spi0)->dr,        // write address
                          &master_txbuf[0],             // read address
                          Master_TX_DMA_BUFF_SIZE,      // element count 
                          false);                       // dont start
            if(debug_printing)
            {                   
                printf("Before SPI Started\n");                                     
                print_dma_regs("TX DMA ", master_dma_tx);                                                                          
                print_spi_regs(0);  
                printf("\n");
            }
                          
            //Now start TX DMA
            dma_start_channel_mask(1u << master_dma_tx);

            // Wait for TX DMA to finish
            dma_channel_wait_for_finish_blocking(master_dma_tx); 
                                    
            // But SPI peripheral has fifo so DMA finishes prior to spi 
            // transfer completion.
        
            while( spi_is_busy( spi0 ) ); // So now wait for SPI TX to finish

            if(debug_printing)
            {                           
                printf("After SPI Finished\n");                                      
                print_dma_regs("TX DMA ", master_dma_tx);                                                                           
                print_spi_regs(0);  
                printf("\n");
            }
        
            for(int i=4; i<Master_TX_DMA_BUFF_SIZE; i++)
            {
                master_txbuf[i] = master_txbuf[i] + 1;
            }
        }
 
        ////////////////////////// I2C  ////////////////////////////////
        if( i2c_tx_enabled == 1)
        {
            i2c_run_master();
        }
        
        /////////////// Async Serial Communication  ////////////////////
        if( uart_enabled )
        { 
            // Send the message
            uart_puts(uart0, uart_tx_message);
        }
        
        /////////////// Manchester Encoded TX //////////////////////////
        if( manchester_tx_enabled )
        {
            pio_sm_set_enabled(pio, sm_tx, false);
            // PIO implementation transmits LSB  first
            pio_sm_put_blocking(pio, sm_tx, 0x084215Af);
            pio_sm_put_blocking(pio, sm_tx, 0x0);
            pio_sm_set_enabled(pio, sm_tx, true);
        }
        
        ////////////////////////////////////////////////////////////////
        ///// Check if a command was enterd and if so process it ///////
        ////////////////////////////////////////////////////////////////
        sleep_ms(250);
        
        if(UI_on)
        {
            c=getchar_timeout_us(0);
            do_UI(c);
        }
        else
        {   // UI is off
            UI_off_count++;
            if(UI_off_count >= 120 )
            {
                // Restore UI functionality
                UI_off_count = 0;
                UI_on = 1;
                printf("Restoring UI functionality.\n");
            }
        } 
    }
}
    
void do_UI(char lc )
{
    switch( c )
    {
        case '0':
            printf("Setting PWM_RAMP.\n");
            pwm_sig_genID = PWM_RAMP;
            break;
            
        case '1':
            printf("Setting PWM_SAW.\n");
            pwm_sig_genID = PWM_SAW;
            break;
            
        case '2':
            printf("Setting PWM_STAIRCASE.\n");
            staircase_value = 0;
            staircase_repeat = 0;
            pwm_sig_genID = PWM_STAIRCASE;
            break;
            
        case '3':
            printf("Setting PWM_SINE.\n");
            pwm_sig_genID = PWM_SINE;
            break;
            
        case '4':
            printf("Setting PWM_FWR_SINE.\n");
            pwm_sig_genID = PWM_FWR_SINE;
            break;
            
        case '5':
            printf("Setting PWM_GAUSSIAN_STAIRCASE.\n");
            pwm_sig_genID = PWM_GAUSSIAN_STAIRCASE;
            break;
            
        case '6':
            printf("Setting PWM_UNIFORM_STAIRCASE.\n");
            pwm_sig_genID = PWM_UNIFORM_STAIRCASE;
            break;

         case '7':
            printf("Setting PWM_DDS_SINE.\n");
            pwm_sig_genID = PWM_DDS_SINE;
            break;   
            
        case 't':
            printf("Toggling the debug printouts");
            if( debug_printing )
            {
                debug_printing = 0;
                printf(" - Off.\n");
            }
            else
            {
                debug_printing = 1;
                printf(" - On.\n");
            }
            break;
            
        case 'i':
            printf("Toggling I2C transactions");
            if( i2c_tx_enabled )
            {
                i2c_tx_enabled = 0;
                printf(" - Off.\n");
            }
            else
            {
                i2c_tx_enabled = 1;
                printf(" - On.\n");
            }
            break;
            
        case 's':
            printf("Toggling SPI transactions");
            if( spi_enabled )
            {
                spi_enabled = 0;
                printf(" - Off.\n");
            }
            else
            {
                spi_enabled = 1;
                printf(" - On.\n");
            }
            break;
        case 'u':
            printf("Toggling UART transactions");
            if( uart_enabled )
            {
                uart_enabled = 0;
                printf(" - Off.\n");
            }
            else
            {
                uart_enabled = 1;
                printf(" - On.\n");
            }
            break;
            
        case 'm':
            printf("Toggling Manchester Encoded transmissions");
            if( manchester_tx_enabled )
            {
                manchester_tx_enabled = 0;
                printf(" - Off.\n");
            }
            else
            {
                manchester_tx_enabled = 1;
                printf(" - On.\n");
            }
            break;
            
        case 'c':
            printf("PWM2 Duty Cycle set to ");
            GP2_duty_cycle_index++;
            if( GP2_duty_cycle_index >= NUM_OF_DUTY_CYCLES)
                GP2_duty_cycle_index = 0;
            pwm_set_chan_level(slice_num_1, PWM_CHAN_A, 
                            GP2_duty_cycle[GP2_duty_cycle_index]);
            switch(GP2_duty_cycle_index)
            {    
                case 0:
                    printf("50%%.\n");
                    break;
                case 1:
                    printf("25%%.\n");
                    break;
                case 2:
                    printf("10%%.\n");
                    break;
                case 3:
                    printf("5%%.\n");
                    break;
                case 4:
                    printf("1%%.\n");
                    break;
            }    
            break;
            
        case 'h':
            printf("\nRP Pico Oscilloscope Demo/Training Utility. Version 1.1\n");
            printf("\nh - Show the help information.\n");
            printf("w - Display the pinout.\n");
            printf("t - Toggle Debug printouts. Note Debug printouts effect the \"purity\" of PWM4 signals.\n");
            printf("                            They use printf which is blocking and will interfere with\n");
            printf("                            the timing required for the ISR associated with PWM4 signals.\n");
            printf("i - Toggle I2C Transactions.\n");
            printf("m - Toggle Manchester Transmissions.\n");
            printf("s - Toggle SPI Transactions.\n");
            printf("u - Toggle UART Transactions.\n");
            printf("o - Toggle 2X Overclocking.  Warning functionality not assured. Not responsible for damage!\n");
            printf("d - Displays the current operating status.\n");
            // p not for general use.  Used in debug to see if UI was causing signal distortions.
            // printf("p - Temporarily disables UI functionality.\n");
            printf("c - Cycle to the next PWM2 Duty Cycle.\n");
            printf("0 - Generate a 83.19 Hz ramp signal on PWM4.\n");
            printf("1 - Generate a 166.66 Hz sawtooth signal on PWM4.\n");
            printf("2 - Generate a 75.75 Hz staircase signal on PWM4.\n");
            printf("3 - Generate a 390.625 Hz sinewave signal on PWM4.\n");
            printf("4 - Generate a 195.31 Hz full wave rectified sine signal on PWM4.\n");
            printf("5 - Generate a 75.75 Hz staircase signal with a normally distributed segment on PWM4.\n");
            printf("6 - Generate a 75.75 Hz staircase signal with a uniformly distributed segment on PWM4.\n");
            printf("7 - Generate a frequency adjustable sinewave using Direct Digital Synthesis.\n");
            printf("f - Set the frequency for the DDS sinewave.\n");
            printf("[ - Lower DDS frequency.\n");
            printf("] - Raise DDS frequency.\n");
            printf("\n");
            break;

        case '[':
            dds_freq = dds_freq/1.41421356237f;  // Two presses halfs frequency
            if(dds_freq < 1.0f)
                dds_freq = 1.0f;
            dds_inc = (uint32_t)(4294967296.0*dds_freq/200000.0);
            break;

        case ']':
            dds_freq = dds_freq*1.41421356237f; // Two presses doubles frequency
            if(dds_freq > 1000.0f)
                dds_freq = 1000.0f;
            dds_inc = (uint32_t)(4294967296.0*dds_freq/200000.0);
            break;

        case 'f':
            char str[50];
            int c_index = 0;
            float in_float;
            printf("\nEnter the desired frequency (1.0-1000.0 hz) : ");
            while(c_index<49)
            {
                char c;
                c = getchar();               
                if( (c=='\n')||(c=='\r') )
                {
                    // Line entered
                    str[c_index]=0;
                    break;
                }
                else
                {
                    if( (c=='\b') && (c_index > 0))
                    {   
                        c_index--;
                        printf("\b \b");
                    }
                    if(c!='\b')
                    {
                        printf("%c", c);  // Echo Character
                        str[c_index++]=c;                   
                    }
                }
            }
            sscanf(str,"%f", &in_float);
            if( (in_float<1.0f) || (in_float>1000.0))
            {
                printf("Bad number!\n");
                break;
            }
            // Calculated the DDS increment for this frequency
            dds_inc = (uint32_t)(4294967296.0*in_float/200000.0);
            dds_freq = in_float;
            printf("\n");
            break;

        case 'o':
            if(overclocking == 0)
            {
                overclocking = 1;
                printf("Overclocking sys_clock to 240 Mhz.\n");
                set_sys_clock_khz(240000, true);
            }
            else
            {
                overclocking = 0;
                printf("Restoring sys_clock to 120 Mhz.\n");
                set_sys_clock_khz(120000, true);
            }
            break;

        case 'p':
            printf("Temporarily disabling UI functionality.\n");
            UI_off_count = 0;
            UI_on = 0;
            break;
            
        case 'd':
            printf("\nCurrent Operating Status:\n");
            printf("  PWM[1-4] signal generators are on.\n");
            printf("    PWM1 -> GP0  - pin  1   10.00 Mhz Square Wave 50%% Duty Cycle.\n"); 
            printf("    PWM2 -> GP2  - pin  4  100.00 Khz Square Wave");
            switch(GP2_duty_cycle_index)
            {
                case 0:
                    printf(" 50%%");
                    break;
                case 1:
                    printf(" 25%%");
                    break;
                case 2:
                    printf(" 10%%");
                    break;
                case 3:
                    printf(" 5%%");
                    break;
                case 4:
                    printf(" 1%%");
                    break;
            }
            printf(" Duty Cycle.\n");
            printf("    PWM3 -> GP4  - pin  6    1.00 Khz Square Wave 50%% Duty Cycle.\n"); 
            printf("    PWM4 -> GP14 - pin 19 "); 
            switch( pwm_sig_genID )
            {     
                case PWM_RAMP:
                    printf("  83.19  hz Ramp.\n"); 
                    break;
                case PWM_SAW:
                    printf(" 166.66  hz Sawtooth.\n"); 
                    break;
                case PWM_STAIRCASE:
                    printf("  75.75  hz Staircase.\n");
                    break;
                case PWM_SINE:
                    printf(" 390.62  hz Sine.\n");
                    break;
                case PWM_FWR_SINE:
                    printf(" 195.31  hz Full Wave Rectified Sine.\n");
                    break;
                case PWM_GAUSSIAN_STAIRCASE:
                    printf("  75.75  hz Staircase with a Normally Distributed Segment.\n");
                    break;            
                case PWM_UNIFORM_STAIRCASE:
                    printf("  75.75  hz Staircase with a Uniformly Distributed Segment.\n");
                    break;
                case PWM_DDS_SINE:
                    printf("%7.2f  hz DDS generated Sine.\n", dds_freq);
                    break;
            }
            printf("    PWM5 -> GP10 - pin 14   60.00 Mhz Square Wave 50%% Duty Cycle.\n");
            printf("  Debug printouts are");
            if( debug_printing ) {printf(" - On.\n"); } else { printf(" - Off.\n"); }
            
            printf("  I2C Transactions are");
            if( i2c_tx_enabled ) {printf(" - On.\n"); } else { printf(" - Off.\n"); }
            
            printf("  Manchester Transmissions are");
            if( manchester_tx_enabled ) {printf(" - On.\n"); } else { printf(" - Off.\n"); }
            
            printf("  SPI Transmission are");
            if( spi_enabled ) {printf(" - On.\n"); } else { printf(" - Off.\n"); }
            
            printf("  UART Transmission are");
            if( uart_enabled) {printf(" - On.\n"); } else { printf(" - Off.\n"); }

            if(overclocking == 0)
            {
                printf("  Sys_clock is 120 Mhz.\n");
            }
            else
            {
                printf("  Sys_clock is 240 Mhz.\n");
            }
            break;

        case 'w':
            printf("\n                     Pinout\n\n"                );
            printf("                          S S   S              \n");          
            printf("                          P P   P              \n");
            printf("                          I I   I              \n");
            printf("                                      1        \n");
            printf("          G       6 G       C G C     0 G   1  \n");
            printf("          N       0 N     T L N S 1   0 N   0  \n");
            printf("          D       M D     X K D N K   K D   M  \n");
            printf("\n"                                               );
            printf("      2 1 1 1 1 1 1 1 1 1 1                    \n");
            printf("      0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1  \n");
            printf("    +-----------------------------------------o\n");
            printf("    | SWCLK                                   |\n");
            printf("    | GND                                USB  |\n");
            printf("    | SWDIO                                   |\n");
            printf("    +-----------------------------------------+\n");
            printf("      2 2 2 2 2 2 2 2 2 3 3 3 3 3 3 3 3 3 3 4 \n" );
            printf("      1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0\n"  );
            printf("\n"                                               );
            printf("      U   G I I     G M   P   G         G      \n");
            printf("      A   N 2 2     N A   W   N         N      \n");
            printf("      R   D C C     D N   M   D         D      \n");
            printf("      T               C                        \n");
            printf("            S S       H   D                    \n");
            printf("      T     D C       S   A                    \n");
            printf("      X     A L       T   C                    \n");
            break;

    }
}


