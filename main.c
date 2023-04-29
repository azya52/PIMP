#include "tusb.h"
#include "fatfs/ff.h"
#include "main.h"
#include "stdlib.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/structs/syscfg.h"
#include "hardware/adc.h"
#include "hardware/rosc.h"
#include "hardware/xosc.h"
#include "mk90.pio.h"

FATFS FATFS_Obj;
FIL	appFile;
char fileList[256][12+1];
uint8_t	readBuffer;
PIO pio;
uint32_t sm_tx;
uint32_t sm_rx;

static void pio_irq(void);

static void error_handler(void) {
    while(1) {
        gpio_xor_mask(0x01 << PIN_LED);
        busy_wait_ms(255);
	}
}

static void clocks_initialize(void) {  
    // Enable the xosc
    xosc_init();

    // Before we touch PLLs, switch sys and ref cleanly away from their aux sources.
    hw_clear_bits(&clocks_hw->clk[clk_sys].ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_sys].selected != 0x1) tight_loop_contents();

    pll_init(pll_usb, 1, 768 * MHZ, 4, 4);
    pll_deinit(pll_sys);

    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    clock_configure(clk_usb,
                    0, // No GLMUX
                    CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    clock_configure(clk_adc,
                    0, // No GLMUX
                    CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    clock_stop(clk_ref);
    clock_stop(clk_peri);
    clock_stop(clk_rtc);
    clock_stop(clk_gpout0);
    clock_stop(clk_gpout1);
    clock_stop(clk_gpout2);
    clock_stop(clk_gpout3);

    clocks_hw->wake_en0 = ~(
        //CLOCKS_WAKE_EN0_CLK_SYS_VREG_AND_CHIP_RESET_BITS |
        CLOCKS_WAKE_EN0_CLK_SYS_RESETS_BITS |
        CLOCKS_WAKE_EN0_CLK_SYS_SPI0_BITS |
        CLOCKS_WAKE_EN0_CLK_PERI_SPI0_BITS |
        CLOCKS_WAKE_EN0_CLK_SYS_SPI1_BITS |
        CLOCKS_WAKE_EN0_CLK_PERI_SPI1_BITS |
        CLOCKS_WAKE_EN0_CLK_SYS_ROSC_BITS |
        CLOCKS_WAKE_EN0_CLK_SYS_PLL_SYS_BITS |
        CLOCKS_WAKE_EN0_CLK_SYS_PWM_BITS |
        CLOCKS_WAKE_EN0_CLK_SYS_PIO1_BITS |
        CLOCKS_WAKE_EN0_CLK_SYS_JTAG_BITS |
        CLOCKS_WAKE_EN0_CLK_SYS_I2C0_BITS |
        CLOCKS_WAKE_EN0_CLK_SYS_I2C1_BITS |
        CLOCKS_WAKE_EN0_CLK_SYS_RTC_BITS |
        CLOCKS_WAKE_EN0_CLK_RTC_RTC_BITS |
        CLOCKS_WAKE_EN0_CLK_SYS_DMA_BITS
    );

    clocks_hw->wake_en1 = ~(
        CLOCKS_WAKE_EN1_CLK_SYS_WATCHDOG_BITS |
        CLOCKS_WAKE_EN1_CLK_SYS_UART0_BITS |
        CLOCKS_WAKE_EN1_CLK_PERI_UART0_BITS |
        CLOCKS_WAKE_EN1_CLK_SYS_UART1_BITS |
        CLOCKS_WAKE_EN1_CLK_PERI_UART1_BITS |
        CLOCKS_WAKE_EN1_CLK_SYS_TBMAN_BITS |
        CLOCKS_WAKE_EN1_CLK_SYS_TIMER_BITS |
        CLOCKS_WAKE_EN1_CLK_SYS_SYSINFO_BITS
    );
    
}

static void dormant_goto_and_comeback(void) {
    uint32_t intstatus = save_and_disable_interrupts();
    gpio_set_dormant_irq_enabled(PIN_SELECT, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS, true);

    // Turn off flash
    xip_ctrl_hw->ctrl |= XIP_CTRL_POWER_DOWN_BITS;
    // Turn off SRAMs
    syscfg_hw->mempowerdown =
        0x00000020 | //SRAM5
        0x00000010 | //SRAM4
        0x00000080 | //USB
        0x00000040; //ROM

    // Before we touch PLLs, switch sys and ref cleanly away from their aux sources.
    hw_clear_bits(&clocks_hw->clk[clk_sys].ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_sys].selected != 0x1) tight_loop_contents();
    pll_deinit(pll_sys);
    pll_deinit(pll_usb);

    // Sleep
    xosc_dormant();

    // Turn on SRAMs
    syscfg_hw->mempowerdown = 0;
    // Turn on flash
    xip_ctrl_hw->ctrl &= ~XIP_CTRL_POWER_DOWN_BITS;
    // Clear the irq so we can go back to dormant mode again if we want
    gpio_acknowledge_irq(PIN_SELECT, IO_BANK0_DORMANT_WAKE_INTE0_GPIO0_EDGE_LOW_BITS);
    // Restore clocks
    clocks_initialize();
    restore_interrupts(intstatus);
}

static void adc_irq(void) {
    uint16_t value = adc_fifo_get();
    if ((value > 0) && (value < ADC_4V2_THRESH)) {
        dormant_goto_and_comeback();
    }
}

static inline void adc_initialize(void) {
    adc_init();
	adc_gpio_init(29);
	adc_select_input(3);	// start with measurement for GPIO29 - IP Used in ADC mode (ADC3) to measure VSYS/3
	adc_fifo_setup(true, false, 1, true, false);
	irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_irq);
	adc_irq_set_enabled(true);
	irq_set_enabled(ADC_IRQ_FIFO, true);
    irq_set_priority(ADC_IRQ_FIFO, PICO_LOWEST_IRQ_PRIORITY);
	adc_set_clkdiv(65535);	// slowest possible 65535: 832 measurements / sec
	adc_run(true);
}

static inline void gpio_initialize(void) {
    gpio_pull_up(PIN_CLK);
    gpio_pull_up(PIN_SELECT);
    gpio_pull_up(PIN_DATA);

    gpio_init(PIN_SELECT);
    gpio_init(PIN_VBUS);
    gpio_init(PIN_LED);

    gpio_set_dir(PIN_VBUS, GPIO_IN);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_set_dir(PIN_SELECT, GPIO_IN);
}

static inline void pio_initialize(void) {
    pio = pio0;
    sm_tx = pio_claim_unused_sm(pio, true);
    sm_rx = pio_claim_unused_sm(pio, true);
    mk90bus_program_init(pio, sm_tx, sm_rx);
    
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq);
    irq_set_enabled(PIO0_IRQ_0, true);
    pio_set_irq0_source_enabled(pio, pis_sm1_rx_fifo_not_empty, true);
    irq_set_priority(PIO0_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);
}

FRESULT openFile(char *filename) {
    DIR dj;
    FILINFO fno;
    FRESULT fr;

    f_close(&appFile);

    fr = f_opendir(&dj, "");
    if (fr == FR_OK) {
		uint32_t i = 0;
		do {
			fr = f_readdir(&dj, &fno);
			if (fr == FR_OK && (strcmp(fno.fname, "AUTORUN.BIN") != 0) && !(fno.fattrib & AM_DIR)) {
				strcpy(fileList[i++], fno.fname);
			}
		} while (fr == FR_OK && i < 255 && fno.fname[0] != 0);
		f_closedir(&dj);
    }

    if (filename != NULL) {
    	fr = f_open(&appFile, filename, FA_READ);
    }

    if (fr != FR_OK && fileList[0]) {
    	fr = f_open(&appFile, fileList[0], FA_READ);
    }
 
    if (fr == FR_OK) {
        uint32_t lktbl[256];
		lktbl[0] = 256;
        appFile.cltbl = lktbl;
		fr = f_lseek(&appFile, CREATE_LINKMAP);
	}

    return fr;
}

static void remount(void) {
    if (f_mount(&FATFS_Obj, "0", 1) == FR_OK) {
        openFile("autorun.bin");
    }
}

static inline void parseCommand(uint32_t command) {
    switch(command) {
        case 0xD8:
        case 0xD0: {// read postincrement
                do {
                    pio_sm_put(pio, sm_tx, readBuffer << 24);
                    f_read_byte(&appFile, &readBuffer);
                    while (!pio_interrupt_get(pio, SENDED_IRQ)) tight_loop_contents();
                } while(!gpio_get(PIN_SELECT));
                f_lseek(&appFile, appFile.fptr-2);
                f_read_byte(&appFile, &readBuffer);
            }
            break;

        case 0x00: // query status
            pio_sm_put(pio, sm_tx, 0x00 << 24);
            while (!pio_interrupt_get(pio, SENDED_IRQ) || !gpio_get(PIN_SELECT)) tight_loop_contents();
            break;

        case 0xA8: // set 24b address
            f_lseek(&appFile, mk90bus_read24(pio, sm_rx));
            f_read_byte(&appFile, &readBuffer);
            break;

        case 0xA0: // set 16b address
            f_lseek(&appFile, mk90bus_read16(pio, sm_rx));
            f_read_byte(&appFile, &readBuffer);
            break;

        case 0xF0: {
            uint32_t i = 0;
            while (fileList[i][0]) {
                uint32_t j = 0;
                do {
                    pio_sm_put(pio, sm_tx, fileList[i][j] << 24);
                    while (!pio_interrupt_get(pio, SENDED_IRQ)) tight_loop_contents();
                } while (fileList[i][j++]);
                i++;
            }
            pio_sm_put(pio, sm_tx, 0xFF << 24);
            while (!pio_interrupt_get(pio, SENDED_IRQ) || !gpio_get(PIN_SELECT)) tight_loop_contents();
            }
            break;

        case 0xF1: {
                char filename[12+1] = {0,};
                uint32_t i = 0;
                do {
                    filename[i] = mk90bus_read8(pio, sm_rx);
                } while (filename[i] && ++i < 13);
                if (openFile(filename) != FR_OK) {
                    error_handler();
                }
            }
            break;

        default: // unknown command
            error_handler();
            break;
    }
}

static void pio_irq(void) {   
    parseCommand(mk90bus_read8(pio, sm_rx));
    pio_sm_clear_fifos(pio, sm_rx);
    irq_clear(PIO0_IRQ_0);
}

int main(void) {
    gpio_initialize();
    rosc_disable();

    //Disable XIP Cache
	xip_ctrl_hw->ctrl &= ~XIP_CTRL_EN_BITS;

    //Sleep if USB VBUS isn't present
    if (!gpio_get(PIN_VBUS)) {
        dormant_goto_and_comeback();
    } else {
        clocks_initialize();
    }
    adc_initialize();
    pio_initialize();
    tusb_init();   
    remount();

    while (1) {
        __wfi();
        tud_task(); // tinyusb device task
    }
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void) itf;
    // connected
    if (dtr && rts) {
        tud_cdc_write_str("\r\nPIMP v1.0\r\n");
        tud_cdc_write_flush();
    }
}

uint32_t mount_str_pos = 0;
// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf) {
    while (tud_cdc_available()) {
        char chr = tud_cdc_n_read_char(itf);
        tud_cdc_write_char(chr);
        if ("MOUNT"[mount_str_pos++] != chr) {
            mount_str_pos = 0;
        }
        if (mount_str_pos == strlen("MOUNT")) {
            remount();
            tud_cdc_write_str("\r\nOK\r\n");
            mount_str_pos = 0;
        }
        tud_cdc_write_flush();
    }   
}