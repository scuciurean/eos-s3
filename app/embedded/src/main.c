#include <string.h>

#include "Fw_global_config.h"

#include "s3x_clock_hal.h"
#include "s3x_clock.h"

#include "eoss3_dev.h"
#include "eoss3_hal_gpio.h"
#include "eoss3_hal_pad_config.h"
#include "eoss3_hal_fpga_sdma_api.h"
#include "eoss3_hal_fpga_sdma_reg.h"
#include "eoss3_hal_sdma.h"

#include "top_bit.h"

#include "fpga_loader.h"


#define PERIPH_BASE           (0x40000000)
#define FPGA_PERIPH_BASE      (PERIPH_BASE + 0x00020000)

#define GPIO_LED_BLUE_BASE_ADDR     (FPGA_PERIPH_BASE + 0x1000)
#define GPIO_LED_GREEN_BASE_ADDR    (FPGA_PERIPH_BASE + 0x2000)
#define PWM_LED_RED_BASE_ADDR       (FPGA_PERIPH_BASE + 0x3000)
#define PWM_PMOD_CNV_BASE_ADDR      (FPGA_PERIPH_BASE + 0x4000)
#define PWM_PMOD_SCK_BASE_ADDR      (FPGA_PERIPH_BASE + 0x5000)
#define AD7984_BASE_ADDR            (FPGA_PERIPH_BASE + 0x6000)

#define GPIO_LED_BLUE_REG0          (GPIO_LED_BLUE_BASE_ADDR + 0)
#define GPIO_LED_BLUE_REG1          (GPIO_LED_BLUE_BASE_ADDR + 0x4)

#define GPIO_LED_GREEN_REG0         (GPIO_LED_GREEN_BASE_ADDR + 0)
#define GPIO_LED_GREEN_REG1         (GPIO_LED_GREEN_BASE_ADDR + 0x4)

#define PWM_LED_RED_REG_CTRL        (PWM_LED_RED_BASE_ADDR + 0)
#define PWM_LED_RED_REG_DUTY        (PWM_LED_RED_BASE_ADDR + 0x4)
#define PWM_LED_RED_REG_PERIOD      (PWM_LED_RED_BASE_ADDR + 0x8)

#define PWM_PMOD_CNV_REG_CTRL        (PWM_PMOD_CNV_BASE_ADDR + 0)
#define PWM_PMOD_CNV_REG_DUTY        (PWM_PMOD_CNV_BASE_ADDR + 0x4)
#define PWM_PMOD_CNV_REG_PERIOD      (PWM_PMOD_CNV_BASE_ADDR + 0x8)

#define PWM_PMOD_SCK_REG_CTRL        (PWM_PMOD_SCK_BASE_ADDR + 0)
#define PWM_PMOD_SCK_REG_DUTY        (PWM_PMOD_SCK_BASE_ADDR + 0x4)
#define PWM_PMOD_SCK_REG_PERIOD      (PWM_PMOD_SCK_BASE_ADDR + 0x8)

#define AD7984_REG_CFG              (AD7984_BASE_ADDR + 0)
#define AD7984_REG_DATA             (AD7984_BASE_ADDR + 0x4)
#define AD7984_REG_BUFF_NUM_SAMPLES (AD7984_BASE_ADDR + 0x8)
void qorc_hardwareSetup(void);

void SystemInit(void)
{

}

static void* sdma_ch0_handle;

static void sdma_irq_handler()
{

  NVIC_ClearPendingIRQ(FbMsg_IRQn);
}

void sdma_callback(void *handle)
{
    return;
}


void pwm_config_signals()
{
    uint64_t clk_freq = 12*1000*1000;
    uint64_t clk_period = 1*1000*1000*1000 / clk_freq;

    uint64_t period_250_khz = 4000; //nanoseconds
    uint64_t duty = 83 * 6; //498 ns almost 500 max

    uint32_t pwm_period = (period_250_khz / clk_period);
    uint32_t pwm_duty = (duty / clk_period);

    *(uint32_t *)(PWM_PMOD_CNV_REG_DUTY) = pwm_duty * 1;
    *(uint32_t *)(PWM_PMOD_CNV_REG_PERIOD) = pwm_period * 1;
    *(uint32_t *)(PWM_PMOD_CNV_REG_CTRL) = 1;

    *(uint32_t *)(PWM_PMOD_SCK_REG_DUTY) = 1 * 1;
    *(uint32_t *)(PWM_PMOD_SCK_REG_PERIOD) = 2* 1;
    *(uint32_t *)(PWM_PMOD_SCK_REG_CTRL) = 1;
}

void sdma_config()
{
    /* Clear any pending interrupt of FPGA */
    NVIC_ClearPendingIRQ(FbMsg_IRQn);

    /* NVIC level interrupt enable */
    NVIC_EnableIRQ(FbMsg_IRQn);

    /* ISR registration for FPGA SDMA interrupts. */
    FB_RegisterISR(FB_INTERRUPT_0, sdma_irq_handler);
    FB_ConfigureInterrupt(FB_INTERRUPT_0, FB_INTERRUPT_TYPE_LEVEL,
                          FB_INTERRUPT_POL_LEVEL_HIGH,
                          FB_INTERRUPT_DEST_AP_DISBLE, FB_INTERRUPT_DEST_M4_ENABLE);

    S3x_Clk_Enable (S3X_SDMA_CLK); //temp
}

void fpga_init()
{
    S3x_Clk_Disable(S3X_FB_21_CLK);
    S3x_Clk_Disable(S3X_FB_16_CLK);

    load_fpga(sizeof(axFPGABitStream), axFPGABitStream);
    fpga_iomux_init(sizeof(axFPGAIOMuxInit), axFPGAIOMuxInit);

    S3x_Clk_Enable(S3X_FB_21_CLK);
    S3x_Clk_Enable(S3X_FB_16_CLK);
    S3x_Clk_Set_Rate(S3X_SDMA_CLK, HSOSC_72MHZ);
}

static void nvic_init(void)
 {
    NVIC_SetPriority(Ffe0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(SpiMs_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(CfgDma_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(Uart_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(FbMsg_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
 }

void init()
{
    qorc_hardwareSetup();
    fpga_init();
    nvic_init();
    pwm_config_signals();
    sdma_config();
}

void demo1()
{

    uint32_t data;
    //togle m4 led gpio
	IO_MUX->PAD_22_CTRL = 0x103;
	MISC_CTRL->IO_OUTPUT |= (0x40);
	IO_MUX->PAD_18_CTRL = 0x103;
	MISC_CTRL->IO_OUTPUT |= 0x10;

    *(uint32_t *)(PWM_LED_RED_REG_DUTY) = 0xFFFC;
    data = *(volatile uint32_t *)(PWM_LED_RED_REG_DUTY);

    *(uint32_t *)(PWM_LED_RED_REG_PERIOD) = 0xFFFFF;
    data = *(uint32_t *)(PWM_LED_RED_REG_PERIOD);

    *(uint32_t *)(PWM_LED_RED_REG_CTRL) = 1;
    data = *(uint32_t *)(PWM_LED_RED_REG_CTRL);

    *(uint32_t *)(GPIO_LED_BLUE_REG0) = 1;
    *(uint32_t *)(GPIO_LED_GREEN_REG0) = 1;


    while(true)
    {
        *(uint32_t *)(AD7984_REG_BUFF_NUM_SAMPLES) = 0x4 ;
        data = *(volatile uint32_t *)(AD7984_REG_BUFF_NUM_SAMPLES);


        *(uint32_t *)(AD7984_REG_CFG) = 0x3;
        data = *(volatile uint32_t *)(AD7984_REG_CFG);
        *(uint32_t *)(AD7984_REG_CFG) = 0;
        data = *(volatile uint32_t *)(AD7984_REG_CFG);
        data = *(volatile uint32_t *)(AD7984_REG_DATA);

        data = *(volatile uint32_t *)(0x40030000);
    }
}

int main()
{
    uint32_t buff[1000];
    uint32_t data;

    init();

#if 0

    *(uint32_t *)(AD7984_REG_CFG) = 0x1;
    while (true){};
#endif

    HAL_StatusTypeDef status = HAL_OK;
    // memset((void*)buff,1,40);
    S3x_Clk_Set_Rate(S3X_SDMA_CLK, HSOSC_80MHZ);

	PMU->MISC_SW_WU		= PMU_MISC_SW_WU_SDMA_WU;
	PMU->FFE_FB_PF_SW_WU = PMU_FFE_FB_PF_SW_WU_FFE_WU;
	CRU->C01_CLK_GATE = 0x2DB;
    CRU->C08_X4_CLK_GATE = 0x1;
    CRU->C08_X1_CLK_GATE = 0xF;
	SDMA_BRIDGE->DMA_REQ	= 0;
	SDMA->CHNL_ENABLE_CLR	= 0xFFFFFFFFL;
	SDMA->DMA_CFG			= 0;


    uint32_t *srcPtr = (uint32_t*)AD7984_REG_DATA;
    int n = 1000;
    while(1){
        SDMA_ch_cfg_t chCfg = {
            .data_size = SDMA_XFER_WORD,
            .src_addr_inc = SDMA_ADDR_WIDTH_NOINC,
            .dest_addr_inc = SDMA_ADDR_WIDTH_WORD,
            .rpowerCode = 0
        };
        status = HAL_SDMA_xfer(SDMA_SRAM_CH12, srcPtr, buff, n, &chCfg);

        *(uint32_t *)(AD7984_REG_BUFF_NUM_SAMPLES) = n;
        *(uint32_t *)(AD7984_REG_CFG) = 0x3;
        n++;
    }
    return 0;
}