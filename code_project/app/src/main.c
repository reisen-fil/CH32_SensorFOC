/**
 * @file main.c
 * @brief 应用程序入口与主循环调度。
 * @details 负责系统初始化并周期调用应用系统控制处理和空闲处理接口。
 */
#include "app_system.h"

#include "bsp_led.h"
#include "bsp_encoder.h"
#include "bsp_motor_ctrl.h"
#include "bsp_param_store.h"
#include "bsp_current_sense.h"
#include "bsp_analog_sense.h"
#include "bsp_comm_mgr.h"

#include "ch32v20x.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define APP_CONTROL_PERIOD_US (1000U)

#define FLASH_BASE_ADDR     (0x08000000)

bsp_encoder_status_enum MT6701_Flag,MT6701_status;
bsp_param_store_status_enum Init_Flag,Write_Flag;

float mech_angle_pu,ele_angle_pu = 0.0f;
float comm_mgr_angle_pu[2]={0};

bsp_comm_mgr_status_enum comm_mgr_rx_flag;
uint16_t comm_mgr_rx_length = 0U;
uint8_t comm_mgr_rx_frame[BSP_COMM_MGR_RX_FRAME_BUFFER_SIZE] = {0};

volatile uint32_t dbg_lock, dbg_eop_erase, dbg_eop_pgm, dbg_pgerr, dbg_rb, dbg_wpr, dbg_rpr, dbg_cfgr0, dbg_lock_1;

int main(void)
{
    // app_system_t app_system;
    // app_system_config_t system_config = {
    //     .control_period_us = APP_CONTROL_PERIOD_US,
    //     .communication_period_us = 10000U,
    //     .p_user_data = NULL
    // };

    // if (app_system_create(&app_system, &system_config) != app_system_ok) {
    //     return -1;
    // }

    // if (app_system_init(&app_system) != app_system_ok) {
    //     return -1;
    // }

    // for (;;) {
    //     (void)app_system_process(
    //         &app_system, system_config.control_period_us);
    //     (void)app_system_idle(&app_system);
    // }

    // LED 状态显示（BSP 层）：bsp_led_init 内部完成 LED 引脚组构建与两灯绑定
    // (void)bsp_led_init(&bsp_led_display);
    // (void)bsp_set_green_led(&bsp_led_display);   /* 状态正常：绿灯亮、红灯灭 */
    // (void)bsp_set_red_led(&bsp_led_display);     /* 状态异常：红灯亮、绿灯灭 */
    // 经 BSP LED 函数表等价调用
    // (void)bsp_led_ops->pf_init(&bsp_led_display);

    // (void)bsp_led_ops->pf_set_green(&bsp_led_display);
    // (void)bsp_led_ops->pf_set_red(&bsp_led_display);
    // 当前显示状态：bsp_led_display.indicator（normal / abnormal）


    /* bsp_encoder test */ 
    // MT6701_status = bsp_encoder_mt6701_ops->pf_init(&bsp_encoder_mt6701);
    // MT6701_Flag = bsp_encoder_mt6701_ops->pf_get_mechanical_angle(&bsp_encoder_mt6701);

    // hal_tim_ops->pf_motor_pwm_config(&hal_tim_motor);

    // hal_tim_ops->pf_set_compare(&hal_tim_motor,1,7500);
    // hal_tim_ops->pf_set_compare(&hal_tim_motor,2,7500);
    // hal_tim_ops->pf_set_compare(&hal_tim_motor,3,15000);

    /* bsp_motor_ctrl test */ 
    // bsp_motor_ctrl_ops->pf_init(&bsp_motor_ctrl_three_phase);
    // bsp_motor_ctrl_ops->pf_set_duty(&bsp_motor_ctrl_three_phase,3000,1500,600);


    /* bsp_param_store test */

    /* 快速编程擦写测试 */ 
    // static uint32_t s_page[64];                  /* 256 字节 = 一个快速页 */

    // for (uint32_t i = 0; i < 64u; i++) { s_page[i] = 0x11111111u; }

    // plat_ch32v203_flash_ops->pf_unlock_fast();
    // dbg_lock = FLASH->CTLR & (1 << 15);         /* bit15 FLOCK 应为 0 */

    // while(!plat_ch32v203_flash_ops->pf_clear_flag(PLAT_FLASH_FLAG_CLEAR_ALL));
    // plat_ch32v203_flash_ops->pf_lock_fast(); 

    // // 擦除256B
    // plat_ch32v203_flash_ops->pf_unlock_fast();
    // while(!plat_ch32v203_flash_ops->pf_erase_page_fast(FLASH_BASE_ADDR+0x2000));
    // dbg_eop_erase = FLASH->STATR & (1 << 5);                             /* EOP(0x20) 是否置 1 */
    // plat_ch32v203_flash_ops->pf_lock_fast();

    // // 写入
    // plat_ch32v203_flash_ops->pf_unlock_fast();
    // while(!plat_ch32v203_flash_ops->pf_program_page_fast(FLASH_BASE_ADDR+0x2000,s_page));
    // dbg_eop_pgm = FLASH->STATR & ((1 << 5) | (1 << 4));                               /* EOP / PGERR(0x04) / WRPRTERR(0x10) */
    // dbg_pgerr   = FLASH->STATR   & 0x04;
    // dbg_rb      = *(volatile uint32_t *)(FLASH_BASE_ADDR+0x2000);               /* 立即回读 */
    // dbg_wpr     = FLASH->WPR;
    // dbg_cfgr0   = RCC->CFGR0;

    // plat_ch32v203_flash_ops->pf_lock_fast();
    // dbg_lock_1 = FLASH->CTLR & (1 << 15);
    // dbg_rpr = FLASH->OBR;

    /* 标准页擦写测试_1 */   
    // plat_ch32v203_flash_ops->pf_unlock();
    // dbg_lock = FLASH->CTLR & (1 << 7);                                   /* bit7 LOCK 应为 0 */
    // // plat_ch32v203_flash_ops->pf_access_clock_cfg(PLAT_FLASH_ACCESS_CLOCK_HCLK_DIV2);

    // while(!plat_ch32v203_flash_ops->pf_clear_flag(PLAT_FLASH_FLAG_CLEAR_ALL));
    // plat_ch32v203_flash_ops->pf_lock();    

    // plat_ch32v203_flash_ops->pf_unlock(); 
    // while(!plat_ch32v203_flash_ops->pf_erase_page(FLASH_BASE_ADDR+0x2000));
    // dbg_eop_erase = FLASH->STATR & (1 << 5);                             /* EOP(0x20) 是否置 1 */
    // plat_ch32v203_flash_ops->pf_lock();

    // plat_ch32v203_flash_ops->pf_unlock();
    // while(!plat_ch32v203_flash_ops->pf_program_halfword(FLASH_BASE_ADDR+0x2000,0x2222));
    // dbg_eop_pgm = FLASH->STATR & ((1 << 5) | (1 << 4));                               /* EOP / PGERR(0x04) / WRPRTERR(0x10) */
    // dbg_pgerr   = FLASH->STATR   & 0x04;
    // dbg_rb      = *(volatile uint32_t *)(FLASH_BASE_ADDR+0x2000);               /* 立即回读 */
    // dbg_wpr     = FLASH->WPR;
    // dbg_cfgr0   = RCC->CFGR0;

    // plat_ch32v203_flash_ops->pf_lock();
    // dbg_lock_1 = FLASH->CTLR & (1 << 7);
    // dbg_rpr = FLASH->OBR;


    /* 标准页擦写测试_2 */
    // plat_ch32v203_flash_ops->pf_unlock();
    // dbg_lock = FLASH->CTLR & (1 << 7);                                   /* bit7 LOCK 应为 0 */

    // while(!plat_ch32v203_flash_ops->pf_clear_flag(PLAT_FLASH_FLAG_CLEAR_ALL));    

    // static uint32_t s_page[64];                  /* 256 字节 = 一个快速页 */

    // for (uint32_t i = 0; i < 64u; i++) { s_page[i] = 0x11111111u; }

    // (void)FLASH_ROM_ERASE(FLASH_BASE_ADDR+0x2000u, 0x1000u);     /* WCH 下载器同款擦除（双解锁） */
    // dbg_eop_erase = FLASH->STATR & (1 << 5);                             /* EOP(0x20) 是否置 1 */

    // (void)FLASH_ROM_WRITE(FLASH_BASE_ADDR+0x2000u, s_page, 0x100u);   /* WCH 下载器同款页编程 */
    // dbg_eop_pgm = FLASH->STATR & ((1 << 5) | (1 << 4));                               /* EOP / PGERR(0x04) / WRPRTERR(0x10) */
    // dbg_pgerr   = FLASH->STATR   & 0x04;
    // dbg_wpr     = FLASH->WPR;
    // dbg_cfgr0   = RCC->CFGR0;

    // dbg_rb = *(volatile uint32_t *)(FLASH_BASE_ADDR+0x2000u);      /* 期望 0x11111111 */

    // plat_ch32v203_flash_ops->pf_lock();
    // dbg_lock_1 = FLASH->CTLR & (1 << 7);
    // dbg_rpr = FLASH->OBR;


    /* 参数表写入测试 */
    /* 【验证"刷写固件不擦第 15 页"的正确姿势】当前这两行是关掉的，所以固件从不写
     * 0x0800F000，该页永远是空白态 0xE339E339 —— 此时再刷一遍固件、读回还是
     * 0xE339E339，"擦"与"没擦"根本无法区分，实验没有意义。
     * 正确步骤：
     *   1) 保留下面两行（放开注释）→ 构建 + 烧录 + 复位，让固件把参数块写进页 15；
     *      回读 0x0800F000 应看到 0x4D435450 ("PTCM")。
     *   2) 再把两行注释掉 → 重新构建 + 烧录（就是"新一轮固件刷写"）。
     *   3) 回读 0x0800F000：
     *        0x4D435450 → 参数页活着，擦除范围确实只到第 14 扇区；
     *        0xE339E339 → 页 15 被擦了，需要查擦除范围/镜像是否越过 0xF000。
     * 也可以不依赖固件，直接用 build/seed-page15.cfg 手工种入测试数据。
     *
     * 【注意 save() 是"先擦后写"（bsp_param_store.c:285/292），所以页里读到
     *  0xE339E339 有歧义】：既可能是"写成功了、之后又被整片擦掉"，也可能是
     *  "快速擦成功了、快速写失败了"。判别办法：在调试器里看
     *    - Write_Flag   ：bsp_param_store_save() 的返回值，ok=0；
     *    - Flash_flag   ：厂商 FLASH_ProgramPage_Fast 内部的 SR_WR_BSY 违规标志，
     *                     若为 1 说明快速写时序不满足（快速编程函数被放在普通
     *                     FLASH 中执行，手册要求"仅可放在零等待区 FLASH"，见
     *                     bsp_param_store.h:27 的警告）；
     *    - FLASH->STATR ：bit2 PGERR / bit4 WRPRTERR 是否置位。 */
    // Init_Flag = bsp_param_store_ops->pf_init(&bsp_param_store_board);
    // Write_Flag = bsp_param_store_ops->pf_save(&bsp_param_store_board);

    /* adc注入组采样与触发测试 */ 
    // bsp_motor_ctrl_ops->pf_init(&bsp_motor_ctrl_three_phase);
    // bsp_motor_ctrl_ops->pf_set_duty(&bsp_motor_ctrl_three_phase,3000,1500,600);
    
    // hal_gpio_ops->pf_build(&hal_gpio_io_test,
    //                         hal_gpio_io_test_pin_table,
    //                         HAL_GPIO_TABLE_COUNT(hal_gpio_io_test_pin_table));
    
    // bsp_current_sense_ops->pf_init(&bsp_current_sense_three_phase);

    // hal_adc_ops->pf_enable_injected_irq(&hal_adc_injected,1);

    /* adc规则组读取测试、母线电压采样与电位器读取验证 */
    // bsp_analog_sense_ops->pf_init(&bsp_analog_sense_board);
    // bsp_analog_sense_ops->pf_update(&bsp_analog_sense_board);

    /* bsp_comm_mgr justfloat发送测试 */
    // bsp_comm_mgr_ops->pf_init(&bsp_comm_mgr_debug);

    /* bsp_comm_mgr rx_idle test */
    // bsp_comm_mgr_ops->pf_init(&bsp_comm_mgr_debug);
    // bsp_comm_mgr_ops->pf_enable_rx_irq(&bsp_comm_mgr_debug,ENABLE);

    while(1)
    {
        /* bsp_encoder test */

        // MT6701_Flag = bsp_encoder_mt6701_ops->pf_get_mechanical_angle(&bsp_encoder_mt6701);


        /* bsp_comm_mgr_justfloat_send test */

        // MT6701_Flag = bsp_encoder_mt6701_ops->pf_get_electrical_angle(&bsp_encoder_mt6701);

        // bsp_encoder_mt6701_ops->pf_get_angle_pu(&bsp_encoder_mt6701,bsp_encoder_angle_raw_mechanical,&mech_angle_pu);

        // bsp_encoder_mt6701_ops->pf_get_angle_pu(&bsp_encoder_mt6701,bsp_encoder_angle_raw_electrical,&ele_angle_pu);

        // Init_Flag++;s
        
        // if(Init_Flag >= 5)
        // {
        //     comm_mgr_angle_pu[0] = mech_angle_pu;
        //     comm_mgr_angle_pu[1] = ele_angle_pu;

        //     bsp_comm_mgr_ops->pf_send_justfloat(&bsp_comm_mgr_debug,comm_mgr_angle_pu,2);

        //     Init_Flag = 0;
        // }

        
        /* bsp_comm_mgr rx_idle test */
        // if (bsp_comm_mgr_ops->pf_is_data_available(&bsp_comm_mgr_debug))
        // {
        //     /* 帧元数据环是先进先出：pf_get_frame 取出的就是最早到达的那一帧，
        //      * 成功后整帧字节已拷入 rx.frame_buffer[0..frame_length)，
        //      * 帧长见 rx.frame_length；下一次取帧会覆盖该缓冲，需保留请自行拷走。 */
        //     comm_mgr_rx_flag = bsp_comm_mgr_ops->pf_get_frame(&bsp_comm_mgr_debug);

        //     if (comm_mgr_rx_flag == bsp_comm_mgr_ok)
        //     {
        //         uint16_t index;

        //         comm_mgr_rx_length = bsp_comm_mgr_debug.rx.frame_length;

        //         for (index = 0U; index < comm_mgr_rx_length; index++)
        //         {
        //             comm_mgr_rx_frame[index] =
        //                 bsp_comm_mgr_debug.rx.frame_buffer[index];
        //         }
        //     }
        // }
    }
}









