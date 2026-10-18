/**
 * @file platform_ch32v203.h
 * @brief CH32V203 平台适配公共接口。
 * @details 提供芯片外设映射、GPIO 和中断配置等平台相关定义。
 */
#ifndef __PLATFORM_CH32V203_H__
#define __PLATFORM_CH32V203_H__

#include "ch32v20x_conf.h"
/* IQmath 定点运算库（third_party/wch/ch32v20x/iqmath，随 libIQmath_RV32.a 链接）：
 * 本头用 PLAT_IQ_* 宏封装其通用 Q 格式计算函数，厂商符号只出现在宏体中；
 * 声明随本头一并对上层可见（库内 GLOBAL_Q 由编译宏给出，本工程取 15）。 */
#include "IQmath_RV32.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PLAT_SYS_CORE_CLOCK                 SystemCoreClock

/** @brief 平台通用使能、状态和时钟常量映射。 */
#define PLAT_ENABLE                         ENABLE
#define PLAT_DISABLE                        DISABLE
#define PLAT_SET                            SET
#define PLAT_RESET                          RESET
#define PLAT_READY                          READY
#define PLAT_NOT_READY                      NoREADY
#define PLAT_BIT_SET                        Bit_SET
#define PLAT_BIT_RESET                      Bit_RESET
#define PLAT_HSE_VALUE                      HSE_VALUE
#define PLAT_HSI_VALUE                      HSI_VALUE
#define PLAT_HSE_STARTUP_TIMEOUT            HSE_STARTUP_TIMEOUT

/** @brief AHB总线外设时钟常量映射。 */
#define PLAT_RCC_AHB_DMA1                   RCC_AHBPeriph_DMA1
#define PLAT_RCC_AHB_DMA2                   RCC_AHBPeriph_DMA2
#define PLAT_RCC_AHB_SRAM                   RCC_AHBPeriph_SRAM
#define PLAT_RCC_AHB_CRC                    RCC_AHBPeriph_CRC
#define PLAT_RCC_AHB_USBFS                  RCC_AHBPeriph_USBFS

/** @brief APB2总线外设时钟常量映射。 */
#define PLAT_RCC_APB2_AFIO                  RCC_APB2Periph_AFIO
#define PLAT_RCC_APB2_GPIOA                 RCC_APB2Periph_GPIOA
#define PLAT_RCC_APB2_GPIOB                 RCC_APB2Periph_GPIOB
#define PLAT_RCC_APB2_GPIOC                 RCC_APB2Periph_GPIOC
#define PLAT_RCC_APB2_GPIOD                 RCC_APB2Periph_GPIOD
#define PLAT_RCC_APB2_GPIOE                 RCC_APB2Periph_GPIOE
#define PLAT_RCC_APB2_ADC1                  RCC_APB2Periph_ADC1
#define PLAT_RCC_APB2_ADC2                  RCC_APB2Periph_ADC2
#define PLAT_RCC_APB2_TIM1                  RCC_APB2Periph_TIM1
#define PLAT_RCC_APB2_SPI1                  RCC_APB2Periph_SPI1
#define PLAT_RCC_APB2_USART1                RCC_APB2Periph_USART1

/** @brief APB1总线外设时钟常量映射。 */
#define PLAT_RCC_APB1_TIM2                  RCC_APB1Periph_TIM2
#define PLAT_RCC_APB1_TIM3                  RCC_APB1Periph_TIM3
#define PLAT_RCC_APB1_TIM4                  RCC_APB1Periph_TIM4
#define PLAT_RCC_APB1_TIM5                  RCC_APB1Periph_TIM5
#define PLAT_RCC_APB1_SPI2                  RCC_APB1Periph_SPI2
#define PLAT_RCC_APB1_USART2                RCC_APB1Periph_USART2
#define PLAT_RCC_APB1_USART3                RCC_APB1Periph_USART3
#define PLAT_RCC_APB1_I2C1                  RCC_APB1Periph_I2C1
#define PLAT_RCC_APB1_I2C2                  RCC_APB1Periph_I2C2
#define PLAT_RCC_APB1_CAN1                  RCC_APB1Periph_CAN1

/** @brief ADC 时钟（ADCCLK = PCLK2 / 分频系数）分频常量映射，供 PLAT_RCC_ADC_CLOCK_CONFIG 使用。 */
#define PLAT_RCC_ADC_CLOCK_DIV2             RCC_PCLK2_Div2
#define PLAT_RCC_ADC_CLOCK_DIV4             RCC_PCLK2_Div4
#define PLAT_RCC_ADC_CLOCK_DIV6             RCC_PCLK2_Div6
#define PLAT_RCC_ADC_CLOCK_DIV8             RCC_PCLK2_Div8

/** @brief CH32V203 GPIO、ADC、SPI、I2C、UART、TIM和CAN外设映射。 */
#define PLAT_PERIPH_GPIOA                   GPIOA
#define PLAT_PERIPH_GPIOB                   GPIOB
#define PLAT_PERIPH_GPIOC                   GPIOC
#define PLAT_PERIPH_GPIOD                   GPIOD
#define PLAT_PERIPH_GPIOE                   GPIOE
#define PLAT_PERIPH_GPIOF                   GPIOF
#define PLAT_PERIPH_GPIOG                   GPIOG
#define PLAT_PERIPH_ADC1                    ADC1
#define PLAT_PERIPH_ADC2                    ADC2
#define PLAT_PERIPH_SPI1                    SPI1
#define PLAT_PERIPH_SPI2                    SPI2
#define PLAT_PERIPH_I2C1                    I2C1
#define PLAT_PERIPH_I2C2                    I2C2
#define PLAT_PERIPH_USART1                  USART1
#define PLAT_PERIPH_USART2                  USART2
#define PLAT_PERIPH_USART3                  USART3
#define PLAT_PERIPH_TIM1                   TIM1
#define PLAT_PERIPH_TIM2                   TIM2
#define PLAT_PERIPH_TIM3                   TIM3
#define PLAT_PERIPH_TIM4                   TIM4
#define PLAT_PERIPH_TIM5                   TIM5
#define PLAT_PERIPH_CAN1                   CAN1
#define PLAT_PERIPH_EXTI                   EXTI

/** @brief CH32V203外设中断号映射。 */
#define PLAT_IRQ_ADC                        ADC_IRQn
#define PLAT_IRQ_CAN1_RX0                   USB_LP_CAN1_RX0_IRQn
#define PLAT_IRQ_CAN1_TX                    USB_HP_CAN1_TX_IRQn
#define PLAT_IRQ_I2C1_EVENT                 I2C1_EV_IRQn
#define PLAT_IRQ_I2C1_ERROR                 I2C1_ER_IRQn
#define PLAT_IRQ_SPI1                       SPI1_IRQn
#define PLAT_IRQ_SPI2                       SPI2_IRQn
#define PLAT_IRQ_USART1                     USART1_IRQn
#define PLAT_IRQ_USART2                     USART2_IRQn
#define PLAT_IRQ_USART3                     USART3_IRQn
#define PLAT_IRQ_TIM1_UPDATE                TIM1_UP_IRQn
#define PLAT_IRQ_TIM1_CAPTURE_COMPARE       TIM1_CC_IRQn
#define PLAT_IRQ_TIM2                       TIM2_IRQn
#define PLAT_IRQ_TIM3                       TIM3_IRQn
#define PLAT_IRQ_TIM4                       TIM4_IRQn

/* --------------------------------------------------------------------------
 * 中断服务函数入口（厂商向量名 + 中断属性二次封装）
 * 说明：启动文件对每个中断向量只提供 `.weak` 死循环缺省实现，业务层必须自行实现
 *       对应向量；为避免上层直接书写厂商符号与中断属性，此处把板级用到的三个向量
 *       收进宏，实现处写成：PLAT_ISR_DEFINE(PLAT_ISR_ADC1_2) { … }
 * 注意：本组宏只负责"函数名 + 中断属性"，中断线的优先级配置仍由 PLAT_PFIC_INIT
 *       完成，中断源的使能仍由各外设自己的 pf_it_config 完成。
 * -------------------------------------------------------------------------- */

/**
 * @brief 定义厂商中断服务函数（含 WCH 硬件压栈中断属性）。
 * @note 属性必须写在声明符之前，函数定义处才合法（GCC 要求：
 *       attributes should be specified before the declarator）。
 */
#define PLAT_ISR_DEFINE(isr_vector) \
    __attribute__((interrupt("WCH-Interrupt-fast"))) void isr_vector(void)

/** @brief ADC1/ADC2 共用中断向量（规则组 EOC / 注入组 JEOC）。 */
#define PLAT_ISR_ADC1_2                     ADC1_2_IRQHandler
/** @brief USB 低优先级 + CAN1 RX0 中断向量（CAN 接收 FIFO0）。 */
#define PLAT_ISR_CAN1_RX0                   USB_LP_CAN1_RX0_IRQHandler
/** @brief USART3 中断向量（命令通信：空闲帧 / 收发中断）。 */
#define PLAT_ISR_USART3                     USART3_IRQHandler

/** @brief 使能AHB、APB1或APB2外设时钟。 */
#define PLAT_RCC_AHB_CLOCK_ENABLE(peripheral) \
    RCC_AHBPeriphClockCmd((peripheral), PLAT_ENABLE)
#define PLAT_RCC_APB1_CLOCK_ENABLE(peripheral) \
    RCC_APB1PeriphClockCmd((peripheral), PLAT_ENABLE)
#define PLAT_RCC_APB2_CLOCK_ENABLE(peripheral) \
    RCC_APB2PeriphClockCmd((peripheral), PLAT_ENABLE)

/** @brief 禁用AHB、APB1或APB2外设时钟。 */
#define PLAT_RCC_AHB_CLOCK_DISABLE(peripheral) \
    RCC_AHBPeriphClockCmd((peripheral), PLAT_DISABLE)
#define PLAT_RCC_APB1_CLOCK_DISABLE(peripheral) \
    RCC_APB1PeriphClockCmd((peripheral), PLAT_DISABLE)
#define PLAT_RCC_APB2_CLOCK_DISABLE(peripheral) \
    RCC_APB2PeriphClockCmd((peripheral), PLAT_DISABLE)

/** @brief 复位APB1或APB2外设。 */
#define PLAT_RCC_APB1_RESET(peripheral, state) \
    RCC_APB1PeriphResetCmd((peripheral), (state))
#define PLAT_RCC_APB2_RESET(peripheral, state) \
    RCC_APB2PeriphResetCmd((peripheral), (state))

/**
 * @brief 配置 ADC 时钟分频（ADCCLK = PCLK2 / 分频系数）。
 * @param adc_clock_div 分频系数，取 PLAT_RCC_ADC_CLOCK_DIVx。
 * @note ADCCLK 由 ADC1/ADC2 共用，须在 ADC 使能/校准之前配置；
 *       数据手册要求 fADC ≤ 14MHz（例如 PCLK2 = 72MHz 时取 8 分频 → 9MHz）。
 */
#define PLAT_RCC_ADC_CLOCK_CONFIG(adc_clock_div) \
    RCC_ADCCLKConfig((adc_clock_div))

/** @brief PFIC中断优先级分组常量。 */
#define PLAT_PFIC_PRIORITY_GROUP_0          NVIC_PriorityGroup_0
#define PLAT_PFIC_PRIORITY_GROUP_1          NVIC_PriorityGroup_1

/**
 * @brief PFIC（NVIC）中断初始化结构体类型接入宏。
 * @note 类型也一律经宏接入：本头只允许厂商名字出现在宏体中（延迟展开），
 *       platform_ch32v203.c / 各外设平台 .c 内只出现 PLAT_* 名字。
 *       GPIO / SPI 的初始化结构体类型接入宏分别定义在 platform_ch32v203_gpio.h /
 *       platform_ch32v203_spi.h（宏体无需在解析期可见，故不再需要在此处提前声明）。
 */
#define PLAT_PFIC_INIT_TYPE                 NVIC_InitTypeDef

/**
 * @brief 配置 CH32V203 的中断优先级和定时器通道控制参数。
 * @param priority_group 中断优先级分组配置。
 */
#define PLAT_PFIC_PRIORITY_GROUP_CONFIG(priority_group) \
    do { \
        NVIC_PriorityGroupConfig((priority_group)); \
    } while (0)

/**
 * @brief 当前模块操作表。
 * @param irq_channel 外设通道号。
 * @param preemption_priority 抢占优先级。
 * @param sub_priority 子优先级。
 * @param channel_cmd 定时器通道控制命令。
 */
#define PLAT_PFIC_INIT(irq_channel, preemption_priority, sub_priority, channel_cmd) \
    do { \
        PLAT_PFIC_INIT_TYPE pfic_init_config = { \
            (uint8_t)(irq_channel), \
            (uint8_t)(preemption_priority), \
            (uint8_t)(sub_priority), \
            (channel_cmd) \
        }; \
        NVIC_Init(&pfic_init_config); \
    } while (0)

/* --------------------------------------------------------------------------
 * IQmath 通用 Q 格式定点运算（third_party/wch/ch32v20x/iqmath）
 * 说明：本层统一封装**通用 Q 格式**函数（_IQxxx 一族，具体格式由库内 GLOBAL_Q
 *       决定：本工程以 -DGLOBAL_Q=15 编译，故实际为 Q15，1.0 = 32768）；
 *       厂商符号只出现在宏体中，上层只书写 PLAT_IQ_* 名字。
 * 注意：通用宏的入参/返回格式随 GLOBAL_Q 变化，改该编译宏即整体切换 Q 格式；
 *       使用前需保证 IQmath_RV32.h 对本编译单元可见（本头已包含）。
 * -------------------------------------------------------------------------- */

/** @brief Q15 定点类型（等价于厂商 _iq15，32 位有符号；1.0 = 32768）。 */
#define PLAT_IQ_TYPE                        _iq

/** @brief 浮点 → Q15，以及 Q15 → 浮点 / Q15 → 全局 Q 格式转换。 */
#define PLAT_IQ(value)                      _IQ(value)
#define PLAT_IQ_TO_FLOAT(a)                 _IQtoF(a)
#define PLAT_IQ_TO_DOUBLE(a)                _IQtoD(a)
#define PLAT_IQ_TO_IQ(a)                    ((PLAT_IQ_TYPE)(a))   /* GLOBAL_Q = 15：Q15 即全局 Q 格式 */

/** @brief 乘/带舍入乘/除/快速除/取整/取小数/绝对值/与整数混合乘。 */
#define PLAT_IQ_MPY(a, b)                   _IQmpy((a), (b))
#define PLAT_IQ_RMPY(a, b)                  _IQrmpy((a), (b))
#define PLAT_IQ_DIV(a, b)                   _IQdiv((a), (b))
#define PLAT_IQ_DIV_FAST(a, b)              _IQdiv((a), (b))      /* 库内 DIV_FAST = 1：通用除法即快速除法 */
#define PLAT_IQ_INT(a)                      _IQint(a)
#define PLAT_IQ_FRAC(a)                     _IQfrac(a)
#define PLAT_IQ_ABS(a)                      _IQabs(a)
#define PLAT_IQ_MPY_I32(a, b)               _IQmpyI32((a), (b))
#define PLAT_IQ_MPY_I32_INT(a, b)           _IQmpyI32int((a), (b))
#define PLAT_IQ_MPY_I32_FRAC(a, b)          _IQmpyI32frac((a), (b))
#define PLAT_IQ_MPY_IQX(a, iqa, b, iqb)     _IQmpyIQX((a), (iqa), (b), (iqb))

/** @brief 三角函数：弧度制（SIN/COS/ASIN/ACOS/ATAN/ATAN2）与标幺制（…_PU）。 */
#define PLAT_IQ_SIN(a)                      _IQsin(a)
#define PLAT_IQ_COS(a)                      _IQcos(a)
#define PLAT_IQ_SIN_PU(a)                   _IQsinPU(a)
#define PLAT_IQ_COS_PU(a)                   _IQcosPU(a)
#define PLAT_IQ_ASIN(a)                     _IQasin(a)
#define PLAT_IQ_ACOS(a)                     _IQacos(a)
#define PLAT_IQ_ATAN(a)                     _IQatan(a)
#define PLAT_IQ_ATAN2(a, b)                 _IQatan2((a), (b))
#define PLAT_IQ_ATAN2_PU(a, b)              _IQatan2PU((a), (b))

/** @brief 开方/倒数开方/指数/2 的幂/常用对数/矢量幅值。 */
#define PLAT_IQ_SQRT(a)                     _IQsqrt(a)
#define PLAT_IQ_ISQRT(a)                    _IQisqrt(a)
#define PLAT_IQ_EXP(a)                      _IQexp(a)
#define PLAT_IQ_EXP2(a)                     _IQexp2(a)
#define PLAT_IQ_LOG10(a)                    _IQlog10(a)
#define PLAT_IQ_MAG(a, b)                   _IQmag((a), (b))

/** @brief Q15 转字符串（调试用，等价于库内 __IQNtoa(..., 15)）。 */
#define PLAT_IQ_TOA(buffer, format, a)      _IQtoa((buffer), (format), (a))

#ifdef __cplusplus
}
#endif

#endif /* __PLATFORM_CH32V203_H__ */










