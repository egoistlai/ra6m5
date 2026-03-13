#ifndef _DRV_ADC_H_
#define _DRV_ADC_H_

#include "hal_data.h"

/* * 电压计算参数 (完全参考原 STM32 代码)
 * 3.3V 参考电压, 12位精度 (4096)
 * 4.97f 是分压电阻系数 (例如 15V -> 3V)
 */
#define ADC_REF_VOLTAGE 3.3f
#define ADC_PRECISION 4096.0f
#define ADC_DIVIDER_RATIO 4.97f // 硬件分压系数，请根据实际电路调整

/* 导出函数 */
fsp_err_t DrvAdcInit(void);
float Drv_AdcGetBatVot(void);

#endif /* _DRV_ADC_H_ */