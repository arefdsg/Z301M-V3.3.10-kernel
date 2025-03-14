#ifndef _MTK_ADC_H
#define _MTK_ADC_H

/* ----------------------------------------------------------------------------- */

extern int IMM_IsAdcInitReady(void);
extern int IMM_get_adc_channel_num(char *channel_name, int len);
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int *rawdata);
extern int IMM_GetOneChannelValue_Cali(int Channel, int *voltage);
extern u32 get_devinfo_with_index(u32 index);

#endif	 /*_MTK_ADC_H*/
