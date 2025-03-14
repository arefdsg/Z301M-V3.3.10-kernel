#ifndef ICM20608_H
#define ICM20608_H

#include <linux/ioctl.h>

#define ICM20608_ACCESS_BY_GSE_I2C

#ifdef ICM20608_ACCESS_BY_GSE_I2C
    #define ICM20608_I2C_SLAVE_ADDR		(0xD2)   /* mtk i2c not allow to probe two same address */
#else
    #define ICM20608_I2C_SLAVE_ADDR		0xD0
#endif


/* ICM20608 Register Map  (Please refer to ICM20608 Specifications) */
/**************************************************************************/
#define ICM20608_REG_XG_OFFS_USR_H      0x13

#define ICM20608_REG_YG_OFFS_USR_H      0x15

#define ICM20608_REG_ZG_OFFS_USR_H      0x17

#define ICM20608_REG_SMPLRT_DIV         0x19

#define ICM20608_REG_CONFIG             0x1A
#define BIT_FIFO_MODE                           0x40
#define BIT_DLPF_CFG                            0x07
#define SHIFT_FIFO_MODE                         6
#define SHIFT_DLPF_CFG                          0

#define ICM20608_REG_GYRO_CONFIG        0x1B
#define BIT_GYRO_ST                             0xE0
#define BIT_FS_SEL                              0x18
#define BIT_FCHOICE_B                           0x03
#define SHIFT_FS_SEL                            3
#define SHIFT_FCHOICE_B                         0
#define GYRO_FS_250DPS                          (0x00 << SHIFT_FS_SEL)
#define GYRO_FS_500DPS                          (0x01 << SHIFT_FS_SEL)
#define GYRO_FS_1000DPS                         (0x02 << SHIFT_FS_SEL)
#define GYRO_FS_2000DPS                         (0x03 << SHIFT_FS_SEL)

#define ICM20608_REG_ACCEL_CONFIG       0x1C
#define BIT_ACCEL_ST                            0xE0
#define BIT_ACCEL_FS_SEL                        0x18
#define SHIFT_ACCEL_FS_SEL                      3
#define ACCEL_FS_2G                             (0x00 << SHIFT_ACCEL_FS_SEL)
#define ACCEL_FS_4G                             (0x01 << SHIFT_ACCEL_FS_SEL)
#define ACCEL_FS_8G                             (0x02 << SHIFT_ACCEL_FS_SEL)
#define ACCEL_FS_16G                            (0x03 << SHIFT_ACCEL_FS_SEL)

#define ICM20608_REG_ACCEL_CONFIG2      0x1D
#define BIT_DEC2_CFG                            0x30
#define BIT_ACCEL_FCHOICE_B                     0x08
#define BIT_A_DLPF_CFG                          0x07
#define SHIFT_DEC2_CFG                          4
#define SHIFT_ACCEL_FCHOICE_B                   3
#define SHIFT_A_DLPF_CFG                        0

#define ICM20608_REG_LP_MODE_CFG        0x1E
#define BIT_GYRO_CYCLE                          0x80
#define BIT_G_AVGCFG                            0x70
#define BIT_LPOSC_CLKSEL                        0x07
#define SHIFT_GYRO_CYCLE                        7
#define SHIFT_G_AVGCFG                          4
#define SHIFT_LPOSC_CLKSEL                      0

#define ICM20608_REG_ACCEL_WOM_THR      0x1F

#define ICM20608_REG_FIFO_EN            0x23
#define BIT_TEMP_FIFO_EN                        0x80
#define BIT_GYRO_FIFO_EN                        0x70
#define BIT_ACCEL_FIFO_EN                       0x08

#define ICM20608_REG_INT_PIN_CFG        0x37
#define BIT_INT_LEVEL                           0x80
#define BIT_INT_OPEN                            0x40
#define BIT_LATCH_INT_EN                        0x20
#define BIT_INT_RD_CLEAR                        0x10
#define BIT_ACTL                                0x80
#define BIT_INT_ANYRD_2CLEAR                    0x10

#define ICM20608_REG_INT_ENABLE         0x38
#define BIT_WOM_INT_EN                          0xE0
#define BIT_FIFO_OFLOW_EN                       0x10
#define BIT_GDRIVE_INT_EN                       0x04
#define BIT_DATA_RDY_INT_EN                     0x01

#define ICM20608_REG_INT_STATUS         0x3A
#define BIT_WOM_INT                             0xE0
#define BIT_FIFO_OFLOW_INT                      0x10
#define BIT_GDRIVE_INT                          0x04
#define BIT_DATA_RDY_INT                        0x01

#define ICM20608_REG_ACCEL_XOUT_H       0x3B

#define ICM20608_REG_TEMP_OUT_H         0x41

#define ICM20608_REG_GYRO_XOUT_H        0x43

#define ICM20608_REG_ACCEL_INTEL_CTRL   0x69
#define BIT_ACCEL_INTEL_EN                      0x80
#define BIT_ACCEL_INTEL_MODE                    0x40

#define ICM20608_REG_USER_CTRL          0x6A
#define BIT_FIFO_EN                             0x40
#define BIT_I2C_IF_DIS                          0x10
#define BIT_FIFO_RST                            0x04

#define ICM20608_REG_PWR_MGMT_1         0x6B
#define BIT_DEVICE_RESET                        0x80
#define BIT_SLEEP                               0x40
#define BIT_ACCEL_CYCLE                         0x20
#define BIT_CLKSEL                              0x07
#define BIT_CLK_PLL                             0x01

#define ICM20608_REG_PWR_MGMT_2         0x6C
#define BIT_FIFO_LP_EN                          0x80
#define BIT_STBY_A                              0x38
#define BIT_STBY_G                              0x07

#define ICM20608_REG_MEM_BANK_SEL       0x6D

#define ICM20608_REG_MEM_START_ADDR     0x6E

#define ICM20608_REG_MEM_R_W            0x6F

#define ICM20608_REG_PRGM_START_ADDR_H  0x70

#define ICM20608_REG_FIFO_COUNT_H       0x72

#define ICM20608_REG_FIFO_R_W           0x74

#define ICM20608_REG_WHO_AM_I           0x75

#define ICM20608_REG_XA_OFFS_H          0x77

#define ICM20608_REG_YA_OFFS_H          0x7A

#define ICM20608_REG_ZA_OFFS_H          0x7D
/**************************************************************************/

#define ICM20608_FS_1000_LSB            33
#define ICM20608_FS_MAX_LSB             131

#define ICM20608_G_BW_177HZ            0x01
#define ICM20608_G_BW_108HZ            0x02
#define ICM20608_G_BW_59HZ             0x03
#define ICM20608_G_BW_30HZ             0x04
#define ICM20608_G_BW_15HZ             0x05
#define ICM20608_G_BW_8HZ              0x06

#define ICM20608_SUCCESS             0
#define ICM20608_ERR_I2C             -1
#define ICM20608_ERR_STATUS          -3
#define ICM20608_ERR_SETUP_FAILURE   -4
#define ICM20608_ERR_GETGSENSORDATA  -5
#define ICM20608_ERR_IDENTIFICATION  -6


#define ICM20608_BUFSIZE 60

/* 1 rad = 180/PI degree, MAX_LSB = 131, */
/* 180*131/PI = 7506 */
#define DEGREE_TO_RAD	7506

extern int ICM20608_gse_power(void);
extern int ICM20608_gse_mode(void);

#ifdef ICM20608_ACCESS_BY_GSE_I2C
extern int ICM20608_hwmsen_read_block(u8 addr, u8 *buf, u8 len);
extern int ICM20608_hwmsen_write_block(u8 addr, u8 *buf, u8 len);
#endif

#ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB
#define ICM20608GY_AXES_NUM        3

/*----------------------------------------------------------------------------*/
enum CUST_ACTION {
    ICM20608GY_CUST_ACTION_SET_CUST = 1,
    ICM20608GY_CUST_ACTION_SET_CALI,
    ICM20608GY_CUST_ACTION_RESET_CALI
};
/*----------------------------------------------------------------------------*/
struct ICM20608GY_CUST {
    uint16_t    action;
};
/*----------------------------------------------------------------------------*/
struct ICM20608GY_SET_CUST {
    uint16_t    action;
    uint16_t    part;
    int32_t     data[0];
};
/*----------------------------------------------------------------------------*/
struct ICM20608GY_SET_CALI {
    uint16_t    action;
    int32_t     data[ICM20608GY_AXES_NUM];
};
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
union ICM20608GY_CUST_DATA {
    uint32_t                data[10];
    struct ICM20608GY_CUST         cust;
    struct ICM20608GY_SET_CUST     setCust;
    struct ICM20608GY_SET_CALI     setCali;
    struct ICM20608GY_CUST   resetCali;
};
/*----------------------------------------------------------------------------*/
#endif				/* #ifdef CONFIG_CUSTOM_KERNEL_SENSORHUB */
#endif

