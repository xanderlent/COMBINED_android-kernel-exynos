#ifndef _HL6111_CHARGER_H_
#define _HL6111_CHARGER_H_

#define BITS(_end, _start) ((BIT(_end) - BIT(_start)) + BIT(_end))

#define REG_LATCHED_STATUS                  0x00
#define BIT_I_LDO5_L                        BIT(7)
#define BIT_OCA_L                           BIT(5)
#define BIT_OTA_L                           BIT(4)
#define BIT_OVA_L                           BIT(1)
#define BIT_OUT_EN_L                        BIT(0)

#define REG_VRECT                           0x01
#define BITS_VRECT                          BITS(7,0)

#define REG_IRECT                           0x02
#define BITS_IRECT                          BITS(7,0)

#define REG_TEMP                            0x03
#define BITS_TEMP                           BITS(7,0)

#define REG_CLAMP_TH                        0x04
#define BITS_CLM_VTH                        BITS(6,4)
#define BITS_CLM_DSM_EN                     BITS(2,0)

#define REG_A4WP_CONFIG1                    0x05
#define BIT_PWR_OK                          BIT(1)

#define REG_STATUS                          0x07
#define BIT_I_LDO5_I                        BIT(7)
#define BIT_MODE                            BIT(6)
#define BIT_OCA_I                           BIT(5)
#define BIT_OTA_I                           BIT(4)
#define BIT_OVA_S                           BIT(1)
#define BIT_OUT_EN_S                        BIT(0)

#define REG_ID                              0x0A
#define BITS_ID                             BIT(5,0)

#define REG_TEMP_LIMIT                      0x0B
#define BITS_MPP_TLIM                       BITS(7,0)

#define REG_POWER_LIMIT                     0x0C
#define BITS_POUT_MAX                       BITS(7,0)

#define REG_Z_LIMIT                         0x0D

#define REG_VOUT_TARGET                     0x0E
#define BITS_VOUT_TRGT                      BITS(7,0)

#define REG_PINT_LIMIT                      0x0F

#define REG_VPSNS_REF                       0x10
#define BIT_VPSNS_EN                        BIT(6)
#define BITS_VPSNS_VREF                     BITS(4,0)

#define REG_VPSNS_MAX                       0x11
#define BITS_VPSNS_VMAX                     BITS(4,0)

#define REG_INTERRUPT_ENABLE                0x12
#define BIT_I_LDO_L_EN                      BIT(7)
#define BIT_OCA_L_EN                        BIT(5)
#define BIT_OTA_L_EN                        BIT(4)
#define BIT_OVA_L_EN                        BIT(1)
#define BIT_OUT_EN                          BIT(0)

#define REG_VPSNS_MODE                      0x13
#define BIT_VPSNS_MODE                      BITS(4,0)

#define REG_DSW1_TH_CONFIG                  0x14
#define BITS_VDSW1_FAIL_TH                  BITS(6,4)
#define BITS_VDSW1_RISE_TH                  BITS(3,1)

#define REG_DSW2_TH_CONFIG                  0x15
#define BITS_VDSW2_FAIL_TH                  BITS(6,4)
#define BITS_VDSW2_RISE_TH                  BITS(3,1)

#define REG_GPIO_CONFIG                     0x16
#define BITS_LBEEN_GP1_GPIO                 BITS(5,4)
#define BITS_DSW2_GP4_GPIO                  BITS(3,2)
#define BITS_DSW1_GP3_GPIO                  BITS(1,0)

#define REG_NTC_MEASURED_H                  0x17
#define BITS_NTC_VALUE_H                    BITS(7,0)

#define REG_NTC_MEASURED_L                  0x18
#define BITS_NTC_VALUE_L                    BITS(7,6)

#define REG_NTC_OTA_CONFIG                  0x19
#define BIT_NTC_OTA_TC                      BIT(1)
#define BIT_NTC_OTA_SEL                     BIT(0)

#define REG_I2C_Addr                        0x1A

#define REG_VOUT_BYPASS                     0x20
#define BITS_VOUT_BP                        BITS(7,4)
#define BIT_A4WP_DCDC_MODE                  BIT(2)
#define BITS_PR_AD                          BITS(1,0)

#define REG_SR_MODE_CONFIG                  0x21
#define BIT_DMR_POLARITY                    BIT(3)
#define BIT_SR_MODE_CFG_2                   BIT(2)
#define BITS_SR_MODE_CONFIG                 BITS(1,0)

#define REG_DMR_CONFIG                      0x22
#define BITS_VTH_DMR                        BITS(6,3)

#define REG_VRECT_HEADROOM                  0x27
#define BITS_VRECT_HDRM                     BITS(7,2)

#define REG_IOUT_LIM_SEL                    0x28
#define BITS_IOUTLIM                        BITS(7,3)

#define REG_VRECT_TARGET                    0x2A
#define BITS_VRECT_TRGT                     BITS(4,0)

#define REG_VOUT_RANGE_SEL                  0x30
#define BITS_VOUT_RNG_SEL                   BITS(7,6)

#define REG_LDOP_REF                        0x31
#define BITS_LDOP_REF                       BITS(1,0)

#define REG_VOUT_AVG                        0x83
#define BITS_VOUT                           BITS(7,0)

#define REG_IOUT_AVG                        0x8F
#define BITS_IOUT                           BITS(7,0)

#define REG_MAX     0xFF

#ifdef HALO_DBG
#define LOG_DBG(fmt, args...)   printk(KERN_ERR "[%s]:: " fmt, __func__, ## args);
#else
#define LOG_DBG(fmt, args...)
#endif

#define HL6111_I2C_NAME "hl6111"

enum hl6111_ept_reason {
    internal,
    sys_fault,
    fully_charged,
    over_temp,
};


enum {
    HL6111_WPC_DISCONNECTED,
    HL6111_WPC_CONNECTED,
};


enum {
    OVER_V_TH_19_4V,
    OVER_V_TH_19_9V,
    OVER_V_TH_20_4V,
    OVER_V_TH_20_9V,
    OVER_V_TH_21_4V,
    OVER_V_TH_21_9V,
    OVER_V_TH_22_4V,
    OVER_V_TH_22_9V,
};

enum {
    LDOP_1_8V,
    LDOP_2_5V,
    LDOP_3_3V,
};

enum VOUT_RANGE {
    VOUT_RANGE_20MV = 0,
    VOUT_RANGE_30MV,
    VOUT_RANGE_40MV,
    VOUT_RANGE_16MV,
};

struct hl6111_platform_data{
    /* IRQ NUM */
    unsigned int irq;
    unsigned int irq_det;
    /* GPIO CTRL */
    int det_gpio;
    int int_gpio;

    //dtsi
    unsigned int clm_vth;
    unsigned int bypass;
    unsigned int ldop;
    unsigned int vout_range;
    unsigned int trgt_vout;
    unsigned int trgt_vrect;
    unsigned int temp_limit;
};

struct hl6111_charger{
    struct regmap                   *regmap;
    struct device                   *dev;
    struct power_supply             *psy_chg;
    struct mutex                    i2c_lock;
    struct hl6111_platform_data     *pdata;
    struct i2c_client               *client;
    struct delayed_work             rx_work;
#ifdef HL6111_ENABLE_CHOK
    struct delayed_work             chok_work;
    unsigned int retry_cnt;
#endif

    struct dentry                   *debug_root;

    u32  debug_address;
    bool online;

    unsigned int vrect;
    unsigned int irect;
    unsigned int t_die;//temperature;
    unsigned int trgt_vout;
    unsigned int trgt_vrect;
    unsigned int iout_lim;
    unsigned int temp_lim;
    unsigned int bypass;
    unsigned int vhead;
    unsigned int iout;
    unsigned int vout;
    unsigned int ntc_temp;
    u8           chip_id;
};

#endif
