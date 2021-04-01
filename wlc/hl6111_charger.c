#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/power_supply.h>
#include <linux/pm_runtime.h>

#include "hl6111_charger.h"

#if defined (CONFIG_OF)
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#endif

static const struct regmap_config hl6111_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = REG_MAX,
};

static char *hl6111_supplied_to[] = {
    "battery",
};

static int hl6111_read_reg(struct hl6111_charger *halo, int reg, int *value)
{
    int ret = 0;
    mutex_lock(&halo->i2c_lock);
    ret = regmap_read(halo->regmap, reg, value);
    mutex_unlock(&halo->i2c_lock);

    if (ret < 0)
        pr_info("%s: reg(0x%x), ret(%d)\n", __func__, reg, ret);
    return ret;
}

static int hl6111_write_reg(struct hl6111_charger *halo, int reg, u8 value)
{
    int ret = 0;
    mutex_lock(&halo->i2c_lock);
    ret = regmap_write(halo->regmap, reg, value);
    mutex_unlock(&halo->i2c_lock);

    if (ret < 0)
        pr_info("%s: reg(0x%x), ret(%d)\n", __func__, reg, ret);
    return ret;
}

static int hl6111_update_reg(struct hl6111_charger *halo, int reg, u8 mask, u8 value)
{
    int ret = 0;
    mutex_lock(&halo->i2c_lock);
    ret = regmap_update_bits(halo->regmap, reg, mask, value);
    mutex_unlock(&halo->i2c_lock);
    if (ret < 0)
        pr_info("%s: reg(0x%x), ret(%d)\n", __func__, reg, ret);
    return ret;
}

#if defined (CONFIG_OF)
static int hl6111_parse_dt(struct device *dev, struct hl6111_platform_data *pdata)
{
    struct device_node *np = dev->of_node;
    int rc = 0;
    u32 temp;

    if (!np)
        return -EINVAL;

    pdata->int_gpio = of_get_named_gpio(np, "halo,int-gpio", 0);
    LOG_DBG("irq-gpio:: %d \n", pdata->int_gpio);

    pdata->det_gpio = of_get_named_gpio(np, "halo,det-gpio", 0);
    LOG_DBG("irq-det:: %d \r\n", pdata->det_gpio);

    //*Vrect Clamp threshold*//
    rc = of_property_read_u32(np, "halo,clm-vth", &temp);
    if (rc) {
        pr_err("Invalid clm-vth\n");
    }else{
        pdata->clm_vth = temp;
        LOG_DBG("clm-vth[%d] \n", pdata->clm_vth);
    }

    //*vout bypass*//
    rc = of_property_read_u32(np, "halo,bypass", &temp);
    if (rc) {
        pr_err("Invalid bypass\n");
    }else{
        pdata->bypass = temp;
        LOG_DBG("bypass[%d] \n", pdata->bypass);
    }

    //*ldop ref*//
    rc = of_property_read_u32(np, "halo,ldop", &temp);
    if (rc) {
        pr_err("Invalid ldop\n");
        pdata->ldop = LDOP_1_8V;
    }else{
        pdata->ldop = temp;
        LOG_DBG("ldop[%d] \n", pdata->ldop);
    }

    /*VOUT Range */
    rc = of_property_read_u32(np, "halo,vout-range", &temp);
    if (rc) {
        pr_err("Invalid Vrange\n");
        pdata->vout_range = VOUT_RANGE_16MV;
    }else{
        pdata->vout_range = temp;
        LOG_DBG("VOUT_RANGE[%d] \n", pdata->vout_range);
    }

    /* VOUT_TARGET */
    rc = of_property_read_u32(np, "halo,trgt-vout", &temp);
    if (rc) {
        pr_err("Invalid TargetVout\n");
        pdata->trgt_vout = 0x15;
    }else{
        pdata->trgt_vout_default = temp;
        pdata->trgt_vout = temp;
        LOG_DBG("trgt_vout[%d]\n", pdata->trgt_vout);
    }
    pdata->trgt_vout_override = 0;
    /* VRECT TARGET */
    rc = of_property_read_u32(np, "halo,trgt-vrect", &temp);
    if (rc) {
        pr_err("Invalid TargetVrect\n");
        pdata->trgt_vrect = 0x02;
    }else{
        pdata->trgt_vrect = temp;
        LOG_DBG("trgt_vrect[%d]\n", pdata->trgt_vrect);
    }
    rc = of_property_read_u32(np, "halo,temp-limit", &temp);
    if (rc) {
        pr_err("Invalid TempLimit\n");
        pdata->temp_limit = 0x1a; // 46C
    }else{
        pdata->temp_limit = temp;
        LOG_DBG("temp_limit[%d]\n", pdata->temp_limit);
    }

    return 0;
}

#else
static int hl6111_parse_dt(struct device *dev, struct hl6111_platform_data *pdata)
{
    return 0;
}
#endif

static void hl6111_get_chip_info(struct hl6111_charger *chg)
{
    int r_val;
    int ret;

    ret = hl6111_read_reg(chg, REG_ID, &r_val);
    if(ret < 0){
        LOG_DBG("failed to read REG_ID\r\n");
        chg->chip_id = 0;
        return;
    }

    chg->chip_id = r_val;
}

/*
static void hl6111_dcdc_en_ctrl(struct hl6111_charger *chg, bool onoff)
{
    //gpio ctrl
    if (onoff)
        gpio_set_value(chg->pdata->dcdcen_gpio, 1);
    else
        gpio_set_value(chg->pdata->dcdcen_gpio, 0);

}
*/

static void hl6111_send_ept(struct hl6111_charger *chg, enum hl6111_ept_reason ept)
{
    LOG_DBG("Start!!, ept reason = [%d]\r\n", ept);

    if(ept == internal){
        hl6111_update_reg(chg, 0xED, 0x80, 0x80);
        LOG_DBG("internal!!\r\n");
    } else if (ept == sys_fault){
        dev_info(chg->dev, "Hot: Sending EPT to TX\n");
        hl6111_update_reg(chg, 0xED, 0x40, 0x40);
    }else if (ept == fully_charged){
        dev_info(chg->dev, "Full Charged: Sending EPT to TX\n");
        hl6111_update_reg(chg, 0xED, 0x20, 0x20);
    }else if (ept == over_temp){
        LOG_DBG("Over Temp\n");
        hl6111_update_reg(chg, REG_NTC_OTA_CONFIG, BIT_NTC_OTA_SEL, BIT_NTC_OTA_SEL);
    }
}

static int hl6111_get_health(struct hl6111_charger *chg, union power_supply_propval *val)
{
    int ret, health;
    int sts;

    LOG_DBG("Start!!\r\n");
    ret = hl6111_read_reg(chg, REG_STATUS, &sts);
    if (ret < 0){
        pr_err("Can't read latched_status, reg0x%x rc =%d\r\n", REG_LATCHED_STATUS, ret);
        val->intval = POWER_SUPPLY_HEALTH_DEAD;
        return ret;
    }

    if (sts & BIT_I_LDO5_L){
        LOG_DBG("Over Current Occured!!\n");
        health = POWER_SUPPLY_HEALTH_UNKNOWN;//POWER_SUPPLY_HEALTH_OVERCURRENT;
    }else if (sts & BIT_OCA_L){
        LOG_DBG("Vrect node Over Current\r\n");
        health = POWER_SUPPLY_HEALTH_UNKNOWN;//POWER_SUPPLY_HEALTH_OVERCURRENT;
    }else if (sts & BIT_OTA_L){
        health = POWER_SUPPLY_HEALTH_OVERHEAT;
        LOG_DBG("Over Temperature!!\r\n");
    }else if (sts & BIT_OVA_L){
        health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
        LOG_DBG("Vrect Over Voltage!!\r\n");
    }else if (sts & BIT_OUT_EN_L){
        health = POWER_SUPPLY_HEALTH_DEAD;
        LOG_DBG("BIT_OUT_EN_L is Enabled, %d\r\n", (unsigned int)(sts & BIT_OUT_EN_L));
    }else if (!(sts & BIT_OUT_EN_L)){
        health = POWER_SUPPLY_HEALTH_GOOD;
        LOG_DBG("BIT_OUT_EN_L is Disabled, %d\r\n", (unsigned int)(sts & BIT_OUT_EN_L));
    }

        val->intval = health;
    return 0;
}

static int hl6111_isr_check(struct hl6111_charger *chg)
{
    int ret = 0;
    int sts;

    ret =  hl6111_read_reg(chg, REG_LATCHED_STATUS, &sts);
    if (ret < 0){
        pr_err("Can't read latched_status, reg0x%x rc =%d\r\n", REG_LATCHED_STATUS, ret);
        return -EINVAL;
    }

    LOG_DBG("Start!!, intsts == [0x%x]\r\n", sts);
    if (sts & BIT_I_LDO5_L){
        LOG_DBG("Over Current Occured!!\n");
    }else if (sts & BIT_OCA_L){
        LOG_DBG("Vrect node Over Current\r\n");
    }else if (sts & BIT_OTA_L){
        LOG_DBG("Over Temperature!!\r\n");
    }else if (sts & BIT_OVA_L){
        LOG_DBG("Vrect Over Voltage!!\r\n");
    }else if (sts & BIT_OUT_EN_L){
        LOG_DBG("BIT_OUT_EN_L is Enabled, %d\r\n", (unsigned int)(sts & BIT_OUT_EN_L));
    }else if (!(sts & BIT_OUT_EN_L)){
        LOG_DBG("BIT_OUT_EN_L is Disabled, %d\r\n", (unsigned int)(sts & BIT_OUT_EN_L));
    }

    return ret;
}

static void hl6111_measure_vout(struct hl6111_charger *chg)
{
    int ret;
    int value;

    LOG_DBG("Start!!\r\n");
    ret =  hl6111_read_reg(chg, REG_VOUT_AVG, &value);
    if (ret < 0){
        LOG_DBG("Failed to read VOUT!!");
        return;
    }

    chg->vout = (value * 93750);
    LOG_DBG("address = [0x%x], value = [0x%x], [%dmA]\n", (u8)REG_VOUT_AVG, value, chg->vout);
}

static void hl6111_measure_vheadroom(struct hl6111_charger *chg)
{
    int ret;
    int value;

    LOG_DBG("Start!!\r\n");
    ret =  hl6111_read_reg(chg, REG_VRECT_HEADROOM, &value);
    if (ret < 0){
        LOG_DBG("Failed to read Vrect!!");
        return;
    }

    chg->vhead = (value >> 2) * 250; //250mV * X == Vout Target
    LOG_DBG("address = [0x%x], value = [0x%x], [%dmA]\n", (u8)REG_VRECT_HEADROOM, value, chg->vhead);
}

static void  hl6111_vheadroom_ctrl(struct hl6111_charger *chg, u8 update)
{
    LOG_DBG("Start!!, update ==[0x%x]", update);

    if (update < 0) update = 0;
    else if (update > 0x3F) update = 0x3F;

    hl6111_write_reg(chg, REG_VRECT_HEADROOM, (update << 2));
    hl6111_measure_vheadroom(chg);
}


static void hl6111_measure_vrect(struct hl6111_charger *chg)
{
    int ret;
    int value;

    LOG_DBG("Start!!\r\n");
    ret =  hl6111_read_reg(chg, REG_VRECT, &value);
    if (ret < 0){
        LOG_DBG("Failed to read Vrect!!\r\n");
        return;
    }

    chg->vrect = value * 93750;
    LOG_DBG("addr[0x%x], read[0x%x], vrect is [%d mA]!!\r\n", REG_VRECT, value, chg->vrect);
}

static void hl6111_measure_irect(struct hl6111_charger *chg)
{
    int ret;
    int value;

    LOG_DBG("Start!!");
    ret =  hl6111_read_reg(chg, REG_IRECT, &value);
    if (ret < 0){
        LOG_DBG("Failed to read Irect!!\r\n");
        return;
    }

    chg->irect = (value * 13) * 1000;
    LOG_DBG("addr[0x%x], read[0x%x], Irect[%d uA]\r\n", REG_IRECT, value, chg->irect);
}

static void hl6111_measure_tdie(struct hl6111_charger *chg)
{
    int ret;
    int value;
    //float temp;

    LOG_DBG("Start!!");
    ret =  hl6111_read_reg(chg, REG_TEMP, &value);
    if (ret < 0){
        LOG_DBG("Failed to read TEMP!!\r\n");
        return;
    }

//    temp = (220.09f - value)/0.6316f;
    chg->t_die = value;

    LOG_DBG("addr[0x%x], read[0x%x], tdie[0x%x] \r\n", REG_TEMP, value,  chg->t_die);
}

static void hl6111_measure_iout(struct hl6111_charger *chg)
{
    int ret;
    int value;

    LOG_DBG("Start!!");
    ret =  hl6111_read_reg(chg, REG_IOUT_AVG, &value);
    if (ret < 0){
        LOG_DBG("Failed to read IOUT!!\r\n");
        return;
    }

    chg->iout = ((value * 9171) + 9171);

    LOG_DBG("addr[0x%x], read[0x%x], iout[%d uA]\r\n", REG_IOUT_AVG, value, chg->iout);
}

static void hl6111_measure_ntc_temp(struct hl6111_charger *chg)
{
    int ret, ntc;
    int h_val, l_val;

    LOG_DBG("Start!!");
    ret = hl6111_read_reg(chg, REG_NTC_MEASURED_H, &h_val);
    ret = hl6111_read_reg(chg, REG_NTC_MEASURED_L, &l_val);
    if (ret < 0){
        LOG_DBG("Failed to read NTC TMEP!!\r\n");
        return;
    }

    ntc =  ((int)h_val << 2) | (int)(l_val >> 6);
    chg->ntc_temp = ntc;

    LOG_DBG("high[0x%x], low[0x%x], ntc[0x%x], ntc_temp[%d]\r\n", h_val, l_val, ntc, chg->ntc_temp);
}

static void hl6111_get_target_vout(struct hl6111_charger *chg)
{
    int ret;
    int vout, range;

    ret =  hl6111_read_reg(chg, REG_VOUT_TARGET, &vout);
    if (ret < 0){
        LOG_DBG("Failed to read addr=%X\r\n ", REG_VOUT_TARGET);
        return;
    }
    ret =  hl6111_read_reg(chg, REG_VOUT_RANGE_SEL, &range);
    if (ret < 0){
        LOG_DBG("Failed to read addr=%X\r\n ", REG_VOUT_RANGE_SEL);
        return;
    }

    LOG_DBG("vout=[0x%x], vout_range_sel=[0x%x] \r\n", vout, range);

    if ((range & 0xC0) == 0x40)
        chg->trgt_vout = 7410 + (vout * 30);
    else if ((range & 0xC0) == 0x00)
        chg->trgt_vout = 4940 + (vout * 20);
    else if ((range & 0xC0) == 0x80)
        chg->trgt_vout = 9880 + (vout * 40);
    else if ((range & 0xC0) == 0xC0)
        chg->trgt_vout = 3952 + (vout * 16);

    chg->trgt_vout = chg->trgt_vout * 1000;

    LOG_DBG("Target Vout is [%d uV]\r\n", chg->trgt_vout);
}

static void hl6111_target_vout_ctrl(struct hl6111_charger *chg, u8 update)
{
    //int ret;
    //u8 value;

    LOG_DBG("Start!!, update ==[0x%x]", update);

    if (update < 0) update = 0;
    else if (update > 0xFF) update = 0xFF;

    hl6111_write_reg(chg, REG_VOUT_TARGET, update);
    hl6111_get_target_vout(chg); //to update chg->trgt_vout;
}

static void target_vout_set_voltage(struct hl6111_charger *chg, int voltage) {
    int reg_value, range, ret;
    int base, step, new_voltage;
    if (chg->online) {
        ret =  hl6111_read_reg(chg, REG_VOUT_RANGE_SEL, &range);
        if (ret < 0){
            LOG_DBG("Failed to read addr=%X\r\n ", REG_VOUT_RANGE_SEL);
            return;
        }
        range = (range & VOUT_RANGE_SEL_MASK) >> VOUT_RANGE_SEL_SHIFT;
    } else {
        range = chg->pdata->vout_range;
    }
    switch(range) {
        case VOUT_RANGE_20MV:
            base = 4940;
            step = 20;
            break;
        case VOUT_RANGE_30MV:
            base = 7410;
            step = 30;
            break;
        case VOUT_RANGE_40MV:
            base = 9880;
            step = 40;
            break;
        case VOUT_RANGE_16MV:
            base = 3952;
            step = 16;
            break;
    }
    // Add step - 1 so that we round up instead of down
    reg_value = (voltage + (step - 1) - base) / step;

    if (reg_value < 0 || reg_value > 0xff) {
        new_voltage = base + reg_value * step;
        dev_err(chg->dev, "Invalid Target Vout Value:%d, reg=[%x]\n", new_voltage, reg_value);
        return;
    }
    if (chg->pdata->trgt_vout != reg_value) {
        dev_info(chg->dev, "Setting Target Vout to %d, reg=[%x]\n", voltage, reg_value);
        chg->pdata->trgt_vout = reg_value;
        if (chg->online) {
            hl6111_target_vout_ctrl(chg, (u8) reg_value);
        }
    }
}

static void hl6111_get_target_vrect(struct hl6111_charger *chg)
{
    int ret;
    int vrect;

    ret = hl6111_read_reg(chg, REG_VRECT_TARGET, &vrect);
    if (ret < 0){
        LOG_DBG("Failed to read addr=%X\r\n ", REG_VRECT_TARGET);
        return;
    }
    LOG_DBG("vrect=[0x%x]\r\n", vrect);

    chg->trgt_vrect = 5000000 + 500000 * vrect;

    LOG_DBG("Target Vrect is [%d uV]\r\n", chg->trgt_vrect);
}

static void hl6111_target_vrect_ctrl(struct hl6111_charger *chg, u8 update)
{
    LOG_DBG("Start!!, update ==[0x%x]", update);

    if (update < 0)
        update = 0;
    else if (update > 0xFF)
        update = 0xFF;

    hl6111_write_reg(chg, REG_VRECT_TARGET, update);
    hl6111_get_target_vrect(chg); //to update chg->trgt_vrect;
}

static void hl6111_get_ioutlim(struct hl6111_charger *chg)
{
    int ret;
    int ilim;

    ret =  hl6111_read_reg(chg, REG_IOUT_LIM_SEL, &ilim);
    if (ret < 0){
        LOG_DBG("Failed to read addr=%X\r\n ", REG_IOUT_LIM_SEL);
        return;
    }

    chg->iout_lim = (((ilim >> 3) * 50) + 100) * 1000;

    LOG_DBG("ioutlim:: [%d uA], ilim:[0x%x]\r\n", chg->iout_lim, ilim);
}

static void hl6111_set_ioutlim(struct hl6111_charger *chg, u8 update)
{
    LOG_DBG("START !! update ==[0x%x]", update);
    if (update < 0) update = 0;
    else if (update > 0x1F) update = 0x1F;

    if (update == 0x1F){
        LOG_DBG("No Limit!!\n");
    }

    hl6111_write_reg(chg, REG_IOUT_LIM_SEL, (update << 3));

    hl6111_get_ioutlim(chg);
}

static void hl6111_get_templim(struct hl6111_charger *chg)
{
    int ret;
    int templim;

    ret = hl6111_read_reg(chg, REG_TEMP_LIMIT, &templim);
    if (ret < 0){
        LOG_DBG("Failed to read addr=%X\r\n ", REG_TEMP_LIMIT);
        return;
    }

    chg->temp_lim = templim * 19500;
    LOG_DBG("templim:[%d uV], register:[0x%x]\r\n", chg->temp_lim, templim);
}

static void hl6111_set_templim(struct hl6111_charger *chg, u8 update)
{
    LOG_DBG("START !! update ==[0x%x]", update);
    if (update < 0)
        update = 0;
    else if (update > 0xFF)
        update = 0xFF;

    hl6111_write_reg(chg, REG_TEMP_LIMIT, update);

    hl6111_get_templim(chg);
}

static void hl6111_get_vout_bypass(struct hl6111_charger *chg)
{
    int ret;
    int r_val;

    ret =  hl6111_read_reg(chg, REG_VOUT_BYPASS, &r_val);
    if (ret < 0){
        LOG_DBG("Failed to read addr=%X\r\n ", REG_IOUT_LIM_SEL);
        return;
    }

    chg->pdata->bypass = (r_val >> 4);

    if((r_val >> 4) == 0x0F)
        chg->bypass = 12000;
    else
        chg->bypass = (5000 + ((r_val >> 4) * 500)) * 1000;

    LOG_DBG("vout bypass:: [%duV], pdata->bypass::[0x%x]\r\n", chg->bypass, chg->pdata->bypass);
}

static void hl6111_set_vout_bypass(struct hl6111_charger *chg, u8 update)
{
#if defined (HALO_DBG)
    int r_val;
#endif
    LOG_DBG("Start!! update::[0x%x]\r\n", update);

    if (update < 0 ) update = 0;
    else if (update > 0xF) update = 0xF; //only 4bits being use

    hl6111_update_reg(chg, REG_VOUT_BYPASS, BITS_VOUT_BP, (update << 4));

#if defined (HALO_DBG)
    hl6111_read_reg(chg, REG_VOUT_BYPASS, &r_val);
    LOG_DBG("BITS_VOUT_BP:[0x%x], update:[0x%x], r_val:[0x%x]", (unsigned int)BITS_VOUT_BP, (update << 4), r_val);
#endif
}

static void hl6111_device_init(struct hl6111_charger *chg)
{
    //int voutsel_mode;
#if 1
    int rVal;
#endif

    int bypass = chg->pdata->bypass;
    int clm_vth = chg->pdata->clm_vth;

    dev_info(chg->dev, "Sending initial WLC settings\n");

    hl6111_write_reg(chg, REG_INTERRUPT_ENABLE, 0xFF);   //enable interrupt

    //bypass setting
    hl6111_update_reg(chg, REG_VOUT_BYPASS, BITS_VOUT_BP, (bypass << 4));
    /*clamp setting*/
    hl6111_update_reg(chg, REG_CLAMP_TH,    BITS_CLM_VTH, (clm_vth << 4));


    /* VOUT Range setting to select lowest voltage range which from 3.952 to 8.040V*/
    hl6111_update_reg(chg, REG_VOUT_RANGE_SEL, BITS_VOUT_RNG_SEL, (chg->pdata->vout_range << 6));
    /* Vout == 4.304V 16mV/step*/
    if (chg->pdata->trgt_vout_override) {
       hl6111_target_vout_ctrl(chg, chg->pdata->trgt_vout_override);
    } else {
       hl6111_target_vout_ctrl(chg, chg->pdata->trgt_vout);
    }
    hl6111_target_vrect_ctrl(chg, chg->pdata->trgt_vrect);
    hl6111_set_templim(chg, chg->pdata->temp_limit);

#if 1
    hl6111_read_reg(chg, REG_VOUT_BYPASS, &rVal);
    LOG_DBG("reg[0x%x]: value:[0x%x]\n", REG_VOUT_BYPASS, rVal);
    hl6111_read_reg(chg, REG_INTERRUPT_ENABLE, &rVal);
    LOG_DBG("reg[0x%x]: value:[0x%x]\n", REG_INTERRUPT_ENABLE, rVal);
    hl6111_read_reg(chg, REG_CLAMP_TH, &rVal);
    LOG_DBG("reg[0x%x]: value:[0x%x]\n", REG_CLAMP_TH, rVal);

    hl6111_read_reg(chg, REG_VOUT_RANGE_SEL, &rVal);
    LOG_DBG("reg[0x%x]: value:[0x%x]\n", REG_VOUT_RANGE_SEL, rVal);
    hl6111_read_reg(chg, REG_VOUT_TARGET, &rVal);
    LOG_DBG("reg[0x%x]: value:[0x%x]\n", REG_VOUT_TARGET, rVal);

    hl6111_read_reg(chg, REG_NTC_OTA_CONFIG, &rVal);
    LOG_DBG("reg[0x%x]: value:[0x%x]\n", REG_NTC_OTA_CONFIG, rVal);
    hl6111_read_reg(chg, REG_TEMP_LIMIT, &rVal);
    LOG_DBG("reg[0x%x]: value:[0x%x]\n", REG_TEMP_LIMIT, rVal);
#endif
}

static irqreturn_t hl6111_interrupt_handler(int irg, void *data)
{
    struct hl6111_charger *chg = data;

    int ret;
    //bool handled = false;

    LOG_DBG("START!, irq_handled=%d, irq_none=%d\n", IRQ_HANDLED, IRQ_NONE);

    disable_irq_nosync(chg->pdata->irq);

    //hl6111_write_reg(chg, REG_INTERRUPT_ENABLE, 0x00);

    ret = hl6111_isr_check(chg);
    if (ret < 0)
        goto I2C_FAIL;

    enable_irq(chg->pdata->irq);
    return IRQ_HANDLED;

I2C_FAIL:
    LOG_DBG("Failed to read I2C\r\n");
    return IRQ_NONE;
}

static int hl6111_psy_set_property(struct power_supply *psy, enum power_supply_property psp,
                                        const union power_supply_propval *val)
{
    struct hl6111_charger *chg = power_supply_get_drvdata(psy);
    u8 update;

    LOG_DBG("Start!, prop==[%d], val==[%d]\n", psp, val->intval);
    switch(psp) {
        case POWER_SUPPLY_PROP_ONLINE:
            dev_info(chg->dev, "Setting WLC online: %d\n", val->intval);
            if (val->intval == 0 ) {      //charging off
                chg->online = false;
                disable_irq(chg->pdata->irq);
            } else {
                chg->online = true;
                // Reset vout setting to default
                chg->pdata->trgt_vout = chg->pdata->trgt_vout_default;
                hl6111_write_reg(chg, REG_INTERRUPT_ENABLE, 0xFF);
                enable_irq(chg->pdata->irq);
                hl6111_get_chip_info(chg);
                hl6111_device_init(chg);
            }

            break;
        case POWER_SUPPLY_PROP_STATUS:
            LOG_DBG("STATUS:\r\n");
            if (val->intval == POWER_SUPPLY_STATUS_FULL){
                LOG_DBG("Fully charged!!\r\n");
                hl6111_send_ept(chg, fully_charged);
            } else if (val->intval == POWER_SUPPLY_STATUS_NOT_CHARGING){
                LOG_DBG("Stop charging!!\r\n");
                hl6111_send_ept(chg, sys_fault);
            }
            break;
        case POWER_SUPPLY_PROP_PRESENT:
            LOG_DBG("PRESENT:\r\n");
            break;
        case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:     //OUTPUT CURRENT LIMIT
            if (val->intval == 0)
                hl6111_set_ioutlim(chg, 0x1F);//no limit
            else if (val->intval >= 2200)
                update = 0x1E;//iin = 2200;
            else if ((val->intval <= 1100) && (val->intval > 0))
                update = (val->intval - 100)/50;
            else if ((val->intval > 1100) && (val->intval < 2200))
                update = (val->intval - 1100)/100;

            hl6111_set_ioutlim(chg, update);
            LOG_DBG("output current limit:  value[%d], update[0x%x] \r\n", val->intval, update);
            break;
        case POWER_SUPPLY_PROP_ENERGY_AVG: //target vout
            target_vout_set_voltage(chg, val->intval);
            break;

        default:
            return -EINVAL;
    }
    LOG_DBG("END!");
    return 0;
}

static irqreturn_t hl6111_pad_detect_handler(int irq, void *data)
{
    struct hl6111_charger *chg = data;
    int sts;

    sts = gpio_get_value(chg->pdata->det_gpio);
    if (sts == 0){
        dev_info(chg->dev, "TX connection detected\n");
        chg->tx_det = true;
    } else if (sts == 1) {
        dev_info(chg->dev, "TX disconnection detected\n");
        chg->tx_det = false;
    }

    power_supply_changed(chg->psy_chg);
    return IRQ_HANDLED;
}

static int hl6111_irq_init(struct hl6111_charger *chg, struct i2c_client *client)
{
    struct hl6111_platform_data *pdata = chg->pdata;
    int ret, irq;

    LOG_DBG("start!\n");

    irq = gpio_to_irq(pdata->int_gpio);
    ret = gpio_request_one(pdata->int_gpio, GPIOF_IN, client->name);
    if (ret < 0)
        goto fail;
    LOG_DBG("irq[%d], int_gpio[%d]\n", irq, pdata->int_gpio);
    ret = request_threaded_irq(irq, NULL, hl6111_interrupt_handler,
                                IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                client->name, chg);
    if (ret < 0)
        goto fail_irq;

    pdata->irq = irq;
    client->irq = irq;
    disable_irq(chg->pdata->irq);

    return 0;

fail_irq:
    LOG_DBG("request_threaded_irq failed :: %d\n", ret);
    gpio_free(pdata->int_gpio);
    if (pdata->det_gpio > 0)
       gpio_free(pdata->det_gpio);
fail:
    LOG_DBG("gpio_request failed :: ret->%d\n", ret);
    client->irq = 0;
    return ret;
}

static int hl6111_irq_det_init(struct hl6111_charger *chg)
{
    struct hl6111_platform_data *pdata = chg->pdata;
    int ret, irq_det;

    if (pdata->det_gpio > 0) {
        irq_det = gpio_to_irq(pdata->det_gpio);

        ret = request_threaded_irq(irq_det, NULL, hl6111_pad_detect_handler,
                              IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                              "wireless-det-irq", chg);
        if(ret < 0)
            goto fail_irq_det;
        pdata->irq_det = irq_det;

        LOG_DBG("irq_det::%d, det_gpio::%d\r\n",pdata->irq_det,  pdata->det_gpio);
    }
    return 0;

fail_irq_det:
    LOG_DBG("gpio_request failed :: ret->%d\n", ret);
    gpio_free(pdata->det_gpio);
    return ret;
}

static void hl6111_regmap_init(struct hl6111_charger *chg)
{
    int ret;
    LOG_DBG("Start!!\n");
   if (!i2c_check_functionality(chg->client->adapter, I2C_FUNC_I2C)) {
        dev_err(chg->dev, "%s: check_functionality failed.", __func__);
        return;
    }

    chg->regmap = devm_regmap_init_i2c(chg->client, &hl6111_regmap_config);
    if(IS_ERR(chg->regmap)) {
        ret = PTR_ERR(chg->regmap);
        dev_err(chg->dev, "regmap init failed, err == %d\n", ret);
        return;
    }
    i2c_set_clientdata(chg->client, chg);

    return;
}

static int hl6111_psy_get_property(struct power_supply *psy, enum power_supply_property psp,
                                            union power_supply_propval *val)
{
    struct hl6111_charger *chg = power_supply_get_drvdata(psy);
    //int ret = 0;

    LOG_DBG("Start!");
    switch(psp) {
        case POWER_SUPPLY_PROP_ONLINE:
            LOG_DBG("ONLINE:\r\n");
            val->intval = chg->online;
            break;

        case POWER_SUPPLY_PROP_STATUS:
            LOG_DBG("STATUS:\r\n");
            val->intval = chg->online ? POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_DISCHARGING;
            break;

        case POWER_SUPPLY_PROP_PRESENT:
            LOG_DBG("PRESENT!!\r\n");
            val->intval = chg->tx_det;
            break;

        case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT://output current limit
            LOG_DBG("INPUT_CURRENT_LIMIT:\r\n");
            hl6111_get_ioutlim(chg);
            val->intval = chg->iout_lim;
            break;

        case POWER_SUPPLY_PROP_HEALTH:
            LOG_DBG("HEALTH:\r\n");
            return hl6111_get_health(chg, val);
            break;

        case POWER_SUPPLY_PROP_MANUFACTURER:
            LOG_DBG("MANUFACTURER\r\n");
            val->intval = chg->chip_id;
            break;

        case POWER_SUPPLY_PROP_ENERGY_NOW: //vrect
            LOG_DBG("VRECT\r\n");
             hl6111_measure_vrect(chg);
            val->intval = chg->vrect;
            break;

        case POWER_SUPPLY_PROP_ENERGY_AVG: //vout
            LOG_DBG("VOUT\r\n");
            hl6111_measure_vout(chg);
            val->intval = chg->vout;
            break;

        case POWER_SUPPLY_PROP_CURRENT_AVG:  //iout
            LOG_DBG("IOUT\r\n");
            hl6111_measure_iout(chg);
            val->intval = chg->iout;
            break;

        case POWER_SUPPLY_PROP_CURRENT_NOW: //irect
            LOG_DBG("IREC\r\n");
            hl6111_measure_irect(chg);
            val->intval = chg->irect;
            break;

        case POWER_SUPPLY_PROP_TEMP:        //die temperature
            LOG_DBG("T_DIE\r\n");
            hl6111_measure_tdie(chg);
            val->intval = chg->t_die;
            break;

        case POWER_SUPPLY_PROP_TEMP_AMBIENT: //NTC TEMP
            LOG_DBG("NTC TEMP\r\n");
            hl6111_measure_ntc_temp(chg);
            val->intval = chg->ntc_temp;
            break;

       default:
            return -EINVAL;
    }

    LOG_DBG("END!");
    return 0;
}

static int read_reg(void *data, u64 *val)
{
	struct hl6111_charger *chg = data;
	int rc;
	unsigned int temp;
    LOG_DBG("Start!\n");
	rc = regmap_read(chg->regmap, (u8)chg->debug_address, &temp);
	if (rc) {
		pr_err("Couldn't read reg 0x%x rc = %d\n",
			chg->debug_address, rc);
		return -EAGAIN;
	}

    LOG_DBG("address = [0x%x], value = [0x%x]\n", chg->debug_address, temp);
	*val = temp;
	return 0;
}

static int write_reg(void *data, u64 val)
{
	struct hl6111_charger *chg = data;
	int rc;
	u8 temp;

    LOG_DBG("Start! val == [%x]\n", (u8)val);
	temp = (u8) val;
	rc = regmap_write(chg->regmap, (u8)chg->debug_address, temp);
	if (rc) {
		pr_err("Couldn't write 0x%02x to 0x%02x rc= %d\n",
			temp, chg->debug_address, rc);
		return -EAGAIN;
	}
	return 0;
}

static int vrect_head_get(void *data, u64 *val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start!\n");

    hl6111_measure_vheadroom(chg);
	*val = chg->vhead;
    return 0;
}

static int vrect_head_set(void *data, u64 val)
{
    struct hl6111_charger *chg = data;

    hl6111_vheadroom_ctrl(chg, (u8)val);
	return 0;
}

static int vrect_show(void *data, u64 *val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start!\n");

    hl6111_measure_vrect(chg);
    *val = chg->vrect;

    return 0;
}

static int irect_show(void *data, u64 *val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start!");
    hl6111_measure_irect(chg);
    *val = chg->irect;

    return 0;
}

static int tdie_show(void *data, u64 *val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start!!");

    hl6111_measure_tdie(chg);
    *val = chg->t_die;

    return 0;
}

static int iout_get(void *data, u64 *val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start!!\r\n");

    hl6111_measure_iout(chg);
    *val = chg->iout;

    return 0;
}

static int vout_get(void *data, u64 *val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start!!");
    hl6111_measure_vout(chg);
    *val = chg->vout;

    return 0;
}


static int target_vout_set(void *data, u64 val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start! val == [%x]\n", (u8)val);
    if ((val > 0xFF) || (val < 0))
    {
        return -EAGAIN;
    }

    chg->pdata->trgt_vout_override = val;
    hl6111_target_vout_ctrl(chg, val);

    return 0;
}

static int target_vout_get(void *data, u64 *val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start!!");
    hl6111_get_target_vout(chg);
    *val = chg->trgt_vout;
    return 0;

}

static int target_vrect_get(void *data, u64 *val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start!!");
    hl6111_get_target_vrect(chg);
    *val = chg->trgt_vrect;
    return 0;
}

static int target_vrect_set(void *data, u64 val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start! val == [%x]\n", (u8)val);
    if ((val > 0xFF) || (val < 0))
    {
        return -EAGAIN;
    }

    chg->pdata->trgt_vrect = val;
    hl6111_target_vrect_ctrl(chg, val);

    return 0;
}

static int templim_get(void *data, u64 *val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start!!");
    hl6111_get_templim(chg);
    *val = chg->temp_lim;
    return 0;
}

static int templim_set(void* data, u64 val)
{
   struct hl6111_charger *chg = data;

    LOG_DBG("Start! val == [0x%x]\n", (u8)val);
    if ((val > 0xFF) || (val < 0))
    {
        return -EAGAIN;
    }

    chg->pdata->temp_limit = val;
    hl6111_set_templim(chg, (u8)val);

    return 0;
}

static int ioutlim_get(void *data, u64 *val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start!!");
    hl6111_get_ioutlim(chg);
    *val = chg->iout_lim;
    return 0;
}

static int ioutlim_set(void *data, u64 val)
{
   struct hl6111_charger *chg = data;

    LOG_DBG("Start! val == [0x%x]\n", (u8)val);
    if ((val > 0xFF) || (val < 0))
    {
        return -EAGAIN;
    }

    hl6111_set_ioutlim(chg, (u8)val);

    return 0;
}

static int bypass_get(void *data, u64 *val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start!!");
    hl6111_get_vout_bypass(chg);
    *val = chg->bypass;
    return 0;
}

static int bypass_set(void *data, u64 val)
{
   struct hl6111_charger *chg = data;

    LOG_DBG("Start! val == [%x]\n", (u8)val);
    if ((val > 0x0F) || (val < 0))
    {
        return -EAGAIN;
    }

    hl6111_set_vout_bypass(chg, (u8)(val));
    return 0;
}

static int ept_set(void *data, u64 val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start! val == [%x]\n", (u8)val);
    if ((val > 0x2) || (val < 0))
    {
        return -EAGAIN;
    }

    hl6111_send_ept(chg, (u8)val);
    return 0;
}

static int ntc_temp_get(void *data, u64 *val)
{
    struct hl6111_charger *chg = data;

    LOG_DBG("Start!!");
    hl6111_measure_ntc_temp(chg);
    *val = chg->ntc_temp;

    return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(register_debug_ops, read_reg, write_reg, "0x%02llx\n");
DEFINE_SIMPLE_ATTRIBUTE(headroom_debug_ops, vrect_head_get, vrect_head_set, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(tvout_debug_ops, target_vout_get, target_vout_set, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(tvrect_debug_ops, target_vrect_get, target_vrect_set, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(templim_debug_ops, templim_get, templim_set, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(vrect_debug_ops, vrect_show, NULL, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(irect_debug_ops, irect_show, NULL, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(tdie_debug_ops, tdie_show, NULL, "0x%02llx\n");
DEFINE_SIMPLE_ATTRIBUTE(vout_debug_ops, vout_get, NULL, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(iout_debug_ops, iout_get, NULL, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(ioutlim_debug_ops, ioutlim_get, ioutlim_set, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(bypass_debug_ops, bypass_get, bypass_set, "%lld\n");
DEFINE_SIMPLE_ATTRIBUTE(ept_debug_ops, NULL, ept_set, "0x%02llx\n");
DEFINE_SIMPLE_ATTRIBUTE(ntctemp_debug_ops, ntc_temp_get, NULL, "%lld\n");

static int hl6111_create_debugfs_entries(struct hl6111_charger *chg)
{
    struct dentry *ent;
    int rc = 0;

    chg->debug_root = debugfs_create_dir("hl6111-debugfs", NULL);
    if (!chg->debug_root) {
        dev_err(chg->dev, "Couldn't create debug dir\n");
        rc = -ENOENT;
    } else {
        ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
        chg->debug_root,
        &(chg->debug_address));
        if (!ent) {
            dev_err(chg->dev, "Couldn't create address debug file\n");
            rc = -ENOENT;
         }

         ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &register_debug_ops);
         if (!ent) {
             dev_err(chg->dev, "Couldn't create data debug file\n");
             rc = -ENOENT;
         }

         ent = debugfs_create_file("VrectHeadroom", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &headroom_debug_ops);

        if (!ent) {
             dev_err(chg->dev, "Couldn't create vrectheadroom debug file\n");
             rc = -ENOENT;
         }

        ent = debugfs_create_file("t_vout", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &tvout_debug_ops);

        if (!ent) {
             dev_err(chg->dev, "Couldn't create target vout debug file\n");
             rc = -ENOENT;
         }

        ent = debugfs_create_file("t_vrect", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &tvrect_debug_ops);

        if (!ent) {
             dev_err(chg->dev, "Couldn't create target vrect debug file\n");
             rc = -ENOENT;
         }

        ent = debugfs_create_file("temp_lim", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &templim_debug_ops);

        if (!ent) {
             dev_err(chg->dev, "Couldn't create temp limit debug file\n");
             rc = -ENOENT;
         }

        ent = debugfs_create_file("vrect", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &vrect_debug_ops);

        if (!ent) {
             dev_err(chg->dev, "Couldn't create vrect debug file\n");
             rc = -ENOENT;
         }

        ent = debugfs_create_file("irect", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &irect_debug_ops);

        if (!ent) {
             dev_err(chg->dev, "Couldn't create Irect debug file\n");
             rc = -ENOENT;
         }

        ent = debugfs_create_file("tdie", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &tdie_debug_ops);

        if (!ent) {
             dev_err(chg->dev, "Couldn't create Irect debug file\n");
             rc = -ENOENT;
         }

        ent = debugfs_create_file("vout", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &vout_debug_ops);

        if (!ent) {
             dev_err(chg->dev, "Couldn't create vout debug file\n");
             rc = -ENOENT;
         }

        ent = debugfs_create_file("iout", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &iout_debug_ops);

        if (!ent) {
             dev_err(chg->dev, "Couldn't create iout debug file\n");
             rc = -ENOENT;
         }

        ent = debugfs_create_file("ioutlim", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &ioutlim_debug_ops);

        if (!ent) {
             dev_err(chg->dev, "Couldn't create ioutlim debug file\n");
             rc = -ENOENT;
         }

        ent = debugfs_create_file("bypass", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &bypass_debug_ops);

        if (!ent) {
             dev_err(chg->dev, "Couldn't create bypass debug file\n");
             rc = -ENOENT;
         }
        ent = debugfs_create_file("ept", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &ept_debug_ops);
        if (!ent) {
             dev_err(chg->dev, "Couldn't create ept debug file\n");
             rc = -ENOENT;
         }

         ent = debugfs_create_file("ntc_temp", S_IFREG | S_IWUSR | S_IRUGO,
                       chg->debug_root, chg,
                       &ntctemp_debug_ops);
        if (!ent) {
             dev_err(chg->dev, "Couldn't create ntc_temp debug file\n");
             rc = -ENOENT;
         }
    }

    return rc;
}

static enum power_supply_property hl6111_psy_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_ONLINE,
    /*
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_MANUFACTURER,
    POWER_SUPPLY_PROP_ENERGY_NOW, //vrect
    POWER_SUPPLY_PROP_ENERGY_AVG, //vout
    POWER_SUPPLY_PROP_CURRENT_AVG,  //iout
    POWER_SUPPLY_PROP_CURRENT_NOW, //irect
    POWER_SUPPLY_PROP_TEMP,        //die temperature
    POWER_SUPPLY_PROP_TEMP_AMBIENT, //NTC TEMP
    */
};

static const struct power_supply_desc hl6111_psy_desc = {
    .name = "hl6111-charger",
    .type = POWER_SUPPLY_TYPE_UNKNOWN,
    .get_property = hl6111_psy_get_property,
    .set_property = hl6111_psy_set_property,
    .properties = hl6111_psy_props,
    .num_properties = ARRAY_SIZE(hl6111_psy_props),
};

static int hl6111_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct hl6111_platform_data *pdata;
    struct hl6111_charger *charger;
    struct power_supply_config psy_cfg = {};
    int ret;

    LOG_DBG("Start!\n");

    charger = devm_kzalloc(&client->dev, sizeof(*charger), GFP_KERNEL);
    if (!charger){
        dev_err(&client->dev, "failed to allocate chip memory\n");
        return -ENOMEM;
    }
#if defined(CONFIG_OF)
    if (client->dev.of_node){
        pdata = devm_kzalloc(&client->dev, sizeof(struct hl6111_platform_data), GFP_KERNEL);
        if(!pdata){
            dev_err(&client->dev, "Failed to allocate pdata memory\n");
            return -ENOMEM;
        }

        ret = hl6111_parse_dt(&client->dev, pdata);
        if(ret < 0){
            dev_err(&client->dev, "Failed to get device of node\n");
            return -ENOMEM;
        }

        client->dev.platform_data = pdata;
    } else {
        pdata = client->dev.platform_data;
    }
#else
    pdata = dev->platform_data;
#endif

    if (ret < 0)
        return -EINVAL;

    charger->dev = &client->dev;
    charger->pdata = pdata;
    charger->client = client;
    charger->online = false;

    hl6111_regmap_init(charger);

    psy_cfg.supplied_to     = hl6111_supplied_to;
    psy_cfg.num_supplicants = ARRAY_SIZE(hl6111_supplied_to);
    psy_cfg.drv_data        = charger;

    charger->psy_chg = power_supply_register(&client->dev, &hl6111_psy_desc, &psy_cfg);
    if (IS_ERR(charger->psy_chg)) {
        dev_err(&client->dev, "failed to register power supply\n");
        return PTR_ERR(charger->psy_chg);
    }

    mutex_init(&charger->i2c_lock);

   if (pdata->int_gpio >= 0){
        ret = hl6111_irq_init(charger, client);
        if (ret < 0){
            dev_warn(&client->dev, "failed to initialize IRQ :: %d\n", ret);
            goto FAIL_IRQ;
        }
    }

    if (pdata->det_gpio < 0){
        dev_info(&client->dev, "No detection GPIO, driver won't do anything\n");
    }else {
        if (gpio_get_value(charger->pdata->det_gpio) == 0){

            dev_info(charger->dev, "TX connection initially detected\n");
            charger->tx_det = true;

        } else {
            dev_info(charger->dev, "TX connection was not detected\n");
            charger->tx_det = false;
        }
        ret = hl6111_irq_det_init(charger);
        if (ret < 0) {
            dev_warn(&client->dev, "failed to initialize IRQ_DET :: %d\n", ret);
            goto FAIL_IRQ;
        }
        power_supply_changed(charger->psy_chg);
    }

    ret = hl6111_create_debugfs_entries(charger);
    if (ret < 0){
        goto FAIL_DEBUGFS;
    }

    return 0;

FAIL_DEBUGFS:
    free_irq(charger->pdata->int_gpio, NULL);
FAIL_IRQ:
    power_supply_unregister(charger->psy_chg);
    devm_kfree(&client->dev, charger);
    devm_kfree(&client->dev, pdata);
    mutex_destroy(&charger->i2c_lock);
    return ret;
}


static int hl6111_charger_remove(struct i2c_client *client)
{
    struct hl6111_charger *charger = i2c_get_clientdata(client);

    LOG_DBG("START!!");

    if (client->irq){
        free_irq(client->irq, charger);
        gpio_free(charger->pdata->int_gpio);
    }

    if(charger->psy_chg)
        power_supply_unregister(charger->psy_chg);

    if (charger->pdata->det_gpio >= 0){
        gpio_free(charger->pdata->det_gpio);
    }

    return 0;
}

static void hl6111_charger_shutdown(struct i2c_client *client)
{
    struct hl6111_charger *charger = i2c_get_clientdata(client);
    LOG_DBG("START!!");

    if (client->irq){
        free_irq(client->irq, charger);
        gpio_free(charger->pdata->int_gpio);
    }

    if(charger->psy_chg)
        power_supply_unregister(charger->psy_chg);

    if (charger->pdata->det_gpio >= 0){
        gpio_free(charger->pdata->det_gpio);
    }
}

#if defined (CONFIG_PM)
static int hl6111_charger_resume(struct device *dev)
{
    return 0;
}

static int hl6111_charger_suspend(struct device *dev)
{
    return 0;
}
#else
#define hl6111_charger_suspend   NULL
#define hl6111_charger_resume    NULL
#endif

static const struct i2c_device_id hl6111_id_table[] = {
    {HL6111_I2C_NAME, 0},
    { },
};

static struct of_device_id hl6111_match_table[] = {
    { .compatible = "halo,hl6111", },
    {},
};


const struct dev_pm_ops hl6111_pm_ops = {
    .suspend = hl6111_charger_suspend,
    .resume  = hl6111_charger_resume,
};

static struct i2c_driver hl6111_driver = {
    .driver = {
        .name           = HL6111_I2C_NAME,
#if defined(CONFIG_OF)
        .of_match_table     = hl6111_match_table,
#endif
#if defined(CONFIG_PM)
        .pm                 = &hl6111_pm_ops,
#endif
    },
    .probe      = hl6111_charger_probe,
    .remove     = hl6111_charger_remove,
    .shutdown   = hl6111_charger_shutdown,
    .id_table   = hl6111_id_table,
};


static int __init hl6111_charger_init(void)
{
    int err;

    err = i2c_add_driver(&hl6111_driver);
    if (err) {
        printk(KERN_WARNING " hl6111_charger driver failed"
                                    "(errno = %d\n", err);
    } else {
        printk("Successfully added driver %s\n",
                                    hl6111_driver.driver.name);
    }

    return err;
}

static void __exit hl6111_charger_exit(void)
{
    i2c_del_driver(&hl6111_driver);
}

module_init(hl6111_charger_init);
module_exit(hl6111_charger_exit);


MODULE_AUTHOR("luke.jang@halomicro.com");
MODULE_LICENSE("GPL");

