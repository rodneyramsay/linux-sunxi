/* drivers/pwm/pwm-sunx.c
 *
 * Copyright (c) 2013 Fiddlesticks
 *
 * sunxi  series PWM device core
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
*/

#include <linux/kobject.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm.h>

#include <linux/io.h>
#include <plat/sys_config.h>

#define PWM_BASE		(0xf1c20c00)
#define PWM_CTRL		(PWM_BASE + 0x200)

#define PWM_MODE_0		(1 << 7)
#define PWM_MODE_1		(1 << 22)
#define PWM_CLK_EN_0	(1 << 6)
#define PWM_CLK_EN_1	(1 << 21)
#define PWM_PLRT_0		(1 << 5)
#define PWM_PLRT_1		(1 << 20)
#define PWM_EN_0		(1 << 4)
#define PWM_EN_1		(1 << 19)

enum pwm_polarity{
        PWM_POLARITY_NORMAL,
        PWM_POLARITY_INVERSED,
};

struct sunxi_chip {
	struct platform_device	*pdev;
	void __iomem	*mmio_ctrl_base;
	void __iomem	*mmio_prd_base;
	unsigned int	prscl;
	unsigned int	period_ns;
	unsigned int	duty_ns;
	unsigned int	prd_value;
	unsigned char	pwm_id;
	const char		*label;
//	char			request;
	enum pwm_polarity 	polarity;
	struct device	*dev;
//	struct pwm_chip	chip;
};

struct sunxi_chip *entry_pwm_chip[2];
static u32 pwm_gpio_hdle;
static nr_pwm_ch = 2;

#define to_sunxi_chip(chip)	container_of(chip, struct sunxi_chip, chip)

//static int sunxi_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
static int sunxi_pwm_enable(int number)
{
	//struct sunxi_chip *sunxi_cp = to_sunxi_chip(chip);

	struct sunxi_chip *sunxi_cp =  entry_pwm_chip[number];
	unsigned int ctrl_value;
	unsigned long flags;

	local_irq_save(flags);

	ctrl_value = __raw_readl(sunxi_cp->mmio_ctrl_base);
	if(sunxi_cp->pwm_id){
		ctrl_value |= PWM_EN_1;
	}else{
		ctrl_value |= PWM_EN_0;
	}
	__raw_writel(ctrl_value, sunxi_cp->mmio_ctrl_base);

	local_irq_restore(flags);

	return 0;
}

static void sunxi_pwm_disable(int number)
{
//	struct sunxi_chip *sunxi_cp = to_sunxi_chip(chip);
	struct sunxi_chip *sunxi_cp =  entry_pwm_chip[number];
	unsigned int ctrl_value;
	unsigned long flags;

	local_irq_save(flags);

	ctrl_value = __raw_readl(sunxi_cp->mmio_ctrl_base);
	if(sunxi_cp->pwm_id){
		ctrl_value &= ~PWM_EN_1;
	}else{
		ctrl_value &= ~PWM_EN_0;
	}
	__raw_writel(ctrl_value, sunxi_cp->mmio_ctrl_base);

	local_irq_restore(flags);
}

struct sunxi_pwm_period_map{
	unsigned int prsl;
	unsigned int div;
};
/* pwm_map be use to map register value to the frequency factor(prsl)*/
static struct sunxi_pwm_period_map pwm_map[10]={
	{0x0,120},
	{0x1,180},
	{0x2,240},
	{0x3,360},
	{0x4,480},
	{0x8,12000},
	{0x9,24000},
	{0xa,36000},
	{0xb,48000},
	{0xc,72000},
};
#define SUNXI_MAX_CYS (256)
#define T_MAX (3000)
#define T_MIN (5)
#define CLOCK_SRC (24)

static int 
period_calc(int duty_ns,int period_ns,unsigned int *cys,
	    unsigned int *act_cys,unsigned int *prsl)
{
	int i,j,k,duty_ns_limit = duty_ns *24,period_ns_limit = period_ns*24;
	if(duty_ns == 0){
		*cys = 0;
		*act_cys = 0;
		*prsl = pwm_map[0].prsl;
		return 0;
	}

	if(duty_ns == period_ns){
		*cys = 254;
		*act_cys = 255;
		*prsl = pwm_map[9].prsl;
		return 0;
	}

	printk("duty_ns_limit %d period_ns_limit %d\n",duty_ns_limit,period_ns_limit);
	printk("duty_ns_limit/24 %d period_ns_limit/24 %d\n",duty_ns_limit / 24,period_ns_limit / 24);
	for(k=9;k>=0;k--){
		if(((pwm_map[k].div * SUNXI_MAX_CYS)  < duty_ns_limit)
		   || ((pwm_map[k].div * SUNXI_MAX_CYS) < period_ns_limit))
			break;
		for(i=SUNXI_MAX_CYS;i>0;i--){
			if((pwm_map[k].div * i) < period_ns_limit)
				break;
			if((pwm_map[k].div * i) == period_ns_limit){
				printk("pwm_map[k].div %d i %d div * i %d\n",pwm_map[k].div,i,pwm_map[k].div * i);
				for(j=i;j>0;j--){
					if((pwm_map[k].div * j) < duty_ns_limit)
						break;
					if((pwm_map[k].div * j) == duty_ns_limit){
						*cys = i-1;
						*act_cys = j;
						*prsl = pwm_map[k].prsl;
						return 0;
					}
				}
			}
		}
	}
	printk(KERN_WARNING "don`t found perfect matching!\n");
	for(i=SUNXI_MAX_CYS;i>0;i--)
		for(j=i;j>0;j--){
			if(i == SUNXI_MAX_CYS)
				j--;
			if(i*duty_ns == period_ns*j){
				*cys = i-1;
				*act_cys = j;
				*prsl =0;
				return 0;
			}
		}
	printk(KERN_ERR "don`t found matching!\n");
	return -1;
}

/*
 * variable: act_cys, cys:
 * period_ns = cys * T;
 * duty_ns	 = act_cys * T;
 * */

static int sunxi_pwm_config(int number,
			    int duty_ns, int period_ns)
{
	//struct sunxi_chip *sunxi_cp = to_sunxi_chip(chip);
	struct sunxi_chip *sunxi_cp =  entry_pwm_chip[number];
	unsigned int cys = 0,act_cys= 0,prd_value=0,ctrl_value,prsl=0;	
	unsigned long flags;
	int ret;
	/*
	 * duty_ns and period_ns limit:
	 * because the sunxi pwm controller only have 8 bit to record period_ns
	 * and duty_ns informations so it have some limit.
	 * 1.period_ns / duty_ns <= 256
	 * 2.period_ns < T * 256  (T = prsl / 24MHZ)
	 * 3.duty_ns < T * 255
	 * */
	if (period_ns > (SUNXI_MAX_CYS+1) * T_MAX  || duty_ns > SUNXI_MAX_CYS * T_MAX 
	    || duty_ns > period_ns 	|| duty_ns < period_ns /256)
		return -ERANGE;

	if (period_ns == sunxi_cp->period_ns &&
	    duty_ns == sunxi_cp->duty_ns)
		return 0;

	ret= period_calc(duty_ns,period_ns,&cys,&act_cys,&prsl);
	if(ret){
		dev_err(sunxi_cp->dev, "the Duty Cycle can support!\n");
		return -1;
	}
	
	if(sunxi_cp->period_ns != period_ns)
		sunxi_cp->period_ns = period_ns;

	if(sunxi_cp->duty_ns != duty_ns)
		sunxi_cp->duty_ns = duty_ns;
	printk(KERN_INFO "duty_ns %d period_ns %d cys 0x%x act_cys 0x%x prsl 0x%x\n",
	       duty_ns,period_ns,cys,act_cys,prsl);
	prd_value =  ((cys&0xff) << 16) | (act_cys&0xff);

	/* Update the PWM register block. */
	sunxi_cp->polarity = PWM_POLARITY_NORMAL;

	local_irq_save(flags);

	ctrl_value = __raw_readl(sunxi_cp->mmio_ctrl_base);
	if(sunxi_cp->polarity == PWM_POLARITY_INVERSED){
		if(sunxi_cp->pwm_id == PWM_POLARITY_INVERSED){
			ctrl_value &= ~(PWM_PLRT_1 | PWM_MODE_1);
			ctrl_value |= PWM_CLK_EN_1;
		}else{
			ctrl_value &= ~(PWM_PLRT_0 | PWM_MODE_0);
			ctrl_value |= PWM_CLK_EN_0;
		}
	}else if(sunxi_cp->polarity == PWM_POLARITY_NORMAL){
		if(sunxi_cp->pwm_id){
			ctrl_value &= ~PWM_MODE_1;
			ctrl_value |= (PWM_PLRT_1 | PWM_CLK_EN_1);
		}else{
			ctrl_value &= ~PWM_MODE_0;
			ctrl_value |= (PWM_PLRT_0 | PWM_CLK_EN_0);
		}
	}else{
		printk(KERN_WARNING "err polarity! use PWM_POLARITY_NORMAL NOW!\n");
	}
	__raw_writel(ctrl_value, sunxi_cp->mmio_ctrl_base);
	ctrl_value &= ~(0xf << sunxi_cp->pwm_id*15);
	ctrl_value |= (prsl << sunxi_cp->pwm_id*15);
	mdelay (200); 
	__raw_writel(ctrl_value, sunxi_cp->mmio_ctrl_base);
	__raw_writel(prd_value, sunxi_cp->mmio_prd_base);
	local_irq_restore(flags);

	return 0;
}


int
sunxi_set_polarity(int number, enum pwm_polarity polarity)
{
	//struct sunxi_chip *sunxi_cp = to_sunxi_chip(chip);
	struct sunxi_chip *sunxi_cp =  entry_pwm_chip[number];
	unsigned int ctrl_value;
	if(sunxi_cp->polarity == polarity)
		return 0;
	sunxi_cp->polarity = polarity;
	ctrl_value = __raw_readl(sunxi_cp->mmio_ctrl_base);
	if(sunxi_cp->polarity == PWM_POLARITY_NORMAL){
		if(sunxi_cp->pwm_id)
			ctrl_value |= PWM_PLRT_1;
		else
			ctrl_value |= PWM_PLRT_0;
	}else if(sunxi_cp->polarity == PWM_POLARITY_INVERSED){
	
		if(sunxi_cp->pwm_id)
			ctrl_value &= ~PWM_PLRT_1;
		else
			ctrl_value &= ~PWM_PLRT_0;
	}else{
		printk(KERN_WARNING "err polarity! use PWM_POLARITY_NORMAL NOW!\n");
		sunxi_cp->polarity = PWM_POLARITY_NORMAL;
		if(sunxi_cp->pwm_id)
			ctrl_value |= PWM_PLRT_1;
		else
			ctrl_value |= PWM_PLRT_0;
	}
	__raw_writel(ctrl_value, sunxi_cp->mmio_ctrl_base);
	return 0;
}

static struct pwm_device  *test_pwm_point[2];	

/*
 *echo "channel_numer  duty_ns period_ns" > /sys/kernel/sunxi_pwm/start
 * */
static ssize_t function_test_store(struct kobject *kobj, struct kobj_attribute *attr,
				   const char *buf, size_t size)
{
//	struct pwm_device *test_pwm;	
	char *after,*tmp_p;
	char tmp[16];
	unsigned long flags=0,value;
	int channel,duty_ns,period_ns,i,ret;
	printk("test_pwm0 %p test_pwm1 %p! %d\n",test_pwm_point[0],test_pwm_point[1],__LINE__);
	after = tmp;
	memset(after,0,16);
	for(tmp_p =(char *)buf,i=0;tmp_p < (buf+size);tmp_p++ ){
		if((*tmp_p<'0') || (*tmp_p>'9')){
			if(!flags)
				continue;
			else
				break;
		}else{
			flags = 1;
			tmp[i++] = *tmp_p;
		}
	}
	if(flags == 0){
		printk("%s need get 3 args but only get 0 now!\n",__func__);
		return size;
	}
	value = simple_strtoul(tmp, &after, 10);
	channel = (int) value;
	printk("channel is %d\n",channel);
	if(channel > 1 || channel < 0){
		printk(KERN_ERR "channel is %d don`t support!\n",channel);
		return size;
	}
#if 0
	if(channel){
		test_pwm = pwm_request(1, "sunxi-pwm.1");
	}else{
		test_pwm =  pwm_request(0, "sunxi-pwm.0");
	}
	if (IS_ERR(test_pwm)) {
		printk(KERN_WARNING "NO.%d PWM is busy!\n",channel);
		printk(KERN_WARNING "test_pwm %p!\n",test_pwm);
		return size;
	}else{
		test_pwm_point[channel] = test_pwm;
	}
#endif
	after	= tmp;
	memset(after,0,16);

	for(i=0;tmp_p < (buf+size);tmp_p++ ){
		if((*tmp_p<'0') || (*tmp_p>'9')){
			if(flags)
				continue;
			else
				break;
		}else{
			flags = 0;
			tmp[i++]=*tmp_p;
		}
	}

	if(flags == 1){
		printk("%s need get 3 args but only get 1 now!\n",__func__);
		goto end;
	}

	value = simple_strtoul(tmp, &after, 10);
	duty_ns = (int)value;

	after = tmp;
	memset(after,0,16);
	for(i=0;tmp_p < (buf+size);tmp_p++ ){
		if((*tmp_p<'0') || (*tmp_p>'9')){
			if(!flags)
				continue;
			else
				break;
		}else{
			flags = 1;
			tmp[i++] = *tmp_p;
		}
	}
	if(flags == 0){
		printk("%s need get 3 args but only get 2 now!\n",__func__);
		goto end;
	}
	value = simple_strtoul(tmp, &after, 10);
	period_ns = (int) value;
	printk("duty_ns %d period_ns %d\n",duty_ns,period_ns);

	ret= sunxi_pwm_config(channel,duty_ns,period_ns);
	if(ret)
		goto end;
	if(sunxi_pwm_enable(channel)){
		printk("pwm.%d enable fail\n",channel);
		goto end;
	}
//	test_pwm = NULL;
//	printk("test_pwm0 %p test_pwm1 %p! %d\n",test_pwm_point[0],test_pwm_point[1],__LINE__);
	return size;
end:
	printk("err ? %d\n",__LINE__);
//	pwm_free(test_pwm);
	return size;
}

static ssize_t stop_free_store(struct kobject *kobj, struct kobj_attribute *attr,
			       const char *buf, size_t size)
{
	char *after;
	unsigned long value;
	int channel;
	value = simple_strtoul(buf, &after, 10);
	if(value > 1){
		printk("the pwm.%lu is inexistence",value);
	}
	channel =(int) value;
	printk("test_pwm0 %p test_pwm1 %p! %d\n",test_pwm_point[0],test_pwm_point[1],__LINE__);
	sunxi_pwm_disable(channel);
//	sunxi_pwm_free(test_pwm_point[channel]);
	return size;
}

static struct kobj_attribute start_attribute =
	__ATTR(start, 0220, NULL, function_test_store);

static struct kobj_attribute stop_attribute =
	__ATTR(stop, 0220, NULL, stop_free_store);
static struct kobject *sunxi_pwm_kobj;

static struct attribute *attrs[] = {
	&start_attribute.attr,
	&stop_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

static int __init sunxi_pwm_init(void)
{
	int i;
	for(i=0; i<nr_pwm_ch; i++){
		entry_pwm_chip[i] = kmalloc(sizeof(*entry_pwm_chip[i]), GFP_KERNEL);
		if(!entry_pwm_chip[i])
			return -1;
		entry_pwm_chip[i]->mmio_prd_base = (void __iomem *)(PWM_CTRL + (i+1)* 0x4);
		entry_pwm_chip[i]->mmio_ctrl_base = (void __iomem *)PWM_CTRL;
	}

	return 0;
}

static int __init sunxi_pwm_test_init(void)
{
	int ret;

        if(0 == (pwm_gpio_hdle = gpio_request_ex("pwm_para", "pwm_ch0"))){
		printk("try to request pwm_para gpio failed. \n");
		return -1;
        }

	ret = sunxi_pwm_init();

	if (ret) {
		pr_err("sunxi_pwm: init error\n");
		return -1;
	}

	sunxi_pwm_kobj = kobject_create_and_add("sunxi_pwm", kernel_kobj);
	if(!sunxi_pwm_kobj) {
		return -ENOMEM;
	}

	ret  = sysfs_create_group(sunxi_pwm_kobj, &attr_group);
	if(ret) {
		kobject_put(sunxi_pwm_kobj);
	}

	return ret;
}

static void __exit sunxi_pwm_test_exit(void)
{
	int i;

	gpio_release(pwm_gpio_hdle, 2);
	kobject_put(sunxi_pwm_kobj);

	for(i=0; i<nr_pwm_ch; i++){
                kfree(entry_pwm_chip[i]);
        }
}


module_init(sunxi_pwm_test_init);
module_exit(sunxi_pwm_test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fiddlesticks");
MODULE_DESCRIPTION("Allwinner A10 PWM driver");
