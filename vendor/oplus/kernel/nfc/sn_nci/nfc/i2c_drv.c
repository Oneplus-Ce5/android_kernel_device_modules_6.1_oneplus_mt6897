// SPDX-License-Identifier: GPL-2.0-only
/******************************************************************************
 * Copyright (C) 2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2013-2022 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ******************************************************************************/
/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include "common_ese.h"
#include <soc/oplus/boot/boot_mode.h>

#define MIXED_CHIPSET    "mixed-chipset"
#define MAX_ID_COUNT     5
#define SUPPORT_MIXED_CHIPSET     1
#define NOT_SUPPORT_MIXED_CHIPSET     0
#define SUPPORT_CHIPSET_LIST     "SN100T|SN110T|SN220T|SN220U|SN220P|SN220E|PN560"

struct id_entry {
    u32 key;
    const char *chipset;
};


/**
 * i2c_disable_irq()
 *
 * Check if interrupt is disabled or not
 * and disable interrupt
 *
 * Return: int
 */
int i2c_disable_irq(struct nfc_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->i2c_dev.irq_enabled_lock, flags);
	if (dev->i2c_dev.irq_enabled) {
		disable_irq_nosync(dev->i2c_dev.client->irq);
		dev->i2c_dev.irq_enabled = false;
	}
	spin_unlock_irqrestore(&dev->i2c_dev.irq_enabled_lock, flags);

	return 0;
}

/**
 * i2c_enable_irq()
 *
 * Check if interrupt is enabled or not
 * and enable interrupt
 *
 * Return: int
 */
int i2c_enable_irq(struct nfc_dev *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->i2c_dev.irq_enabled_lock, flags);
	if (!dev->i2c_dev.irq_enabled) {
		dev->i2c_dev.irq_enabled = true;
		enable_irq(dev->i2c_dev.client->irq);
	}
	spin_unlock_irqrestore(&dev->i2c_dev.irq_enabled_lock, flags);

	return 0;
}

static irqreturn_t i2c_irq_handler(int irq, void *dev_id)
{
	struct nfc_dev *nfc_dev = dev_id;
	struct i2c_dev *i2c_dev = &nfc_dev->i2c_dev;

	if (device_may_wakeup(&i2c_dev->client->dev))
		pm_wakeup_event(&i2c_dev->client->dev, WAKEUP_SRC_TIMEOUT);

	i2c_disable_irq(nfc_dev);
	wake_up(&nfc_dev->read_wq);

	return IRQ_HANDLED;
}

int i2c_read(struct nfc_dev *nfc_dev, char *buf, size_t count, int timeout)
{
	int ret;
	struct i2c_dev *i2c_dev = &nfc_dev->i2c_dev;
	struct platform_gpio *nfc_gpio = &nfc_dev->configs.gpio;
 	//#ifdef OPLUS_BUG_STABILITY
	//IS_ENABLED(CONFIG_NXP_NFC_CLK_REQ_HIGH)
	struct platform_configs *nfc_config = &nfc_dev->configs;
	//#endif /*OPLUS_BUG_STABILITY*/
	/*pr_debug("%s: reading %zu bytes.\n", __func__, count);*/

	if (timeout > NCI_CMD_RSP_TIMEOUT_MS)
		timeout = NCI_CMD_RSP_TIMEOUT_MS;

	if (count > MAX_NCI_BUFFER_SIZE)
		count = MAX_NCI_BUFFER_SIZE;

	if (!gpio_get_value(nfc_gpio->irq)) {
		while (1) {
			ret = 0;
			if (!i2c_dev->irq_enabled) {
				i2c_dev->irq_enabled = true;
				enable_irq(i2c_dev->client->irq);
			}
			if (!gpio_get_value(nfc_gpio->irq)) {
				if (timeout) {
					ret = wait_event_interruptible_timeout(
						nfc_dev->read_wq,
						!i2c_dev->irq_enabled,
						msecs_to_jiffies(timeout));
					if (ret <= 0) {
						pr_err("%s: timeout error\n",
						       __func__);
						goto err;
					}
				} else {
					ret = wait_event_interruptible(
						nfc_dev->read_wq,
						!i2c_dev->irq_enabled);
					if (ret) {
						/*marked confused log of system-suspend*/
						/*pr_err("%s: err wakeup of wq\n",
						       __func__);*/
						goto err;
					}
				}
			}
			//#ifdef OPLUS_BUG_STABILITY
			//IS_ENABLED(CONFIG_NXP_NFC_CLK_REQ_HIGH)
                        if(gpio_is_valid(nfc_gpio->clkreq)) {
				if (nfc_config->sys_idle_clkreq) {
					pr_err("%s: NFC sys_idle_clkreq -->recovering  state \n", __func__);
					nfc_config->sys_idle_clkreq = false;
        	                        ret = -EREMOTEIO;
                                        goto err;
				}
                        }
			//#endif /*OPLUS_BUG_STABILITY*/
			i2c_disable_irq(nfc_dev);

			if (gpio_get_value(nfc_gpio->irq))
				break;
			if (!gpio_get_value(nfc_gpio->ven)) {
				pr_info("%s: releasing read\n", __func__);
				ret = -EIO;
				goto err;
			}
			/*
			 * NFC service wanted to close the driver so,
			 * release the calling reader thread asap.
			 *
			 * This can happen in case of nfc node close call from
			 * eSE HAL in that case the NFC HAL reader thread
			 * will again call read system call
			 */
			if (nfc_dev->release_read) {
				pr_debug("%s: releasing read\n", __func__);
				return 0;
			}
			pr_warn("%s: spurious interrupt detected\n", __func__);
		}
	}

	memset(buf, 0x00, count);
	/* Read data */
	ret = i2c_master_recv(nfc_dev->i2c_dev.client, buf, count);
	if (ret <= 0) {
		pr_err("%s: returned %d\n", __func__, ret);
		goto err;
	}
	/* check if it's response of cold reset command
	 * NFC HAL process shouldn't receive this data as
	 * command was sent by driver
	 */
	if (nfc_dev->cold_reset.rsp_pending) {
		if (IS_PROP_CMD_RSP(buf)) {
			/* Read data */
			ret = i2c_master_recv(nfc_dev->i2c_dev.client,
					      &buf[NCI_PAYLOAD_IDX],
					      buf[NCI_PAYLOAD_LEN_IDX]);
			if (ret <= 0) {
				pr_err("%s: error reading cold rst/prot rsp\n",
				       __func__);
				goto err;
			}
			wakeup_on_prop_rsp(nfc_dev, buf);
			/*
			 * NFC process doesn't know about cold reset command
			 * being sent as it was initiated by eSE process
			 * we shouldn't return any data to NFC process
			 */
			return 0;
		}
	}
err:
	return ret;
}

int i2c_write(struct nfc_dev *nfc_dev, const char *buf, size_t count,
	      int max_retry_cnt)
{
	int ret = -EINVAL;
	int retry_cnt;
	struct platform_gpio *nfc_gpio = &nfc_dev->configs.gpio;

	if (count > MAX_DL_BUFFER_SIZE)
		count = MAX_DL_BUFFER_SIZE;

	/*pr_debug("%s: writing %zu bytes.\n", __func__, count);*/
	/*
	 * Wait for any pending read for max 15ms before write
	 * This is to avoid any packet corruption during read, when
	 * the host cmds resets NFCC during any parallel read operation
	 */
	for (retry_cnt = 1; retry_cnt <= MAX_WRITE_IRQ_COUNT; retry_cnt++) {
		if (gpio_get_value(nfc_gpio->irq)) {
			pr_warn("%s: irq high during write, wait\n", __func__);
			usleep_range(WRITE_RETRY_WAIT_TIME_US,
				     WRITE_RETRY_WAIT_TIME_US + 100);
		} else {
			break;
		}
		if (retry_cnt == MAX_WRITE_IRQ_COUNT &&
		    gpio_get_value(nfc_gpio->irq)) {
			pr_warn("%s: allow after maximum wait\n", __func__);
		}
	}

	for (retry_cnt = 1; retry_cnt <= max_retry_cnt; retry_cnt++) {
		ret = i2c_master_send(nfc_dev->i2c_dev.client, buf, count);
		if (ret <= 0) {
			pr_warn("%s: write failed ret(%d), maybe in standby\n",
				__func__, ret);
			usleep_range(WRITE_RETRY_WAIT_TIME_US,
				     WRITE_RETRY_WAIT_TIME_US + 100);
		} else if (ret != count) {
			pr_err("%s: failed to write %d\n", __func__, ret);
			ret = -EIO;
		} else if (ret == count)
			break;
	}
	return ret;
}

ssize_t nfc_i2c_dev_read(struct file *filp, char __user *buf, size_t count,
			 loff_t *offset)
{
	int ret;
	struct nfc_dev *nfc_dev = (struct nfc_dev *)filp->private_data;

	if (!nfc_dev) {
		pr_err("%s: device doesn't exist anymore\n", __func__);
		return -ENODEV;
	}
	mutex_lock(&nfc_dev->read_mutex);
	if (filp->f_flags & O_NONBLOCK) {
		ret = i2c_master_recv(nfc_dev->i2c_dev.client, nfc_dev->read_kbuf, count);
		pr_debug("%s: NONBLOCK read ret = %d\n", __func__, ret);
	} else {
		ret = i2c_read(nfc_dev, nfc_dev->read_kbuf, count, 0);
	}
	if (ret > 0) {
		if (copy_to_user(buf, nfc_dev->read_kbuf, ret)) {
			pr_warn("%s: failed to copy to user space\n", __func__);
			ret = -EFAULT;
		}
	}
	mutex_unlock(&nfc_dev->read_mutex);
	return ret;
}

ssize_t nfc_i2c_dev_write(struct file *filp, const char __user *buf,
			  size_t count, loff_t *offset)
{
	int ret;
	struct nfc_dev *nfc_dev = (struct nfc_dev *)filp->private_data;

	if (count > MAX_DL_BUFFER_SIZE)
		count = MAX_DL_BUFFER_SIZE;

	if (!nfc_dev) {
		pr_err("%s: device doesn't exist anymore\n", __func__);
		return -ENODEV;
	}

	mutex_lock(&nfc_dev->write_mutex);
	if (copy_from_user(nfc_dev->write_kbuf, buf, count)) {
		pr_err("%s: failed to copy from user space\n", __func__);
		mutex_unlock(&nfc_dev->write_mutex);
		return -EFAULT;
	}
	ret = i2c_write(nfc_dev, nfc_dev->write_kbuf, count, NO_RETRY);
	mutex_unlock(&nfc_dev->write_mutex);
	return ret;
}

static const struct file_operations nfc_i2c_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = nfc_i2c_dev_read,
	.write = nfc_i2c_dev_write,
	.open = nfc_dev_open,
	.flush = nfc_dev_flush,
	.release = nfc_dev_close,
	.unlocked_ioctl = nfc_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nfc_dev_compat_ioctl,
#endif
};

static int read_id_properties(struct device_node *np, u32 id_count, struct id_entry *id_entries)
{
    int err;
    u32 i;
    char propname[30];

    for (i = 0; i < id_count; i++) {
        snprintf(propname, sizeof(propname), "id-%u-key", i);
        err = of_property_read_u32(np, propname, &id_entries[i].key);
        if (err) {
          pr_err("Failed to read dts node:%s\n",propname);
          return err;
        }

        snprintf(propname, sizeof(propname), "id-%u-value-chipset", i);
        err = of_property_read_string(np, propname, &id_entries[i].chipset);
        if (err) {
          pr_err("Failed to read dts node:%s\n",propname);
          return err;
        }
    }
    pr_info("read_id_properties success");
    return 0;
}

static int get_gpio_value(struct device_node *np, int *gpio_value)
{
    int gpio_num = of_get_named_gpio(np, "id-gpio", 0);

    if (!gpio_is_valid(gpio_num)) {
        pr_err("id-gpio is not valid\n");
        return -EINVAL;
    }

    *gpio_value = gpio_get_value(gpio_num);
    pr_info("%s, id gpio value is %d", __func__, *gpio_value);
    return 0;
}

static int checkNfcChip(struct device *dev)
{
    struct device_node *np = NULL;
    u32 id_count;
    int i, gpio_value, err;
    bool found = false;
    struct id_entry *id_entries = NULL;
    uint32_t mixed_chipset;

    if (NULL == dev)
    {
        pr_err("%s dev is NULL", __func__);
        return -ENOENT;
    }

    np = dev->of_node;

    if (NULL == np)
    {
        pr_err("%s dev->of_node is NULL", __func__);
        return -ENOENT;
    }

    if (of_property_read_u32(np, MIXED_CHIPSET, &mixed_chipset))
    {
        pr_info("%s, read dts property mixed-chipset failed", __func__);
        return 0;
    }
    else
    {
        if (SUPPORT_MIXED_CHIPSET == mixed_chipset)
        {
            pr_info("%s, the value of dts property mixed-chipset is 1(true)", __func__);

            err = of_property_read_u32(np, "id_count", &id_count);
            if (err)
            {
                pr_err("%s read dts property id_count failed", __func__);
                return err;
            }

            if (id_count >= MAX_ID_COUNT)
            {
                pr_err("%s error: id_count is more than %d", __func__, MAX_ID_COUNT);
                return -ENOENT;
            }

            id_entries = kzalloc(sizeof(struct id_entry) * id_count, GFP_DMA | GFP_KERNEL);
            if(NULL == id_entries)
            {
                pr_err("%s error: can not kzalloc memory for id_entry", __func__);
                return -ENOMEM;
            }

            err = read_id_properties(np, id_count,id_entries);
            if (err)
            {
                pr_err("%s error: read_id_properties failed", __func__);
                kfree(id_entries);
                return err;
            }

            err = get_gpio_value(np, &gpio_value);
            if (err)
            {
                pr_err("%s error: get_gpio_value failed", __func__);
                kfree(id_entries);
                return err;
            }

            for (i = 0; i < id_count; i++)
            {
                if (id_entries[i].key == gpio_value)
                {
                    if (strstr(SUPPORT_CHIPSET_LIST, id_entries[i].chipset) == NULL)
                    {
                        pr_err("%s this nfc chipset:%s does not correspond to this nfc driver", __func__, id_entries[i].chipset);
                        err = -EINVAL;
                        kfree(id_entries);
                        return err;
                    }

                    pr_debug("%s this nfc chipset:%s corresponds to this nfc driver", __func__, id_entries[i].chipset);
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                pr_err("%s no matching key found for GPIO value\n", __func__);
                err = -EINVAL;
                kfree(id_entries);
                return err;
            }

            pr_info("%s checkNfcChip success\n", __func__);
            kfree(id_entries);
            return 0;

        }
        else if (NOT_SUPPORT_MIXED_CHIPSET == mixed_chipset)
        {
            pr_info("%s, the value of dts property mixed-chipset is 0(false)", __func__);
            return 0;
        }
        else
        {
            pr_err("%s, mixed-chipset's value is wrong,it is neither 1 nor 0", __func__);
            return -ENOENT;
        }
    }
}
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6,4,0))
int nfc_i2c_dev_probe(struct i2c_client *client)
#else
int nfc_i2c_dev_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	int ret = 0;
	struct nfc_dev *nfc_dev = NULL;
	struct i2c_dev *i2c_dev = NULL;
	struct platform_configs *nfc_configs = NULL;
	struct platform_gpio *nfc_gpio = NULL;
	pr_debug("%s: enter\n", __func__);
	ret = checkNfcChip(&client->dev);
	if (ret) {
		pr_err("NxpDrv: %s: failed to checkNfcChip\n", __func__);
		struct pinctrl * pinctrl = NULL;
		struct pinctrl_state * pinctrl_clk_nc = NULL;
		pinctrl=devm_pinctrl_get(&client->dev);
		if (IS_ERR(pinctrl)) {
			pr_err("pinctrl fail\n");
		} else {
			pinctrl_clk_nc = pinctrl_lookup_state(pinctrl, "state_nfc_clk_nc");
			if (IS_ERR(pinctrl_clk_nc)) {
				pr_err("pinctrl_clk_nc_init fail\n");
			} else {
				pinctrl_select_state(pinctrl, pinctrl_clk_nc);
				pr_err(" pinctrl_clk_nc change to mode gpio\n");
			}
		}
		goto err;
	}

	nfc_dev = kzalloc(sizeof(struct nfc_dev), GFP_KERNEL);
	if (nfc_dev == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	nfc_configs = &nfc_dev->configs;
	nfc_gpio = &nfc_configs->gpio;
	/* retrieve details of gpios from dt */
	ret = nfc_parse_dt(&client->dev,nfc_configs, PLATFORM_IF_I2C);
	if (ret) {
		pr_err("%s: failed to parse dt\n", __func__);
		goto err_free_nfc_dev;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_free_nfc_dev;
	}
	nfc_dev->read_kbuf = kzalloc(MAX_NCI_BUFFER_SIZE, GFP_DMA | GFP_KERNEL);
	if (!nfc_dev->read_kbuf) {
		ret = -ENOMEM;
		goto err_free_nfc_dev;
	}
	nfc_dev->write_kbuf = kzalloc(MAX_DL_BUFFER_SIZE, GFP_DMA | GFP_KERNEL);
	if (!nfc_dev->write_kbuf) {
		ret = -ENOMEM;
		goto err_free_read_kbuf;
	}
	nfc_dev->interface = PLATFORM_IF_I2C;
	nfc_dev->nfc_state = NFC_STATE_NCI;
	nfc_dev->i2c_dev.client = client;
	i2c_dev = &nfc_dev->i2c_dev;
	nfc_dev->nfc_read = i2c_read;
	nfc_dev->nfc_write = i2c_write;
	nfc_dev->nfc_enable_intr = i2c_enable_irq;
	nfc_dev->nfc_disable_intr = i2c_disable_irq;
	ret = configure_gpio(nfc_gpio->ven, GPIO_OUTPUT);
	if (ret) {
		pr_err("%s: unable to request nfc reset gpio [%d]\n", __func__,
		       nfc_gpio->ven);
		goto err_free_write_kbuf;
	}
	ret = configure_gpio(nfc_gpio->irq, GPIO_IRQ);
	if (ret <= 0) {
		pr_err("%s: unable to request nfc irq gpio [%d]\n", __func__,
		       nfc_gpio->irq);
		goto err_free_gpio;
	}
	client->irq = ret;
	ret = configure_gpio(nfc_gpio->dwl_req, GPIO_OUTPUT);
	if (ret) {
		pr_err("%s: unable to request nfc firm downl gpio [%d]\n",
		       __func__, nfc_gpio->dwl_req);
	}
	/* init mutex and queues */
	init_waitqueue_head(&nfc_dev->read_wq);
	mutex_init(&nfc_dev->read_mutex);
	mutex_init(&nfc_dev->write_mutex);
	mutex_init(&nfc_dev->dev_ref_mutex);
	spin_lock_init(&i2c_dev->irq_enabled_lock);
	common_ese_init(nfc_dev);
	ret = nfc_misc_register(nfc_dev, &nfc_i2c_dev_fops, DEV_COUNT,
				NFC_CHAR_DEV_NAME, CLASS_NAME);
	if (ret) {
		pr_err("%s: nfc_misc_register failed\n", __func__);
		goto err_mutex_destroy;
	}
	/* interrupt initializations */
	pr_info("%s: requesting IRQ %d\n", __func__, client->irq);
	i2c_dev->irq_enabled = true;
	ret = request_irq(client->irq, i2c_irq_handler, IRQF_TRIGGER_HIGH,
			  client->name, nfc_dev);
	if (ret) {
		pr_err("%s: request_irq failed\n", __func__);
		goto err_nfc_misc_unregister;
	}
	i2c_disable_irq(nfc_dev);
        ret = nfcc_hw_check(nfc_dev);
        if (ret <= 0)
            pr_err("nfc hw check failed ret %d\n", ret);

	gpio_set_ven(nfc_dev, 1);
	gpio_set_ven(nfc_dev, 0);
	gpio_set_ven(nfc_dev, 1);
	device_init_wakeup(&client->dev, true);
	i2c_set_clientdata(client, nfc_dev);
	i2c_dev->irq_wake_up = false;

	dev_err(&client->dev,"%s: get boot mode = %d\n", __func__, get_boot_mode());
#ifdef CONFIG_OPLUS_SYSTEM_KERNEL_QCOM
	if(get_boot_mode() == MSM_BOOT_MODE__FACTORY) {
#else
	if(get_boot_mode() == 4) {
#endif
		dev_err(&client->dev,"%s: enter ftm mode, set ven = 0\n", __func__);
		gpio_set_ven(nfc_dev, 0);
	}

	pr_info("%s: probing nfc i2c successfully\n", __func__);
	return 0;
err_nfc_misc_unregister:
	nfc_misc_unregister(nfc_dev, DEV_COUNT);
err_mutex_destroy:
	mutex_destroy(&nfc_dev->dev_ref_mutex);
	mutex_destroy(&nfc_dev->read_mutex);
	mutex_destroy(&nfc_dev->write_mutex);
err_free_gpio:
	gpio_free_all(nfc_dev);
err_free_write_kbuf:
	kfree(nfc_dev->write_kbuf);
err_free_read_kbuf:
	kfree(nfc_dev->read_kbuf);
err_free_nfc_dev:
	kfree(nfc_dev);
err:
	pr_err("%s: probing not successful, check hardware\n", __func__);
	return ret;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
void nfc_i2c_dev_remove(struct i2c_client *client)
#else
int nfc_i2c_dev_remove(struct i2c_client *client)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	int ret = 0;
#endif
	struct nfc_dev *nfc_dev = NULL;

	pr_info("%s: remove device\n", __func__);
	nfc_dev = i2c_get_clientdata(client);
	if (!nfc_dev) {
		pr_err("%s: device doesn't exist anymore\n", __func__);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
		return;
#else
		ret = -ENODEV;
		return ret;
#endif
	}
	if (nfc_dev->dev_ref_count > 0) {
		pr_err("%s: device already in use\n", __func__);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
		return;
#else
		return -EBUSY;
#endif
	}
	device_init_wakeup(&client->dev, false);
	free_irq(client->irq, nfc_dev);
	nfc_misc_unregister(nfc_dev, DEV_COUNT);
	mutex_destroy(&nfc_dev->read_mutex);
	mutex_destroy(&nfc_dev->write_mutex);
	gpio_free_all(nfc_dev);
	kfree(nfc_dev->read_kbuf);
	kfree(nfc_dev->write_kbuf);
	kfree(nfc_dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return ret;
#endif
}

int nfc_i2c_dev_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct nfc_dev *nfc_dev = i2c_get_clientdata(client);
	struct i2c_dev *i2c_dev = NULL;
	struct platform_gpio *nfc_gpio = &nfc_dev->configs.gpio;
	//#ifdef OPLUS_BUG_STABILITY
	//IS_ENABLED(CONFIG_NXP_NFC_CLK_REQ_HIGH)
	struct platform_configs *nfc_config = &nfc_dev->configs;
	//#endif /*OPLUS_BUG_STABILITY*/
	if (!nfc_dev) {
		pr_err("%s: device doesn't exist anymore\n", __func__);
		return -ENODEV;
	}
	i2c_dev = &nfc_dev->i2c_dev;

	if (device_may_wakeup(&client->dev) && i2c_dev->irq_enabled) {
		if (!enable_irq_wake(client->irq))
			i2c_dev->irq_wake_up = true;
	}
	//#ifdef OPLUS_BUG_STABILITY
	//IS_ENABLED(CONFIG_NXP_NFC_CLK_REQ_HIGH)
	if(gpio_is_valid(nfc_gpio->clkreq)) {
		if (gpio_get_value(nfc_gpio->clkreq)) {
			nfc_config->sys_idle_clkreq = true;
		} else {
			nfc_config->sys_idle_clkreq = false;
		}
		pr_err("%s: clkreq = %d , sys_idle_clkreq = %d  \n",__func__ ,gpio_get_value(nfc_gpio->clkreq) , nfc_config->sys_idle_clkreq);
	}
	//#endif /*OPLUS_BUG_STABILITY*/

	return 0;
}

int nfc_i2c_dev_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);
	struct nfc_dev *nfc_dev = i2c_get_clientdata(client);
	struct i2c_dev *i2c_dev = NULL;
	struct platform_gpio *nfc_gpio = &nfc_dev->configs.gpio;
	if (!nfc_dev) {
		pr_err("%s: device doesn't exist anymore\n", __func__);
		return -ENODEV;
	}
	i2c_dev = &nfc_dev->i2c_dev;

	if (device_may_wakeup(&client->dev) && i2c_dev->irq_wake_up) {
		if (!disable_irq_wake(client->irq))
			i2c_dev->irq_wake_up = false;
	}
	pr_debug("%s: irq_wake_up = %d", __func__, i2c_dev->irq_wake_up);

	if(gpio_is_valid(nfc_gpio->clkreq)){
		pr_info("%s: clkreq = %d \n",__func__ ,gpio_get_value(nfc_gpio->clkreq));
	}

	return 0;
}

static const struct i2c_device_id nfc_i2c_dev_id[] = { { NFC_I2C_DEV_ID, 0 },
						       {} };

static const struct of_device_id nfc_i2c_dev_match_table[] = {
	{
		.compatible = NFC_I2C_DRV_STR,
	},
	{}
};

static const struct dev_pm_ops nfc_i2c_dev_pm_ops = { SET_SYSTEM_SLEEP_PM_OPS(
	nfc_i2c_dev_suspend, nfc_i2c_dev_resume) };

static struct i2c_driver nfc_i2c_dev_driver = {
	.id_table = nfc_i2c_dev_id,
	.probe = nfc_i2c_dev_probe,
	.remove = nfc_i2c_dev_remove,
	.driver = {
		.name = NFC_I2C_DRV_STR,
		.pm = &nfc_i2c_dev_pm_ops,
		.of_match_table = nfc_i2c_dev_match_table,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

MODULE_DEVICE_TABLE(of, nfc_i2c_dev_match_table);

static int __init nfc_i2c_dev_init(void)
{
	int ret = 0;

	pr_info("%s: Loading NXP NFC I2C driver\n", __func__);
	ret = i2c_add_driver(&nfc_i2c_dev_driver);
	if (ret != 0)
		pr_err("%s: NFC I2C add driver error ret %d\n", __func__, ret);
	return ret;
}

module_init(nfc_i2c_dev_init);

static void __exit nfc_i2c_dev_exit(void)
{
	pr_info("%s: Unloading NXP NFC I2C driver\n", __func__);
	i2c_del_driver(&nfc_i2c_dev_driver);
}

module_exit(nfc_i2c_dev_exit);

MODULE_DESCRIPTION("NXP NFC I2C driver");
MODULE_LICENSE("GPL");
