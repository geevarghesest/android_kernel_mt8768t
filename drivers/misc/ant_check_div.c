#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#define CONFIG_ANT_DIV_SYS
//#define CONFIG_ANT_DIV_IRQ

struct ant_div_check_info
{
	struct mutex io_lock;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	int irq;
	int ant_div_check_state;
	struct input_dev *ipdev;
#ifdef CONFIG_ANT_DIV_SYS
	struct class *ant_div_sys_class;
#endif
};

struct ant_div_check_info *global_ant_div_info;

/*function*/
#ifdef CONFIG_ANT_DIV_IRQ
static irqreturn_t ant_div_interrupt(int irq, void *data)
{
	struct ant_div_check_info *ant_div_info = data;
	int ant_div_gpio;

	ant_div_gpio = gpio_get_value_cansleep(ant_div_info->irq_gpio);
	pr_err("Macle irq interrupt gpio = %d\n", ant_div_gpio);
	if(ant_div_gpio == ant_div_info->ant_div_check_state){
		return IRQ_HANDLED;
	}else{
		ant_div_info->ant_div_check_state = ant_div_gpio;
		pr_err("Macle report key s ");
	}
	if (ant_div_gpio) {
			input_report_key(ant_div_info->ipdev, DIV_ANT_CONNECT, 1);
			input_report_key(ant_div_info->ipdev, DIV_ANT_CONNECT, 0);
			input_sync(ant_div_info->ipdev);
	}else{
			input_report_key(ant_div_info->ipdev, DIV_ANT_UNCONNECT, 1);
			input_report_key(ant_div_info->ipdev, DIV_ANT_UNCONNECT, 0);
			input_sync(ant_div_info->ipdev);
	}
	//enable_irq(irq);
	//mutex_unlock(&ant_div_info->io_lock);
    return IRQ_HANDLED;
}
#endif

static int ant_div_parse_dt(struct device *dev, struct ant_div_check_info *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "ant_check_div_gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	pr_info("Macle irq_gpio=%d\n", pdata->irq_gpio);
	return 0;
}


#ifdef CONFIG_ANT_DIV_SYS
static ssize_t ant_div_state_show(struct class *class, struct class_attribute *attr, char *buf)
{
	int state;

	if (global_ant_div_info->ant_div_check_state) {
		state = 3;
	}else{
		state = 2;
	}
	pr_err("Macle ant_state_show state = %d, custome_state=%d\n", global_ant_div_info->ant_div_check_state, state);
	return sprintf(buf, "%d\n", state);
}

static struct class_attribute ant_div_state = __ATTR(ant_div_state, S_IRUGO, ant_div_state_show, NULL);

static int ant_div_register_class_dev(struct ant_div_check_info *ant_div_info){
	int err;
	if (!ant_div_info->ant_div_sys_class) {
		ant_div_info->ant_div_sys_class = class_create(THIS_MODULE, "ant_div_class");
		if (IS_ERR(ant_div_info->ant_div_sys_class)){
			ant_div_info->ant_div_sys_class = NULL;
			pr_debug("could not allocate ant_div_class\n");
			return -1;
		}
	}

	err = class_create_file(ant_div_info->ant_div_sys_class, &ant_div_state);
	if(err < 0){
		class_destroy(ant_div_info->ant_div_sys_class);
		return -1;
	}
	return 0;
}
#endif

static int ant_div_probe(struct platform_device *pdev)
{
	int rc = 0;
	int err;
	struct ant_div_check_info *ant_div_info;
	pr_err("Macle ant_div_probe\n");

	if (pdev->dev.of_node) {
		ant_div_info = kzalloc(sizeof(struct ant_div_check_info), GFP_KERNEL);
		if (!ant_div_info) {
			pr_err("Macle %s: failed to alloc memory for module data\n",__func__);
			return -ENOMEM;
		}
		err = ant_div_parse_dt(&pdev->dev, ant_div_info);
		if (err) {
			dev_err(&pdev->dev, "Macle ant_div_probe DT parsing failed\n");
			goto free_struct;
		}
	} else{
		return -ENOMEM;
	}
	mutex_init(&ant_div_info->io_lock);

	platform_set_drvdata(pdev, ant_div_info);

#ifdef CONFIG_ANT_DIV_IRQ
/*input system config*/
	ant_div_info->ipdev = input_allocate_device();
	if (!ant_div_info->ipdev) {
		pr_err("ant_div_probe: input_allocate_device fail\n");
		goto input_error;
	}
	ant_div_info->ipdev->name = "ant_div_check-input";
	input_set_capability(ant_div_info->ipdev, EV_KEY, DIV_ANT_CONNECT);
	input_set_capability(ant_div_info->ipdev, EV_KEY, DIV_ANT_UNCONNECT);
	//set_bit(INPUT_PROP_NO_DUMMY_RELEASE, ant_div_info->ipdev->propbit);
	rc = input_register_device(ant_div_info->ipdev);
	if (rc) {
		pr_err("ant_div_probe: input_register_device fail rc=%d\n", rc);
		goto input_error;
	}

//	ant_power_on(&pdev->dev);

/*interrupt config*/
	if (gpio_is_valid(ant_div_info->irq_gpio)) {
		rc = gpio_request(ant_div_info->irq_gpio, "ant_check_div");
		if (rc < 0) {
		        pr_err("ant_div_probe: gpio_request fail rc=%d\n", rc);
		        goto free_input_device;
		}

		rc = gpio_direction_input(ant_div_info->irq_gpio);
		if (rc < 0) {
		        pr_err("ant_div_probe: gpio_direction_input fail rc=%d\n", rc);
		        goto err_irq;
		}
		ant_div_info->ant_div_check_state = gpio_get_value(ant_div_info->irq_gpio);
		pr_err("ant_div_probe: gpios = %d, gpion=%d\n", ant_div_info->ant_div_check_state, ant_div_info->ant_div_check_state);

		ant_div_info->irq = gpio_to_irq(ant_div_info->irq_gpio);
		pr_err("Macle irq = %d\n", ant_div_info->irq);
		rc = devm_request_threaded_irq(&pdev->dev, ant_div_info->irq, NULL,
			ant_div_interrupt,
			IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_ONESHOT, "ant-div-switch-irq", ant_div_info);
		if (rc < 0) {
			pr_err("ant_div_probe: request_irq fail rc=%d\n", rc);
			goto err_irq;
		}

		device_init_wakeup(&pdev->dev, true);
		irq_set_irq_wake(ant_div_info->irq,1);
	}else{
		pr_err("Macle irq gpio not provided\n");
	        goto free_input_device;
	}
#endif

       pr_err("ant_div_probe end\n");
#ifdef CONFIG_ANT_DIV_SYS
	ant_div_register_class_dev(ant_div_info);
#endif
	   global_ant_div_info = ant_div_info;
       return 0;

#ifdef CONFIG_ANT_DIV_IRQ
err_irq:
	disable_irq_wake(ant_div_info->irq);
	device_init_wakeup(&pdev->dev, 0);
	gpio_free(ant_div_info->irq_gpio);

free_input_device:
	input_unregister_device(ant_div_info->ipdev);
input_error:
	platform_set_drvdata(pdev, NULL);
#endif

free_struct:
	kfree(ant_div_info);

	return rc;
}

static int ant_div_remove(struct platform_device *pdev)
{
	struct ant_div_check_info *ant = platform_get_drvdata(pdev);
#ifdef CONFIG_ANT_DIV_SYS
	class_destroy(ant->ant_div_sys_class);
#endif
	pr_err("ant_remove\n");

#ifdef CONFIG_ANT_DIV_IRQ
	disable_irq_wake(ant->irq);
	device_init_wakeup(&pdev->dev, 0);
	free_irq(ant->irq, ant->ipdev);
#endif
	gpio_free(ant->irq_gpio);
#ifdef CONFIG_ANT_DIV_IRQ
	input_unregister_device(ant->ipdev);
#endif
	return 0;
}



static struct of_device_id sn_match_table[] = {
	{ .compatible = "ant_check_div", },
	{ },
};

static struct platform_driver ant_div_driver = {
	.probe                = ant_div_probe,
	.remove               = ant_div_remove,
	.driver               = {
		.name           = "ant_check_div",
		.owner          = THIS_MODULE,
		.of_match_table = sn_match_table,
	},
};

static int __init ant_div_init(void)
{
	pr_err("ant_div_init\n");
	return platform_driver_register(&ant_div_driver);
}

static void __exit ant_div_exit(void)
{
	platform_driver_unregister(&ant_div_driver);
}

module_init(ant_div_init);
module_exit(ant_div_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("lisuyang");
