#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <mach/pm.h>
#include <asm/icc.h>
#include <asm/irq_handler.h>

void platform_send_ipi_cpu(unsigned int cpu, int irq)
{
	BUG_ON(cpu >= 2);
	SSYNC();
	bfin_sec_raise_irq(irq);
	SSYNC();
}

void platform_send_ipi(cpumask_t callmap, int irq)
{
	unsigned int cpu;

	for_each_cpu_mask(cpu, callmap) {
		BUG_ON(cpu >= 2);
		SSYNC();
		platform_send_ipi_cpu(cpu, irq);
		SSYNC();
	}
}

void platform_clear_ipi(unsigned int cpu, int irq)
{
	BUG_ON(cpu >= 2);
}

static irqreturn_t coreb_resource_manage_dummy(int irq, void *dev_id)
{
	return 1;
}

int platform_res_manage_request_irq(uint16_t subid, unsigned int cpu)
{
	int i, n;
	int ret;
	unsigned int bank, bank_base;

	if (((subid - IRQ_PINT0) >= 0) && ((subid - IRQ_PINT0) < NR_PINT_SYS_IRQS)) {
		bank = subid - IRQ_PINT0;
		bank_base = GPIO_IRQ_BASE + (bank << 4);
		for (i = 0; i < GPIO_BANKSIZE; i++) {
			ret = request_irq(bank_base + i,
				coreb_resource_manage_dummy,
					IRQ_TYPE_EDGE_BOTH, "coreb dummy", NULL);
			if (ret) {
				n = i;
				for (i = 0; i < n; i++)
					free_irq(bank_base + i, NULL);
				return ret;
			}
		}
		icc_irq_set_affinity(IRQ_PINT0 + bank, cpumask_of(cpu));
	} else {
		ret = request_irq(subid, coreb_resource_manage_dummy,
				IRQ_TYPE_EDGE_BOTH, "coreb dummy", NULL);
		if (ret) {
			printk(KERN_DEBUG "request irq %d failed\n", subid);
			return ret;
		}
		icc_irq_set_affinity(subid, cpumask_of(cpu));
	}

	return 0;
}

void platform_res_manage_free_irq(uint16_t subid)
{
	int i;
	unsigned int bank, bank_base;

	if (((subid - IRQ_PINT0) >= 0) && ((subid - IRQ_PINT0) < NR_PINT_SYS_IRQS)) {
		bank = subid - IRQ_PINT0;
		bank_base = GPIO_IRQ_BASE + (bank << 4);
		for (i = 0; i < GPIO_BANKSIZE; i++)
			free_irq(bank_base + i, NULL);
		icc_irq_set_affinity(IRQ_PINT0 + bank, cpumask_of(0));
	} else {
		free_irq(subid, NULL);
		icc_irq_set_affinity(subid, cpumask_of(0));
	}
}
