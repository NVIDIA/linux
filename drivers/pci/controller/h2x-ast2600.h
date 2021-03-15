#ifndef __H2X_ASPEED_H_INCLUDED
#define __H2X_ASPEED_H_INCLUDED

#include <linux/reset.h>
#include <linux/irqdomain.h>
#include <linux/aspeed_pcie_io.h>
#include <linux/msi.h>

struct aspeed_pcie {
	struct device *dev;

	void __iomem *pciereg_base;
	void __iomem *h2x_rc_base;
	u32 irq;
	u32 rc_offset;
	u32 msi_address;

	struct reset_control *reset;
	/* PCIe operations */
	struct pci_ops aspeed_pcie_ops;

	struct resource mem;
	unsigned int busnr;

	/* INTx IRQ Domain operations */
	struct irq_domain *leg_domain;
	struct irq_chip aspeed_h2x_intx_chip;
	struct irq_domain_ops intx_domain_ops;

	// msi
	struct irq_domain *msi_domain;
	struct msi_controller aspeed_pcie_msi_chip;
	struct irq_chip aspeed_msi_irq_chip;
	struct irq_domain_ops msi_domain_ops;
};

extern void aspeed_h2x_intx_ack_irq(struct irq_data *d);
extern void aspeed_h2x_intx_mask_irq(struct irq_data *d);
extern void aspeed_h2x_intx_unmask_irq(struct irq_data *d);
extern void aspeed_h2x_msi_enable(struct aspeed_pcie *pcie);
extern void aspeed_h2x_rc_intr_handler(struct aspeed_pcie *pcie);

extern int aspeed_h2x_rd_conf(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *val);

extern int aspeed_h2x_wr_conf(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 val);

extern void aspeed_h2x_workaround(struct aspeed_pcie *pcie);

extern void aspeed_h2x_rc_init(struct aspeed_pcie *pcie);

#endif
