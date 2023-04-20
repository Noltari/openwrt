// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright (C) 2020 Álvaro Fernández Rojas <noltari@gmail.com>
 */

#include <linux/pci.h>
#include <linux/pci_ids.h>

#define PCIE_BUS_DEVICE	1

int bmips_pci_irq = -1;
int bmips_wlan_irq = -1;

int pcibios_plat_dev_init(struct pci_dev *pci_dev)
{
	return PCIBIOS_SUCCESSFUL;
}

int pcibios_map_irq(const struct pci_dev *pci_dev, u8 slot, u8 pin)
{
	if ((pci_dev->bus->number == PCIE_BUS_DEVICE) &&
	    ((pci_dev->class >> 8) != PCI_CLASS_BRIDGE_PCI))
		return bmips_pci_irq;
	return bmips_wlan_irq;
}
