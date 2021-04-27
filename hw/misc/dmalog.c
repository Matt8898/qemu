#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/pci/pci.h"
#include "hw/hw.h"
#include "hw/pci/msi.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "qapi/visitor.h"
#include "chardev/char.h"
#include "chardev/char-fe.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"

#define TYPE_PCI_DMALOG_DEVICE "dmalog"
#define DMALOG(obj)        OBJECT_CHECK(DmalogState, obj, TYPE_PCI_DMALOG_DEVICE)

#define DMA_IRQ         0x00000100

#define DMA_START       0x40000
#define DMA_SIZE        16384

struct descriptor {
    unsigned int flags; // If bit 1 is set, send MSI on completion.
    unsigned int status; // Initialized to 0 by the guest; host writes 1 on completion.
    uint64_t payload; // Ptr to buffer.
    uint64_t size; // Size of buffer.
    uint64_t msi_address;
    uint64_t msi_data;
};

typedef struct {
    PCIDevice pdev;
    MemoryRegion mmio;

	uint64_t region_base;
	size_t   region_size;

    uint64_t in_descriptor_addr;
    struct descriptor in_descriptor;
    size_t cur;
    bool in_valid;

    uint32_t irq_status;

    CharBackend chr;
    Chardev* chrd;

    char dma_buf[DMA_SIZE];
    uint64_t dma_mask;
    char* tag;
    size_t taglen;
} DmalogState;

static uint64_t dmalog_mmio_read(void *opaque, hwaddr addr, unsigned size) {
    DmalogState *dmalog = opaque;

    if ((addr >= 0x10) && (addr <= (0x10 + 64))) {
        size_t idx = addr - 0x10;
        return (idx < dmalog->taglen) ? dmalog->tag[idx] : 0;
    }
    return 0;
}


static void dmalog_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                unsigned size)
{
    DmalogState *dmalog = opaque;
    switch (addr) {
        case 0x0 :;
            struct descriptor descr = {0};
		    dma_memory_read(&address_space_memory, val, &descr, sizeof(struct descriptor));
		    dma_memory_read(&address_space_memory, val, &descr, sizeof(struct descriptor));
            
            size_t size = descr.size;

            while (size) {
                size_t to_read = (size < DMA_SIZE) ? size : DMA_SIZE;
		        dma_memory_read(&address_space_memory, descr.payload, dmalog->dma_buf, to_read);
                qemu_chr_fe_write_all(&dmalog->chr, (const uint8_t *)dmalog->dma_buf, to_read);
                size -= to_read;
            }
            if (descr.flags) {
                msi_notify(&dmalog->pdev, 0);
            }
            break;
        case 0x8:
            dma_memory_read(&address_space_memory, val, &dmalog->in_descriptor, sizeof(struct descriptor));
            dmalog->cur = 0;
            dmalog->in_descriptor_addr = val;
            dmalog->in_valid = true;
    }

}

static const MemoryRegionOps dmalog_mmio_ops = {
    .write = dmalog_mmio_write,
    .read = dmalog_mmio_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 8,
    },

};

static Property dmalog_properties[] = {
    DEFINE_PROP_CHR("chardev", DmalogState, chr),
    DEFINE_PROP_STRING("tag", DmalogState, tag),
    DEFINE_PROP_END_OF_LIST(),
};

void dmalog_handle_read(void *opaque, const uint8_t *buf, int size);
void dmalog_handle_read(void *opaque, const uint8_t *buf, int size) {
    DmalogState* dmalog = (DmalogState*)opaque;

    if (!dmalog->in_valid) {
        return;
    }

    if (dmalog->in_descriptor.status) {
        return;
    }

    size_t limit = dmalog->in_descriptor.size;

    size_t to_copy = ((dmalog->cur + size) > limit) ? size : (limit - dmalog->cur);

	dma_memory_write(&address_space_memory, dmalog->in_descriptor.payload + dmalog->cur, buf, to_copy);

    dmalog->cur += to_copy;

    if (dmalog->cur == limit || buf[size - 1] == '\n') {
        dmalog->in_descriptor.status = 1;
	    dma_memory_write(&address_space_memory, dmalog->in_descriptor_addr, &dmalog->in_descriptor, sizeof(struct descriptor));
        if (dmalog->in_descriptor.flags) {
            msi_notify(&dmalog->pdev, 0);
        }
    }
}

static int dmalog_can_recv(void *opaque) {
    DmalogState* dmalog = (DmalogState*)opaque;

    if (!dmalog->in_valid) {
        return 0;
    }

    if (dmalog->in_descriptor.status) {
        return 0;
    }

    size_t limit = dmalog->in_descriptor.size;

    return (limit - dmalog->cur);
}

static void pci_dmalog_realize(PCIDevice *pdev, Error **errp)
{
    DmalogState *dmalog = DMALOG(pdev);
    uint8_t *pci_conf = pdev->config;

    pci_config_set_interrupt_pin(pci_conf, 1);

    if (msi_init(pdev, 0, 1, true, false, errp)) {
        return;
    }
    qemu_chr_fe_set_open(&dmalog->chr, true);
    qemu_chr_fe_set_handlers(&dmalog->chr, &dmalog_can_recv, &dmalog_handle_read, NULL, NULL, dmalog, NULL, true);

    dmalog->taglen = strlen(dmalog->tag);

    memory_region_init_io(&dmalog->mmio, OBJECT(dmalog), &dmalog_mmio_ops, dmalog,
                    "dmalog-mmio", 1 * MiB);
    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &dmalog->mmio);
}

static void pci_dmalog_uninit(PCIDevice *pdev)
{
    //msi_uninit(pdev);
}

static void dmalog_instance_init(Object *obj)
{
    DmalogState *dmalog = DMALOG(obj);
    dmalog->in_valid = false;

    dmalog->dma_mask = (1UL << 28) - 1;
    object_property_add_uint64_ptr(obj, "dma_mask",
                                   &dmalog->dma_mask, OBJ_PROP_FLAG_READWRITE);
}

static void dmalog_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_dmalog_realize;
    k->exit = pci_dmalog_uninit;
    k->vendor_id = PCI_VENDOR_ID_QEMU;
    k->device_id = 0x69e8;
    k->revision = 0x12;
    k->class_id = 0x50;
    device_class_set_props(dc, dmalog_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static InterfaceInfo interfaces[] = {
    { INTERFACE_CONVENTIONAL_PCI_DEVICE },
    { },
};

static const TypeInfo dmalog_info = {
    .name          = TYPE_PCI_DMALOG_DEVICE,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(DmalogState),
    .instance_init = dmalog_instance_init,
    .class_init    = dmalog_class_init,
    .interfaces = interfaces,
};

static void pci_dmalog_register_types(void)
{
    type_register_static(&dmalog_info);
}
type_init(pci_dmalog_register_types)
