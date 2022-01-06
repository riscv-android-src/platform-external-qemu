/*
 * QEMU RISC-V Ranchu Board
 *
 * Copyright (C) 2021 Alibaba Group Holding Limited
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/char/serial.h"
#include "target/riscv/cpu.h"
#include "hw/riscv/riscv_htif.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/sifive_plic.h"
#include "hw/riscv/sifive_clint.h"
#include "hw/riscv/sifive_test.h"
#include "hw/riscv/riscv_ranchu.h"
#include "chardev/char.h"
#include "sysemu/arch_init.h"
#include "sysemu/device_tree.h"
#include "exec/address-spaces.h"
#include "elf.h"
#include "sysemu/ranchu.h"
#include "hw/pci/pci.h"
#include "hw/pci-host/gpex.h"

enum MemoryType{
    RANCHU_CLINT = 0,
    RANCHU_PLIC,
    RANCHU_UART0,
    RANCHU_VIRTIO,
    RANCHU_GOLDFISH_FB,
    RANCHU_GOLDFISH_BATTERY,
    RANCHU_GOLDFISH_AUDIO,
    RANCHU_GOLDFISH_EVDEV,
    RANCHU_GOLDFISH_PIPE,
    RANCHU_GOLDFISH_SYNC,
    RANCHU_PCIE_MMIO,
    RANCHU_PCIE_PIO,
    RANCHU_PCIE_ECAM,
    RANCHU_DRAM,
};

static const struct MemmapEntry {
    hwaddr base;
    hwaddr size;
} ranchu_memmap[] = {
    [RANCHU_CLINT]          =  {  0x2000000,    0x10000 },
    [RANCHU_PLIC]           =  {  0xc000000,  0x4000000 },
    [RANCHU_PCIE_PIO]       =  {  0x3000000,    0x10000 },
    [RANCHU_UART0]          =  { 0x10000000,      0x100 },
    [RANCHU_GOLDFISH_FB]    =  { 0x10002000,     0x1000 },
    [RANCHU_GOLDFISH_AUDIO] =  { 0x10003000,     0x1000 },
    [RANCHU_GOLDFISH_EVDEV] =  { 0x10004000,     0x1000 },
    [RANCHU_GOLDFISH_PIPE]  =  { 0x10005000,     0x1000 },
    [RANCHU_GOLDFISH_SYNC]  =  { 0x10006000,     0x1000 },
    [RANCHU_GOLDFISH_BATTERY]  =  { 0x10007000,  0x1000 },
    [RANCHU_VIRTIO]         =  { 0x20000000,      0x200 },
    [RANCHU_PCIE_ECAM]      =  { 0x30000000, 0x10000000 },
    [RANCHU_PCIE_MMIO]      =  { 0x40000000, 0x40000000 },
    [RANCHU_DRAM]           =  { 0x80000000,        0x0 },
};

/* PCIe high mmio for RV64, size is fixed but base depends on top of RAM */
#define GiB     (INT64_C(1) << 30)
#define VIRT64_HIGH_PCIE_MMIO_SIZE  (16 * GiB)

static struct MemmapEntry virt_high_pcie_memmap;

static void copy_le32_to_phys(hwaddr pa, uint32_t *rom, size_t len)
{
    int i;
    for (i = 0; i < (len >> 2); i++) {
        stl_phys(&address_space_memory, pa + (i << 2), rom[i]);
    }
}

# define KERNEL_BOOT_ADDRESS 0x80200000
static uint64_t identity_translate(void *opaque, uint64_t addr)
{
    //return addr;
    return addr+KERNEL_BOOT_ADDRESS;
}

target_ulong riscv_load_kernel(const char *kernel_filename, symbol_fn_t sym_cb)
{
    uint64_t kernel_entry, kernel_low, kernel_high;

    if (load_elf_ram_sym(kernel_filename, identity_translate, NULL,
                &kernel_entry, &kernel_low, &kernel_high, 0,
                EM_RISCV, 1, 0, NULL, true, sym_cb) > 0) {
        return kernel_low;
    }

    if (load_uimage_as(kernel_filename, &kernel_entry, NULL, NULL,
                NULL, NULL, NULL) > 0) {
        return kernel_entry;
    }

    if (load_image_targphys_as(kernel_filename, KERNEL_BOOT_ADDRESS,
                ram_size, NULL) > 0) {
        return KERNEL_BOOT_ADDRESS;
    }

    error_report("could not load kernel '%s'", kernel_filename);
    exit(1);
}

static hwaddr load_initrd(const char *filename, uint64_t mem_size,
                          uint64_t kernel_entry, hwaddr *start)
{
    int size;

    /* We want to put the initrd far enough into RAM that when the
     * kernel is uncompressed it will not clobber the initrd. However
     * on boards without much RAM we must ensure that we still leave
     * enough room for a decent sized initrd, and on boards with large
     * amounts of RAM we must avoid the initrd being so far up in RAM
     * that it is outside lowmem and inaccessible to the kernel.
     * So for boards with less  than 256MB of RAM we put the initrd
     * halfway into RAM, and for boards with 256MB of RAM or more we put
     * the initrd at 128MB.
     */
    *start = kernel_entry + MIN(mem_size / 2, 128 * 1024 * 1024);

    size = load_ramdisk(filename, *start, mem_size - *start);
    if (size == -1) {
        size = load_image_targphys(filename, *start, mem_size - *start);
        if (size == -1) {
            error_report("qemu: could not load ramdisk '%s'", filename);
            exit(1);
        }
    }
    return *start + size;
}

target_ulong riscv_load_firmware(const char *firmware_filename,
        hwaddr firmware_load_addr,
        symbol_fn_t sym_cb)
{
    uint64_t firmware_entry, firmware_start, firmware_end;

    if (load_elf_ram_sym(firmware_filename, NULL, NULL,
                &firmware_entry, &firmware_start, &firmware_end,
                0, EM_RISCV, 1, 0, NULL, true, sym_cb) > 0) {
        return firmware_entry;
    }

    if (load_image_targphys_as(firmware_filename, firmware_load_addr,
                ram_size, NULL) > 0) {
        return firmware_load_addr;
    }

    error_report("could not load firmware '%s'", firmware_filename);
    exit(1);
}

static QemuDeviceTreeSetupFunc device_tree_setup_func = NULL;
void qemu_device_tree_setup_callback(QemuDeviceTreeSetupFunc setup_func)
{
    device_tree_setup_func = setup_func;
}

static void create_device(RISCVVirtState *s, void *fdt, enum MemoryType type)
{
    char * nodename = NULL;
    char * compat = NULL;
    char * dev_name = NULL;
    int irq = -1;

    switch (type) {
        case RANCHU_GOLDFISH_FB:
            nodename = g_strdup_printf("/goldfish_fb@%" PRIx64, ranchu_memmap[type].base);
            compat = g_strdup_printf("google,goldfish-fb");
            dev_name = "goldfish_fb";
            irq = RANCHU_GOLDFISH_FB_IRQ;
            break;
        case RANCHU_GOLDFISH_AUDIO:
            dev_name = "goldfish_audio";
            nodename = g_strdup_printf("/goldfish_audio@%" PRIx64, ranchu_memmap[type].base);
            compat = g_strdup_printf("google,goldfish-audio");
            irq = RANCHU_GOLDFISH_AUDIO_IRQ;
            break;
        case RANCHU_GOLDFISH_BATTERY:
            dev_name = "goldfish_battery";
            nodename = g_strdup_printf("/goldfish_battery@%" PRIx64, ranchu_memmap[type].base);
            compat = g_strdup_printf("google,goldfish-battery");
            irq = RANCHU_GOLDFISH_BATTERY_IRQ;
            break;

        case RANCHU_GOLDFISH_SYNC:
            dev_name = "goldfish_sync";
            nodename = g_strdup_printf("/goldfish_sync@%" PRIx64, ranchu_memmap[type].base);
            compat = g_strdup_printf("google,goldfish-sync");
            irq = RANCHU_GOLDFISH_SYNC_IRQ;
            break;
        case RANCHU_GOLDFISH_PIPE:
            dev_name = "goldfish_pipe";
            nodename = g_strdup_printf("/goldfish_pipe@%" PRIx64, ranchu_memmap[type].base);
            compat = g_strdup_printf("google,android-pipe");
            irq = RANCHU_GOLDFISH_PIPE_IRQ;
            break;
        case RANCHU_GOLDFISH_EVDEV:
            dev_name = "goldfish-events";
            nodename = g_strdup_printf("/goldfish-events-keypad@%" PRIx64, ranchu_memmap[type].base);
            compat = g_strdup_printf("google,goldfish-events-keypad");
            irq = RANCHU_GOLDFISH_EVDEV_IRQ;
            break;
        default:
            return;
    }

    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", compat);
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, ranchu_memmap[type].base,
        0x0, ranchu_memmap[type].size);

    //qemu_fdt_setprop_cell(fdt, nodename, "clock-frequency", 3686400);

    char *plic_nodename = g_strdup_printf("/soc/interrupt-controller@%lx", (long)ranchu_memmap[RANCHU_PLIC].base);
    int plic_phandle = qemu_fdt_get_phandle(fdt, plic_nodename);

    if (irq != -1) {
        qemu_fdt_setprop_cells(fdt, nodename, "interrupt-parent", plic_phandle);
        qemu_fdt_setprop_cells(fdt, nodename, "interrupts", irq);
    }
    sysbus_create_simple(dev_name, ranchu_memmap[type].base, SIFIVE_PLIC(s->plic)->irqs[irq]);

    g_free(plic_nodename);
    g_free(nodename);
    g_free(compat);
}

static void create_pcie_irq_map(void *fdt, char *nodename,
                                uint32_t plic_phandle)
{
    int pin, dev;
    uint32_t
        full_irq_map[GPEX_NUM_IRQS * GPEX_NUM_IRQS * FDT_INT_MAP_WIDTH] = {};
    uint32_t *irq_map = full_irq_map;

    /* This code creates a standard swizzle of interrupts such that
     * each device's first interrupt is based on it's PCI_SLOT number.
     * (See pci_swizzle_map_irq_fn())
     *
     * We only need one entry per interrupt in the table (not one per
     * possible slot) seeing the interrupt-map-mask will allow the table
     * to wrap to any number of devices.
     */
    for (dev = 0; dev < GPEX_NUM_IRQS; dev++) {
        int devfn = dev * 0x8;

        for (pin = 0; pin < GPEX_NUM_IRQS; pin++) {
            int irq_nr = PCIE_IRQ + ((pin + PCI_SLOT(devfn)) % GPEX_NUM_IRQS);
            int i = 0;

            irq_map[i] = cpu_to_be32(devfn << 8);

            i += FDT_PCI_ADDR_CELLS;
            irq_map[i] = cpu_to_be32(pin + 1);

            i += FDT_PCI_INT_CELLS;
            irq_map[i++] = cpu_to_be32(plic_phandle);

            i += FDT_PLIC_ADDR_CELLS;
            irq_map[i] = cpu_to_be32(irq_nr);

            irq_map += FDT_INT_MAP_WIDTH;
        }
    }
    qemu_fdt_setprop(fdt, nodename, "interrupt-map",
                     full_irq_map, sizeof(full_irq_map));

    qemu_fdt_setprop_cells(fdt, nodename, "interrupt-map-mask",
                           0x1800, 0, 0, 0x7);
}

static void *create_fdt(RISCVVirtState *s, const struct MemmapEntry *memmap,
    uint64_t mem_size, const char *cmdline)
{
    void *fdt;
    int cpu;
    uint32_t *cells;
    char *nodename;
    uint32_t plic_phandle, phandle = 1;
    int i;

    fdt = s->fdt = create_device_tree(&s->fdt_size);
    if (!fdt) {
        error_report("create_device_tree() failed");
        exit(1);
    }

    qemu_fdt_setprop_string(fdt, "/", "model", "riscv-virtio,qemu");
    qemu_fdt_setprop_string(fdt, "/", "compatible", "riscv-virtio");
    qemu_fdt_setprop_cell(fdt, "/", "#size-cells", 0x2);
    qemu_fdt_setprop_cell(fdt, "/", "#address-cells", 0x2);

    qemu_fdt_add_subnode(fdt, "/firmware");
    qemu_fdt_add_subnode(fdt, "/firmware/android");
    qemu_fdt_setprop_string(fdt, "/firmware/android", "compatible", "android,firmware");
    qemu_fdt_setprop_string(fdt, "/firmware/android", "hardware", "ranchu");

    qemu_fdt_add_subnode(fdt, "/soc");
    qemu_fdt_setprop(fdt, "/soc", "ranges", NULL, 0);
    qemu_fdt_setprop_string(fdt, "/soc", "compatible", "riscv-virtio-soc");
    qemu_fdt_setprop_cell(fdt, "/soc", "#size-cells", 0x2);
    qemu_fdt_setprop_cell(fdt, "/soc", "#address-cells", 0x2);

    nodename = g_strdup_printf("/memory@%lx",
        (long)memmap[RANCHU_DRAM].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        memmap[RANCHU_DRAM].base >> 32, memmap[RANCHU_DRAM].base,
        mem_size >> 32, mem_size);
    qemu_fdt_setprop_string(fdt, nodename, "device_type", "memory");
    g_free(nodename);

    qemu_fdt_add_subnode(fdt, "/cpus");
    qemu_fdt_setprop_cell(fdt, "/cpus", "timebase-frequency", 10000000);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#size-cells", 0x0);
    qemu_fdt_setprop_cell(fdt, "/cpus", "#address-cells", 0x1);

    for (cpu = s->soc.num_harts - 1; cpu >= 0; cpu--) {
        int cpu_phandle = phandle++;
        nodename = g_strdup_printf("/cpus/cpu@%d", cpu);
        char *intc = g_strdup_printf("/cpus/cpu@%d/interrupt-controller", cpu);
        char *isa = riscv_isa_string(&s->soc.harts[cpu]);
        qemu_fdt_add_subnode(fdt, nodename);
        qemu_fdt_setprop_cell(fdt, nodename, "clock-frequency", 1000000000);
        qemu_fdt_setprop_string(fdt, nodename, "mmu-type", "riscv,sv48");
        qemu_fdt_setprop_string(fdt, nodename, "riscv,isa", isa);
        qemu_fdt_setprop_string(fdt, nodename, "compatible", "riscv");
        qemu_fdt_setprop_string(fdt, nodename, "status", "okay");
        qemu_fdt_setprop_cell(fdt, nodename, "reg", cpu);
        qemu_fdt_setprop_string(fdt, nodename, "device_type", "cpu");
        qemu_fdt_add_subnode(fdt, intc);
        qemu_fdt_setprop_cell(fdt, intc, "phandle", cpu_phandle);
        qemu_fdt_setprop_cell(fdt, intc, "linux,phandle", cpu_phandle);
        qemu_fdt_setprop_string(fdt, intc, "compatible", "riscv,cpu-intc");
        qemu_fdt_setprop(fdt, intc, "interrupt-controller", NULL, 0);
        qemu_fdt_setprop_cell(fdt, intc, "#interrupt-cells", 1);
        g_free(isa);
        g_free(intc);
        g_free(nodename);
    }

    cells =  g_new0(uint32_t, s->soc.num_harts * 4);
    for (cpu = 0; cpu < s->soc.num_harts; cpu++) {
        nodename =
            g_strdup_printf("/cpus/cpu@%d/interrupt-controller", cpu);
        uint32_t intc_phandle = qemu_fdt_get_phandle(fdt, nodename);
        cells[cpu * 4 + 0] = cpu_to_be32(intc_phandle);
        cells[cpu * 4 + 1] = cpu_to_be32(IRQ_M_SOFT);
        cells[cpu * 4 + 2] = cpu_to_be32(intc_phandle);
        cells[cpu * 4 + 3] = cpu_to_be32(IRQ_M_TIMER);
        g_free(nodename);
    }
    nodename = g_strdup_printf("/soc/clint@%lx",
        (long)memmap[RANCHU_CLINT].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "riscv,clint0");
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[RANCHU_CLINT].base,
        0x0, memmap[RANCHU_CLINT].size);
    qemu_fdt_setprop(fdt, nodename, "interrupts-extended",
        cells, s->soc.num_harts * sizeof(uint32_t) * 4);
    g_free(cells);
    g_free(nodename);

    plic_phandle = phandle++;
    cells =  g_new0(uint32_t, s->soc.num_harts * 4);
    for (cpu = 0; cpu < s->soc.num_harts; cpu++) {
        nodename =
            g_strdup_printf("/cpus/cpu@%d/interrupt-controller", cpu);
        uint32_t intc_phandle = qemu_fdt_get_phandle(fdt, nodename);
        cells[cpu * 4 + 0] = cpu_to_be32(intc_phandle);
        cells[cpu * 4 + 1] = cpu_to_be32(IRQ_M_EXT);
        cells[cpu * 4 + 2] = cpu_to_be32(intc_phandle);
        cells[cpu * 4 + 3] = cpu_to_be32(IRQ_S_EXT);
        g_free(nodename);
    }
    nodename = g_strdup_printf("/soc/interrupt-controller@%lx",
        (long)memmap[RANCHU_PLIC].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cell(fdt, nodename,
        "#address-cells", FDT_PLIC_ADDR_CELLS);
    qemu_fdt_setprop_cell(fdt, nodename,
        "#interrupt-cells", FDT_PLIC_INT_CELLS);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "riscv,plic0");
    qemu_fdt_setprop(fdt, nodename, "interrupt-controller", NULL, 0);
    qemu_fdt_setprop(fdt, nodename, "interrupts-extended",
        cells, s->soc.num_harts * sizeof(uint32_t) * 4);
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[RANCHU_PLIC].base,
        0x0, memmap[RANCHU_PLIC].size);
    qemu_fdt_setprop_string(fdt, nodename, "reg-names", "control");
    qemu_fdt_setprop_cell(fdt, nodename, "riscv,max-priority", 7);
    qemu_fdt_setprop_cell(fdt, nodename, "riscv,ndev", VIRTIO_NDEV);
    qemu_fdt_setprop_cells(fdt, nodename, "phandle", plic_phandle);
    qemu_fdt_setprop_cells(fdt, nodename, "linux,phandle", plic_phandle);
    plic_phandle = qemu_fdt_get_phandle(fdt, nodename);
    g_free(cells);
    g_free(nodename);

    for (i = 0; i < VIRTIO_COUNT; i++) {
        nodename = g_strdup_printf("/virtio_mmio@%lx",
            (long)(memmap[RANCHU_VIRTIO].base + i * memmap[RANCHU_VIRTIO].size));
        qemu_fdt_add_subnode(fdt, nodename);
        qemu_fdt_setprop_string(fdt, nodename, "compatible", "virtio,mmio");
        qemu_fdt_setprop_sized_cells(fdt, nodename, "reg",
            0x2, memmap[RANCHU_VIRTIO].base + i * memmap[RANCHU_VIRTIO].size,
            0x2, memmap[RANCHU_VIRTIO].size);
        qemu_fdt_setprop_cells(fdt, nodename, "interrupt-parent", plic_phandle);
        qemu_fdt_setprop_cells(fdt, nodename, "interrupts", VIRTIO_IRQ + i);
        g_free(nodename);
    }

    /* init pci fdt */
    nodename = g_strdup_printf("/pci@%lx",
        (long) memmap[RANCHU_PCIE_ECAM].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_cell(fdt, nodename, "#address-cells",
        FDT_PCI_ADDR_CELLS);
    qemu_fdt_setprop_cell(fdt, nodename, "#interrupt-cells",
        FDT_PCI_INT_CELLS);
    qemu_fdt_setprop_cell(fdt, nodename, "#size-cells", 0x2);
    qemu_fdt_setprop_string(fdt, nodename, "compatible",
        "pci-host-ecam-generic");
    qemu_fdt_setprop_string(fdt, nodename, "device_type", "pci");
    qemu_fdt_setprop_cell(fdt, nodename, "linux,pci-domain", 0);
    qemu_fdt_setprop_cells(fdt, nodename, "bus-range", 0,
        memmap[RANCHU_PCIE_ECAM].size / PCIE_MMCFG_SIZE_MIN - 1);
    qemu_fdt_setprop(fdt, nodename, "dma-coherent", NULL, 0);
    qemu_fdt_setprop_cells(fdt, nodename, "reg", 0,
        memmap[RANCHU_PCIE_ECAM].base, 0, memmap[RANCHU_PCIE_ECAM].size);
    qemu_fdt_setprop_sized_cells(fdt, nodename, "ranges",
        1, FDT_PCI_RANGE_IOPORT, 2, 0,
        2, memmap[RANCHU_PCIE_PIO].base, 2, memmap[RANCHU_PCIE_PIO].size,
        1, FDT_PCI_RANGE_MMIO,
        2, memmap[RANCHU_PCIE_MMIO].base,
        2, memmap[RANCHU_PCIE_MMIO].base, 2, memmap[RANCHU_PCIE_MMIO].size,
        1, FDT_PCI_RANGE_MMIO_64BIT,
        2, virt_high_pcie_memmap.base,
        2, virt_high_pcie_memmap.base, 2, virt_high_pcie_memmap.size);

    create_pcie_irq_map(fdt, nodename, plic_phandle);
    g_free(nodename);

    nodename = g_strdup_printf("/uart@%lx",
        (long)memmap[RANCHU_UART0].base);
    qemu_fdt_add_subnode(fdt, nodename);
    qemu_fdt_setprop_string(fdt, nodename, "compatible", "ns16550a");
    qemu_fdt_setprop_cells(fdt, nodename, "reg",
        0x0, memmap[RANCHU_UART0].base,
        0x0, memmap[RANCHU_UART0].size);
    qemu_fdt_setprop_cell(fdt, nodename, "clock-frequency", 3686400);
        qemu_fdt_setprop_cells(fdt, nodename, "interrupt-parent", plic_phandle);
        qemu_fdt_setprop_cells(fdt, nodename, "interrupts", UART0_IRQ);

    qemu_fdt_add_subnode(fdt, "/chosen");
    qemu_fdt_setprop_string(fdt, "/chosen", "stdout-path", nodename);
    qemu_fdt_setprop_string(fdt, "/chosen", "bootargs", cmdline);
    g_free(nodename);


    return fdt;
}

static inline DeviceState *gpex_pcie_init(RISCVVirtState *s, MemoryRegion *sys_mem,
                                          hwaddr ecam_base, hwaddr ecam_size,
                                          hwaddr mmio_base, hwaddr mmio_size,
                                          hwaddr high_mmio_base,
                                          hwaddr high_mmio_size,
                                          hwaddr pio_base,
                                          DeviceState *plic)
{
    DeviceState *dev;
    MemoryRegion *ecam_alias, *ecam_reg;
    MemoryRegion *mmio_alias, *high_mmio_alias, *mmio_reg;
    qemu_irq irq;
    int i;

    dev = qdev_create(NULL, TYPE_GPEX_HOST);
    qdev_init_nofail(dev);

    ecam_alias = g_new0(MemoryRegion, 1);
    ecam_reg = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_init_alias(ecam_alias, OBJECT(dev), "pcie-ecam",
                             ecam_reg, 0, ecam_size);
    memory_region_add_subregion(get_system_memory(), ecam_base, ecam_alias);

    mmio_alias = g_new0(MemoryRegion, 1);
    mmio_reg = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 1);
    memory_region_init_alias(mmio_alias, OBJECT(dev), "pcie-mmio",
                             mmio_reg, mmio_base, mmio_size);
    memory_region_add_subregion(get_system_memory(), mmio_base, mmio_alias);

    /* Map high MMIO space */
    high_mmio_alias = g_new0(MemoryRegion, 1);
    memory_region_init_alias(high_mmio_alias, OBJECT(dev), "pcie-mmio-high",
                             mmio_reg, high_mmio_base, high_mmio_size);
    memory_region_add_subregion(get_system_memory(), high_mmio_base,
                                high_mmio_alias);

    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 2, pio_base);
    for (i = 0; i < GPEX_NUM_IRQS; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i, SIFIVE_PLIC(s->plic)->irqs[PCIE_IRQ + i]);
        gpex_set_irq_num(GPEX_HOST(dev), i, PCIE_IRQ + i);
    }

    return dev;
}

static void riscv_ranchu_board_init(MachineState *machine)
{
    const struct MemmapEntry *memmap = ranchu_memmap;

    RISCVVirtState *s = g_new0(RISCVVirtState, 1);
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *main_mem = g_new(MemoryRegion, 1);
    MemoryRegion *boot_rom = g_new(MemoryRegion, 1);
    char *plic_hart_config;
    size_t plic_hart_config_len;
    int i;
    void *fdt;

    /* Initialize SOC */
    object_initialize(&s->soc, sizeof(s->soc), TYPE_RISCV_HART_ARRAY);
    object_property_add_child(OBJECT(machine), "soc", OBJECT(&s->soc),
                              &error_abort);
    object_property_set_str(OBJECT(&s->soc), RANCHU_CPU, "cpu-type",
                            &error_abort);
    object_property_set_int(OBJECT(&s->soc), smp_cpus, "num-harts",
                            &error_abort);
    object_property_set_bool(OBJECT(&s->soc), true, "realized",
                            &error_abort);

    /* register system main memory (actual RAM) */
    memory_region_init_ram(main_mem, NULL, "riscv_ranchu_board.ram",
                           machine->ram_size, &error_fatal);
    memory_region_add_subregion(system_memory, memmap[RANCHU_DRAM].base,
        main_mem);

    /* create device tree */
    fdt = create_fdt(s, memmap, machine->ram_size, machine->kernel_cmdline);

    // FIXME: Can't bootup while enabling.
    if (device_tree_setup_func) {
        device_tree_setup_func (fdt);
    }

    /* create PLIC hart topology configuration string */
    plic_hart_config_len = (strlen(RANCHU_PLIC_HART_CONFIG) + 1) * smp_cpus;
    plic_hart_config = g_malloc0(plic_hart_config_len);
    for (i = 0; i < smp_cpus; i++) {
        if (i != 0) {
            strncat(plic_hart_config, ",", plic_hart_config_len);
        }
        strncat(plic_hart_config, RANCHU_PLIC_HART_CONFIG, plic_hart_config_len);
        plic_hart_config_len -= (strlen(RANCHU_PLIC_HART_CONFIG) + 1);
    }


    /* MMIO */
    s->plic = sifive_plic_create(memmap[RANCHU_PLIC].base,
        plic_hart_config,
        RANCHU_PLIC_NUM_SOURCES,
        RANCHU_PLIC_NUM_PRIORITIES,
        RANCHU_PLIC_PRIORITY_BASE,
        RANCHU_PLIC_PENDING_BASE,
        RANCHU_PLIC_ENABLE_BASE,
        RANCHU_PLIC_ENABLE_STRIDE,
        RANCHU_PLIC_CONTEXT_BASE,
        RANCHU_PLIC_CONTEXT_STRIDE,
        memmap[RANCHU_PLIC].size);
    sifive_clint_create(memmap[RANCHU_CLINT].base,
        memmap[RANCHU_CLINT].size, smp_cpus,
        SIFIVE_SIP_BASE, SIFIVE_TIMECMP_BASE, SIFIVE_TIME_BASE);

    virt_high_pcie_memmap.size = VIRT64_HIGH_PCIE_MMIO_SIZE;
    virt_high_pcie_memmap.base = memmap[RANCHU_DRAM].base + machine->ram_size;
    virt_high_pcie_memmap.base =
        ROUND_UP(virt_high_pcie_memmap.base, virt_high_pcie_memmap.size);

    for (i = 0; i < VIRTIO_COUNT; i++) {
        sysbus_create_simple("virtio-mmio",
            memmap[RANCHU_VIRTIO].base + i * memmap[RANCHU_VIRTIO].size,
            SIFIVE_PLIC(s->plic)->irqs[VIRTIO_IRQ + i]);
    }


    create_device(s, fdt, RANCHU_GOLDFISH_FB);
    create_device(s, fdt, RANCHU_GOLDFISH_PIPE);
    create_device(s, fdt, RANCHU_GOLDFISH_SYNC);
    create_device(s, fdt, RANCHU_GOLDFISH_AUDIO);
    create_device(s, fdt, RANCHU_GOLDFISH_EVDEV);
    create_device(s, fdt, RANCHU_GOLDFISH_BATTERY);

    /* boot rom */
    memory_region_init_ram(boot_rom, NULL, "riscv_ranchu_board.bootrom",
                           s->fdt_size + 0x2000, &error_fatal);
    memory_region_add_subregion(system_memory, 0x0, boot_rom);

    if (machine->firmware) {
        /* -bios set.  */
        riscv_load_firmware (machine->firmware, memmap[RANCHU_DRAM].base, NULL);
    }

    if (machine->kernel_filename) {
        uint64_t kernel_entry = riscv_load_kernel(machine->kernel_filename, NULL);

        if (machine->initrd_filename) {
            hwaddr start;
            hwaddr end = load_initrd(machine->initrd_filename,
                                     machine->ram_size, kernel_entry,
                                     &start);
            printf ("kernel@%lx, initrd@%lx\n", kernel_entry, start);
            qemu_fdt_setprop_cell(fdt, "/chosen",
                                  "linux,initrd-start", start);
            qemu_fdt_setprop_cell(fdt, "/chosen", "linux,initrd-end",
                                  end);
        }
    }

    /* reset vector */
    uint32_t reset_vec[8] = {
        0x00000297,                  /* 1:  auipc  t0, %pcrel_hi(dtb) */
        0x02028593,                  /*     addi   a1, t0, %pcrel_lo(1b) */
        0xf1402573,                  /*     csrr   a0, mhartid  */
#if defined(TARGET_RISCV32)
        0x0182a283,                  /*     lw     t0, 24(t0) */
#elif defined(TARGET_RISCV64)
        0x0182b283,                  /*     ld     t0, 24(t0) */
#endif
        0x00028067,                  /*     jr     t0 */
        0x00000000,
        memmap[RANCHU_DRAM].base,      /* start: .dword memmap[RANCHU_DRAM].base */
        0x00000000,
                                     /* dtb: */
    };

    /* copy in the reset vector */
    copy_le32_to_phys(ROM_BASE, reset_vec, sizeof(reset_vec));

    /* copy in the device tree */
    qemu_fdt_dumpdtb(s->fdt, s->fdt_size);
    cpu_physical_memory_write(ROM_BASE + sizeof(reset_vec),
        s->fdt, s->fdt_size);

    gpex_pcie_init(s, system_memory,
                   memmap[RANCHU_PCIE_ECAM].base,
                   memmap[RANCHU_PCIE_ECAM].size,
                   memmap[RANCHU_PCIE_MMIO].base,
                   memmap[RANCHU_PCIE_MMIO].size,
                   virt_high_pcie_memmap.base,
                   virt_high_pcie_memmap.size,
                   memmap[RANCHU_PCIE_PIO].base,
                   DEVICE(s->plic));

    serial_mm_init(system_memory, memmap[RANCHU_UART0].base,
        0, SIFIVE_PLIC(s->plic)->irqs[UART0_IRQ], 399193,
        serial_hds[0], DEVICE_LITTLE_ENDIAN);
}

static int riscv_ranchu_board_sysbus_device_init(SysBusDevice *sysbusdev)
{
    return 0;
}

static void riscv_ranchu_board_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    k->init = riscv_ranchu_board_sysbus_device_init;
}

static const TypeInfo riscv_ranchu_board_device = {
    .name          = TYPE_RISCV_RANCHU_BOARD,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(RISCVVirtState),
    .class_init    = riscv_ranchu_board_class_init,
};

static void riscv_ranchu_board_machine_init(MachineClass *mc)
{
    mc->desc = "RISC-V RANCHU Board (Privileged spec v1.10)";
    mc->init = riscv_ranchu_board_init;
    mc->pci_allow_0_address = true;
    mc->max_cpus = 8; /* hardcoded limit in BBL */
}

DEFINE_MACHINE("ranchu", riscv_ranchu_board_machine_init)

static void riscv_ranchu_board_register_types(void)
{
    type_register_static(&riscv_ranchu_board_device);
}

type_init(riscv_ranchu_board_register_types);
