/*
 * SiFive VirtIO Board
 *
 * Copyright (c) 2017 SiFive, Inc.
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

#ifndef HW_RISCV_RANCHU_H
#define HW_RISCV_RANCHU_H

#define TYPE_RISCV_RANCHU_BOARD "riscv.ranchu"
#define RANCHU(obj) \
    OBJECT_CHECK(RISCVVirtState, (obj), TYPE_RISCV_RANCHU_BOARD)

enum { ROM_BASE = 0x1000};

typedef struct {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    RISCVHartArrayState soc;
    DeviceState *plic;
    void *fdt;
    int fdt_size;
} RISCVVirtState;


enum {
    UART0_IRQ = 10,
    RANCHU_GOLDFISH_FB_IRQ,
    RANCHU_GOLDFISH_AUDIO_IRQ,
    RANCHU_GOLDFISH_EVDEV_IRQ,
    RANCHU_GOLDFISH_PIPE_IRQ,
    RANCHU_GOLDFISH_SYNC_IRQ,
    VIRTIO_IRQ = 1, /* 1 to 8 */
    VIRTIO_COUNT = 8,
    VIRTIO_NDEV = 10
};

#define RANCHU_PLIC_HART_CONFIG "MS"
#define RANCHU_PLIC_NUM_SOURCES 127
#define RANCHU_PLIC_NUM_PRIORITIES 7
#define RANCHU_PLIC_PRIORITY_BASE 0x0
#define RANCHU_PLIC_PENDING_BASE 0x1000
#define RANCHU_PLIC_ENABLE_BASE 0x2000
#define RANCHU_PLIC_ENABLE_STRIDE 0x80
#define RANCHU_PLIC_CONTEXT_BASE 0x200000
#define RANCHU_PLIC_CONTEXT_STRIDE 0x1000

#if defined(TARGET_RISCV32)
#define RANCHU_CPU TYPE_RISCV_CPU_RV32GCSU_V1_10_0
#elif defined(TARGET_RISCV64)
#define RANCHU_CPU TYPE_RISCV_CPU_RV64GCSU_V1_10_0
#endif

#endif
