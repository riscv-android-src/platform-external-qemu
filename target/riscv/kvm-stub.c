/*
 * QEMU KVM RISC-V specific function stubs
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "cpu.h"
#include "sysemu/kvm.h"

void* kvm_gpa2hva(uint64_t gpa, bool* found) {
    (void)gpa;
    *found = false;
    return NULL;
}
