/*
 * ARM NVIDIA Tegra2 emulation.
 *
 * Copyright (c) 2014-2015 Dmitry Osipenko <digetx@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "tegra_common.h"

#include "hw/arm/boot.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "exec/exec-all.h"

#include "devices.h"
#include "iomap.h"
#include "tegra_cpu.h"
#include "tegra_trace.h"

#define HALT_WFE    0xff

static int tegra_cpus_by_id[TEGRA_NCPUS] = {};
static int tegra_cpus_by_index[TEGRA_NCPUS] = {};

void add_tegra_cpu(int cpu_id, int cpu_index)
{
    tegra_cpus_by_id[cpu_id] = cpu_index + 1;
    tegra_cpus_by_index[cpu_index] = cpu_id + 1;
}

CPUState* tegra_get_cpu(int cpu_id)
{
    if (cpu_id < 0 || cpu_id >= TEGRA_NCPUS) {
        return NULL;
    }
    int idx = tegra_cpus_by_id[cpu_id] - 1;
    assert(idx < 0 || qemu_get_cpu(idx));
    return idx < 0 ? NULL : qemu_get_cpu(idx);
}

int get_tegra_cpu_id(int cpu_index)
{
    if (cpu_index < 0 || cpu_index >= TEGRA_NCPUS) {
        return -1;
    }

    return tegra_cpus_by_index[cpu_index] - 1;
}

int __attribute__((const)) tegra_sibling_cpu(int cpu_id)
{
    switch (cpu_id) {
    case TEGRA2_A9_CORE0:
        return TEGRA2_A9_CORE1;
        break;
    case TEGRA2_A9_CORE1:
        return TEGRA2_A9_CORE0;
        break;
    }

    return cpu_id;
}

uint32_t tegra_get_wfe_bitmap(void)
{
    uint32_t wfe_bitmap = 0;
    int i;

    for (i = 0; i < TEGRA2_A9_NCORES; i++) {
        CPUState *cs = tegra_get_cpu(TEGRA2_A9_CORE0 + i);
        wfe_bitmap |= (cs->halted == HALT_WFE) << i;
    }

    return wfe_bitmap;
}


void HELPER(wfe)(CPUARMState *env)
{
    CPUState *cs = env_cpu(env);
    int cpu_id = cs->cpu_index;

    if (tegra_get_cpu(cpu_id)) {
//         TPRINT("WFE: cpu %d\n", cpu_id);

        cs->halted = HALT_WFE;

        tegra_flow_wfe_handle(cpu_id);

        /* Won't return here if flow powergated CPU.  */
        cs->halted = 0;
    }

    HELPER(yield)(env);
}
