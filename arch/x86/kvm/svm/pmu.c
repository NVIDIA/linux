// SPDX-License-Identifier: GPL-2.0-only
/*
 * KVM PMU support for AMD
 *
 * Copyright 2015, Red Hat, Inc. and/or its affiliates.
 *
 * Author:
 *   Wei Huang <wei@redhat.com>
 *
 * Implementation is based on pmu_intel.c file
 */
#include <linux/types.h>
#include <linux/kvm_host.h>
#include <linux/perf_event.h>
#include "x86.h"
#include "cpuid.h"
#include "lapic.h"
#include "pmu.h"
#include "svm.h"

enum pmu_type {
	PMU_TYPE_COUNTER = 0,
	PMU_TYPE_EVNTSEL,
};

<<<<<<< HEAD
enum index {
	INDEX_ZERO = 0,
	INDEX_ONE,
	INDEX_TWO,
	INDEX_THREE,
	INDEX_FOUR,
	INDEX_FIVE,
	INDEX_ERROR,
};

/* duplicated from amd_perfmon_event_map, K7 and above should work. */
static struct kvm_event_hw_type_mapping amd_event_mapping[] = {
	[0] = { 0x76, 0x00, PERF_COUNT_HW_CPU_CYCLES },
	[1] = { 0xc0, 0x00, PERF_COUNT_HW_INSTRUCTIONS },
	[2] = { 0x7d, 0x07, PERF_COUNT_HW_CACHE_REFERENCES },
	[3] = { 0x7e, 0x07, PERF_COUNT_HW_CACHE_MISSES },
	[4] = { 0xc2, 0x00, PERF_COUNT_HW_BRANCH_INSTRUCTIONS },
	[5] = { 0xc3, 0x00, PERF_COUNT_HW_BRANCH_MISSES },
	[6] = { 0xd0, 0x00, PERF_COUNT_HW_STALLED_CYCLES_FRONTEND },
	[7] = { 0xd1, 0x00, PERF_COUNT_HW_STALLED_CYCLES_BACKEND },
};

/* duplicated from amd_f17h_perfmon_event_map. */
static struct kvm_event_hw_type_mapping amd_f17h_event_mapping[] = {
	[0] = { 0x76, 0x00, PERF_COUNT_HW_CPU_CYCLES },
	[1] = { 0xc0, 0x00, PERF_COUNT_HW_INSTRUCTIONS },
	[2] = { 0x60, 0xff, PERF_COUNT_HW_CACHE_REFERENCES },
	[3] = { 0x64, 0x09, PERF_COUNT_HW_CACHE_MISSES },
	[4] = { 0xc2, 0x00, PERF_COUNT_HW_BRANCH_INSTRUCTIONS },
	[5] = { 0xc3, 0x00, PERF_COUNT_HW_BRANCH_MISSES },
	[6] = { 0x87, 0x02, PERF_COUNT_HW_STALLED_CYCLES_FRONTEND },
	[7] = { 0x87, 0x01, PERF_COUNT_HW_STALLED_CYCLES_BACKEND },
};

/* amd_pmc_perf_hw_id depends on these being the same size */
static_assert(ARRAY_SIZE(amd_event_mapping) ==
	     ARRAY_SIZE(amd_f17h_event_mapping));

static unsigned int get_msr_base(struct kvm_pmu *pmu, enum pmu_type type)
=======
static struct kvm_pmc *amd_pmc_idx_to_pmc(struct kvm_pmu *pmu, int pmc_idx)
>>>>>>> origin/linux_6.1.15_upstream
{
	unsigned int num_counters = pmu->nr_arch_gp_counters;

	if (pmc_idx >= num_counters)
		return NULL;

	return &pmu->gp_counters[array_index_nospec(pmc_idx, num_counters)];
}

static inline struct kvm_pmc *get_gp_pmc_amd(struct kvm_pmu *pmu, u32 msr,
					     enum pmu_type type)
{
	struct kvm_vcpu *vcpu = pmu_to_vcpu(pmu);
	unsigned int idx;

	if (!vcpu->kvm->arch.enable_pmu)
		return NULL;

	switch (msr) {
	case MSR_F15H_PERF_CTL0 ... MSR_F15H_PERF_CTR5:
		if (!guest_cpuid_has(vcpu, X86_FEATURE_PERFCTR_CORE))
			return NULL;
		/*
		 * Each PMU counter has a pair of CTL and CTR MSRs. CTLn
		 * MSRs (accessed via EVNTSEL) are even, CTRn MSRs are odd.
		 */
		idx = (unsigned int)((msr - MSR_F15H_PERF_CTL0) / 2);
		if (!(msr & 0x1) != (type == PMU_TYPE_EVNTSEL))
			return NULL;
		break;
	case MSR_K7_EVNTSEL0 ... MSR_K7_EVNTSEL3:
		if (type != PMU_TYPE_EVNTSEL)
			return NULL;
		idx = msr - MSR_K7_EVNTSEL0;
		break;
	case MSR_K7_PERFCTR0 ... MSR_K7_PERFCTR3:
		if (type != PMU_TYPE_COUNTER)
			return NULL;
		idx = msr - MSR_K7_PERFCTR0;
		break;
	default:
		return NULL;
	}

	return amd_pmc_idx_to_pmc(pmu, idx);
}

<<<<<<< HEAD
static unsigned int amd_pmc_perf_hw_id(struct kvm_pmc *pmc)
{
	struct kvm_event_hw_type_mapping *event_mapping;
	u8 event_select = pmc->eventsel & ARCH_PERFMON_EVENTSEL_EVENT;
	u8 unit_mask = (pmc->eventsel & ARCH_PERFMON_EVENTSEL_UMASK) >> 8;
	int i;

	if (guest_cpuid_family(pmc->vcpu) >= 0x17)
		event_mapping = amd_f17h_event_mapping;
	else
		event_mapping = amd_event_mapping;

	for (i = 0; i < ARRAY_SIZE(amd_event_mapping); i++)
		if (event_mapping[i].eventsel == event_select
		    && event_mapping[i].unit_mask == unit_mask)
			break;

	if (i == ARRAY_SIZE(amd_event_mapping))
		return PERF_COUNT_HW_MAX;

	return event_mapping[i].event_type;
}

/* return PERF_COUNT_HW_MAX as AMD doesn't have fixed events */
static unsigned amd_find_fixed_event(int idx)
{
	return PERF_COUNT_HW_MAX;
=======
static bool amd_hw_event_available(struct kvm_pmc *pmc)
{
	return true;
>>>>>>> origin/linux_6.1.15_upstream
}

/* check if a PMC is enabled by comparing it against global_ctrl bits. Because
 * AMD CPU doesn't have global_ctrl MSR, all PMCs are enabled (return TRUE).
 */
static bool amd_pmc_is_enabled(struct kvm_pmc *pmc)
{
	return true;
}

static bool amd_is_valid_rdpmc_ecx(struct kvm_vcpu *vcpu, unsigned int idx)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);

	idx &= ~(3u << 30);

	return idx < pmu->nr_arch_gp_counters;
}

/* idx is the ECX register of RDPMC instruction */
static struct kvm_pmc *amd_rdpmc_ecx_to_pmc(struct kvm_vcpu *vcpu,
	unsigned int idx, u64 *mask)
{
	return amd_pmc_idx_to_pmc(vcpu_to_pmu(vcpu), idx & ~(3u << 30));
}

static bool amd_is_valid_msr(struct kvm_vcpu *vcpu, u32 msr)
{
	/* All MSRs refer to exactly one PMC, so msr_idx_to_pmc is enough.  */
	return false;
}

static struct kvm_pmc *amd_msr_idx_to_pmc(struct kvm_vcpu *vcpu, u32 msr)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
	struct kvm_pmc *pmc;

	pmc = get_gp_pmc_amd(pmu, msr, PMU_TYPE_COUNTER);
	pmc = pmc ? pmc : get_gp_pmc_amd(pmu, msr, PMU_TYPE_EVNTSEL);

	return pmc;
}

static int amd_pmu_get_msr(struct kvm_vcpu *vcpu, struct msr_data *msr_info)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
	struct kvm_pmc *pmc;
	u32 msr = msr_info->index;

	/* MSR_PERFCTRn */
	pmc = get_gp_pmc_amd(pmu, msr, PMU_TYPE_COUNTER);
	if (pmc) {
		msr_info->data = pmc_read_counter(pmc);
		return 0;
	}
	/* MSR_EVNTSELn */
	pmc = get_gp_pmc_amd(pmu, msr, PMU_TYPE_EVNTSEL);
	if (pmc) {
		msr_info->data = pmc->eventsel;
		return 0;
	}

	return 1;
}

static int amd_pmu_set_msr(struct kvm_vcpu *vcpu, struct msr_data *msr_info)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
	struct kvm_pmc *pmc;
	u32 msr = msr_info->index;
	u64 data = msr_info->data;

	/* MSR_PERFCTRn */
	pmc = get_gp_pmc_amd(pmu, msr, PMU_TYPE_COUNTER);
	if (pmc) {
		pmc->counter += data - pmc_read_counter(pmc);
		pmc_update_sample_period(pmc);
		return 0;
	}
	/* MSR_EVNTSELn */
	pmc = get_gp_pmc_amd(pmu, msr, PMU_TYPE_EVNTSEL);
	if (pmc) {
		data &= ~pmu->reserved_bits;
<<<<<<< HEAD
		if (data != pmc->eventsel)
			reprogram_gp_counter(pmc, data);
=======
		if (data != pmc->eventsel) {
			pmc->eventsel = data;
			reprogram_counter(pmc);
		}
>>>>>>> origin/linux_6.1.15_upstream
		return 0;
	}

	return 1;
}

static void amd_pmu_refresh(struct kvm_vcpu *vcpu)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);

	if (guest_cpuid_has(vcpu, X86_FEATURE_PERFCTR_CORE))
		pmu->nr_arch_gp_counters = AMD64_NUM_COUNTERS_CORE;
	else
		pmu->nr_arch_gp_counters = AMD64_NUM_COUNTERS;

	pmu->counter_bitmask[KVM_PMC_GP] = ((u64)1 << 48) - 1;
	pmu->reserved_bits = 0xfffffff000280000ull;
	pmu->raw_event_mask = AMD64_RAW_EVENT_MASK;
	pmu->version = 1;
	/* not applicable to AMD; but clean them to prevent any fall out */
	pmu->counter_bitmask[KVM_PMC_FIXED] = 0;
	pmu->nr_arch_fixed_counters = 0;
	pmu->global_status = 0;
	bitmap_set(pmu->all_valid_pmc_idx, 0, pmu->nr_arch_gp_counters);
}

static void amd_pmu_init(struct kvm_vcpu *vcpu)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
	int i;

	BUILD_BUG_ON(KVM_AMD_PMC_MAX_GENERIC > AMD64_NUM_COUNTERS_CORE);
	BUILD_BUG_ON(KVM_AMD_PMC_MAX_GENERIC > INTEL_PMC_MAX_GENERIC);

	for (i = 0; i < KVM_AMD_PMC_MAX_GENERIC ; i++) {
		pmu->gp_counters[i].type = KVM_PMC_GP;
		pmu->gp_counters[i].vcpu = vcpu;
		pmu->gp_counters[i].idx = i;
		pmu->gp_counters[i].current_config = 0;
	}
}

static void amd_pmu_reset(struct kvm_vcpu *vcpu)
{
	struct kvm_pmu *pmu = vcpu_to_pmu(vcpu);
	int i;

	for (i = 0; i < KVM_AMD_PMC_MAX_GENERIC; i++) {
		struct kvm_pmc *pmc = &pmu->gp_counters[i];

		pmc_stop_counter(pmc);
		pmc->counter = pmc->eventsel = 0;
	}
}

<<<<<<< HEAD
struct kvm_pmu_ops amd_pmu_ops = {
	.pmc_perf_hw_id = amd_pmc_perf_hw_id,
	.find_fixed_event = amd_find_fixed_event,
=======
struct kvm_pmu_ops amd_pmu_ops __initdata = {
	.hw_event_available = amd_hw_event_available,
>>>>>>> origin/linux_6.1.15_upstream
	.pmc_is_enabled = amd_pmc_is_enabled,
	.pmc_idx_to_pmc = amd_pmc_idx_to_pmc,
	.rdpmc_ecx_to_pmc = amd_rdpmc_ecx_to_pmc,
	.msr_idx_to_pmc = amd_msr_idx_to_pmc,
	.is_valid_rdpmc_ecx = amd_is_valid_rdpmc_ecx,
	.is_valid_msr = amd_is_valid_msr,
	.get_msr = amd_pmu_get_msr,
	.set_msr = amd_pmu_set_msr,
	.refresh = amd_pmu_refresh,
	.init = amd_pmu_init,
	.reset = amd_pmu_reset,
};
