// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2019-2022 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/memory_group_manager.h>

#include <mali_kbase.h>
#include <mali_kbase_native_mgm.h>

#if CONFIG_MALI_USE_ION
#define ION_FREE_DBG 0
#include <linux/dma-buf.h>
#include <linux/amlogic/ion.h>
#include <linux/spinlock.h>
unsigned int meson_ion_cma_heap_id_get(void);
#define ION_FLAG_EXTEND_MESON_HEAP          BIT(30)
LIST_HEAD(cma_list);
static DEFINE_SPINLOCK(cma_list_lock);
#endif

/**
 * kbase_native_mgm_alloc - Native physical memory allocation method
 *
 * @mgm_dev:  The memory group manager the request is being made through.
 * @group_id: A physical memory group ID, which must be valid but is not used.
 *            Its valid range is 0 .. MEMORY_GROUP_MANAGER_NR_GROUPS-1.
 * @gfp_mask: Bitmask of Get Free Page flags affecting allocator behavior.
 * @order:    Page order for physical page size (order=0 means 4 KiB,
 *            order=9 means 2 MiB).
 *
 * Delegates all memory allocation requests to the kernel's alloc_pages
 * function.
 *
 * Return: Pointer to allocated page, or NULL if allocation failed.
 */
static struct page *kbase_native_mgm_alloc(
	struct memory_group_manager_device *mgm_dev, int group_id,
	gfp_t gfp_mask, unsigned int order)
{
	/*
	 * Check that the base and the mgm defines, from separate header files,
	 * for the max number of memory groups are compatible.
	 */
	BUILD_BUG_ON(BASE_MEM_GROUP_COUNT != MEMORY_GROUP_MANAGER_NR_GROUPS);
	/*
	 * Check that the mask used for storing the memory group ID is big
	 * enough for the largest possible memory group ID.
	 */
	BUILD_BUG_ON((BASEP_CONTEXT_MMU_GROUP_ID_MASK
				>> BASEP_CONTEXT_MMU_GROUP_ID_SHIFT)
			< (BASE_MEM_GROUP_COUNT - 1));

	CSTD_UNUSED(mgm_dev);
	CSTD_UNUSED(group_id);

	/* alloc from ion */
	#if CONFIG_MALI_USE_ION
	struct dma_buf *dmabuf = NULL;
	struct ion_buffer *buffer;
	struct page *ret = NULL;
	unsigned int id, size;
	unsigned long flags;
	size = PAGE_SIZE << order;
	id = meson_ion_cma_heap_id_get();
	if (id)
		dmabuf = ion_alloc(size, (1 << id),
				ION_FLAG_EXTEND_MESON_HEAP);
	if (IS_ERR_OR_NULL(dmabuf)) {
		printk("%s: Failed to alloc ion-cma", __func__);
		return NULL;
	}
	buffer = (struct ion_buffer *)dmabuf->priv;
	sg_dma_address(buffer->sg_table->sgl) = sg_phys(buffer->sg_table->sgl);
	ret = sg_page(buffer->sg_table->sgl);
	struct memory_group_cma *cma_group;
	cma_group = kzalloc(sizeof(struct memory_group_cma), GFP_KERNEL);
	cma_group->dmabuf = dmabuf;
	cma_group->page = ret;
	INIT_LIST_HEAD(&cma_group->list);
	spin_lock_irqsave(&cma_list_lock, flags);
	list_add_tail(&cma_group->list, &cma_list);
	spin_unlock_irqrestore(&cma_list_lock, flags);
	#if ION_FREE_DBG
	printk("%s: ion_alloc dmabuf=0x%px, cg=0x%px, page=0x%px",
		__func__, dmabuf, cma_group, ret);
	#endif
	return ret;
	#else
	return alloc_pages(gfp_mask, order);
	#endif
}

/**
 * kbase_native_mgm_free - Native physical memory freeing method
 *
 * @mgm_dev:  The memory group manager the request is being made through.
 * @group_id: A physical memory group ID, which must be valid but is not used.
 *            Its valid range is 0 .. MEMORY_GROUP_MANAGER_NR_GROUPS-1.
 * @page:     Address of the struct associated with a page of physical
 *            memory that was allocated by calling kbase_native_mgm_alloc
 *            with the same argument values.
 * @order:    Page order for physical page size (order=0 means 4 KiB,
 *            order=9 means 2 MiB).
 *
 * Delegates all memory freeing requests to the kernel's __free_pages function.
 */
static void kbase_native_mgm_free(struct memory_group_manager_device *mgm_dev,
	int group_id, struct page *page, unsigned int order)
{
	CSTD_UNUSED(mgm_dev);
	CSTD_UNUSED(group_id);

	#if CONFIG_MALI_USE_ION
	unsigned long flags;
	struct memory_group_cma *pos = NULL, *tmp = NULL;
	#if ION_FREE_DBG
	printk("%s: page=0x%px", __func__, page);
	#endif
	spin_lock_irqsave(&cma_list_lock, flags);
	list_for_each_entry_safe(pos, tmp, &cma_list, list) {
		if (pos->page == page) {
			#if ION_FREE_DBG
			printk("%s: dma_buf_put 0x%px, cg=0x%px",
				__func__, pos->dmabuf, pos);
			#endif
			dma_buf_put(pos->dmabuf);
			list_del(&pos->list);
			pos->dmabuf = NULL;
			pos->page = NULL;
			kfree(pos);
		}
	}
	spin_unlock_irqrestore(&cma_list_lock, flags);
	#else
	__free_pages(page, order);
	#endif
}

/**
 * kbase_native_mgm_vmf_insert_pfn_prot - Native method to map a page on the CPU
 *
 * @mgm_dev:  The memory group manager the request is being made through.
 * @group_id: A physical memory group ID, which must be valid but is not used.
 *            Its valid range is 0 .. MEMORY_GROUP_MANAGER_NR_GROUPS-1.
 * @vma:      The virtual memory area to insert the page into.
 * @addr:     An address contained in @vma to assign to the inserted page.
 * @pfn:      The kernel Page Frame Number to insert at @addr in @vma.
 * @pgprot:   Protection flags for the inserted page.
 *
 * Called from a CPU virtual memory page fault handler. Delegates all memory
 * mapping requests to the kernel's vmf_insert_pfn_prot function.
 *
 * Return: Type of fault that occurred or VM_FAULT_NOPAGE if the page table
 *         entry was successfully installed.
 */
static vm_fault_t kbase_native_mgm_vmf_insert_pfn_prot(
		struct memory_group_manager_device *mgm_dev, int group_id,
		struct vm_area_struct *vma, unsigned long addr,
		unsigned long pfn, pgprot_t pgprot)
{
	CSTD_UNUSED(mgm_dev);
	CSTD_UNUSED(group_id);

	return vmf_insert_pfn_prot(vma, addr, pfn, pgprot);
}

/**
 * kbase_native_mgm_update_gpu_pte - Native method to modify a GPU page table
 *                                   entry
 *
 * @mgm_dev:   The memory group manager the request is being made through.
 * @group_id:  A physical memory group ID, which must be valid but is not used.
 *             Its valid range is 0 .. MEMORY_GROUP_MANAGER_NR_GROUPS-1.
 * @mmu_level: The level of the MMU page table where the page is getting mapped.
 * @pte:       The prepared page table entry.
 *
 * This function simply returns the @pte without modification.
 *
 * Return: A GPU page table entry to be stored in a page table.
 */
static u64
kbase_native_mgm_update_gpu_pte(struct memory_group_manager_device *mgm_dev,
			      int group_id, int mmu_level, u64 pte)
{
	CSTD_UNUSED(mgm_dev);
	CSTD_UNUSED(group_id);
	CSTD_UNUSED(mmu_level);

	return pte;
}

/**
 * kbase_native_mgm_pte_to_original_pte - Native method to undo changes done in
 *                                        kbase_native_mgm_update_gpu_pte()
 *
 * @mgm_dev:   The memory group manager the request is being made through.
 * @group_id:  A physical memory group ID, which must be valid but is not used.
 *             Its valid range is 0 .. MEMORY_GROUP_MANAGER_NR_GROUPS-1.
 * @mmu_level: The level of the MMU page table where the page is getting mapped.
 * @pte:       The prepared page table entry.
 *
 * This function simply returns the @pte without modification.
 *
 * Return: A GPU page table entry to be stored in a page table.
 */
static u64 kbase_native_mgm_pte_to_original_pte(struct memory_group_manager_device *mgm_dev,
						int group_id, int mmu_level, u64 pte)
{
	CSTD_UNUSED(mgm_dev);
	CSTD_UNUSED(group_id);
	CSTD_UNUSED(mmu_level);

	return pte;
}

struct memory_group_manager_device kbase_native_mgm_dev = {
	.ops = {
		.mgm_alloc_page = kbase_native_mgm_alloc,
		.mgm_free_page = kbase_native_mgm_free,
		.mgm_get_import_memory_id = NULL,
		.mgm_vmf_insert_pfn_prot = kbase_native_mgm_vmf_insert_pfn_prot,
		.mgm_update_gpu_pte = kbase_native_mgm_update_gpu_pte,
		.mgm_pte_to_original_pte = kbase_native_mgm_pte_to_original_pte,
	},
	.data = NULL
};
