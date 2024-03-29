/*
 * Copyright (C) 2011-2014, 2016 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU license.
 *
 * A copy of the license is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __MALI_MMU_PAGE_DIRECTORY_H__
#define __MALI_MMU_PAGE_DIRECTORY_H__

#include "mali_osk.h"

/**
 * Size of an MMU page in bytes
 */
#define MALI_MMU_PAGE_SIZE 0x1000

/*
 * Size of the address space referenced by a page table page
 */
#define MALI_MMU_VIRTUAL_PAGE_SIZE 0x400000 /* 4 MiB */

/**
 * Page directory index from address
 * Calculates the page directory index from the given address
 */
#define MALI_MMU_PDE_ENTRY(address) (((address)>>22) & 0x03FF)

/**
 * Page table index from address
 * Calculates the page table index from the given address
 */
#define MALI_MMU_PTE_ENTRY(address) (((address)>>12) & 0x03FF)

/**
 * Extract the memory address from an PDE/PTE entry
 */
#define MALI_MMU_ENTRY_ADDRESS(value) ((value) & 0xFFFFFC00)

#define MALI_INVALID_PAGE ((u32)(~0))

/**
 *
 */
typedef enum mali_mmu_entry_flags {
	MALI_MMU_FLAGS_PRESENT = 0x01,
	MALI_MMU_FLAGS_READ_PERMISSION = 0x02,
	MALI_MMU_FLAGS_WRITE_PERMISSION = 0x04,
	MALI_MMU_FLAGS_OVERRIDE_CACHE  = 0x8,
	MALI_MMU_FLAGS_WRITE_CACHEABLE  = 0x10,
	MALI_MMU_FLAGS_WRITE_ALLOCATE  = 0x20,
	MALI_MMU_FLAGS_WRITE_BUFFERABLE  = 0x40,
	MALI_MMU_FLAGS_READ_CACHEABLE  = 0x80,
	MALI_MMU_FLAGS_READ_ALLOCATE  = 0x100,
	MALI_MMU_FLAGS_MASK = 0x1FF,
} mali_mmu_entry_flags;


#define MALI_MMU_FLAGS_FORCE_GP_READ_ALLOCATE ( \
		MALI_MMU_FLAGS_PRESENT | \
		MALI_MMU_FLAGS_READ_PERMISSION |  \
		MALI_MMU_FLAGS_WRITE_PERMISSION | \
		MALI_MMU_FLAGS_OVERRIDE_CACHE | \
		MALI_MMU_FLAGS_WRITE_CACHEABLE | \
		MALI_MMU_FLAGS_WRITE_BUFFERABLE | \
		MALI_MMU_FLAGS_READ_CACHEABLE | \
		MALI_MMU_FLAGS_READ_ALLOCATE )

#define MALI_MMU_FLAGS_DEFAULT ( \
				 MALI_MMU_FLAGS_PRESENT | \
				 MALI_MMU_FLAGS_READ_PERMISSION |  \
				 MALI_MMU_FLAGS_WRITE_PERMISSION )


struct mali_page_directory {
	u32 page_directory; /**< Physical address of the memory session's page directory */
	mali_io_address page_directory_mapped; /**< Pointer to the mapped version of the page directory into the kernel's address space */

	mali_io_address page_entries_mapped[1024]; /**< Pointers to the page tables which exists in the page directory mapped into the kernel's address space */
	u32   page_entries_usage_count[1024]; /**< Tracks usage count of the page table pages, so they can be releases on the last reference */
};

/* Map Mali virtual address space (i.e. ensure page tables exist for the virtual range)  */
_mali_osk_errcode_t mali_mmu_pagedir_map(struct mali_page_directory *pagedir, u32 mali_address, u32 size);
_mali_osk_errcode_t mali_mmu_pagedir_unmap(struct mali_page_directory *pagedir, u32 mali_address, u32 size);

/* Back virtual address space with actual pages. Assumes input is contiguous and 4k aligned. */
void mali_mmu_pagedir_update(struct mali_page_directory *pagedir, u32 mali_address,
			     mali_dma_addr phys_address, u32 size, u32 permission_bits);

u32 mali_allocate_empty_page(mali_io_address *virtual);
void mali_free_empty_page(mali_dma_addr address, mali_io_address virt_addr);
_mali_osk_errcode_t mali_create_fault_flush_pages(mali_dma_addr *page_directory,
		mali_io_address *page_directory_mapping,
		mali_dma_addr *page_table, mali_io_address *page_table_mapping,
		mali_dma_addr *data_page, mali_io_address *data_page_mapping);
void mali_destroy_fault_flush_pages(
	mali_dma_addr *page_directory, mali_io_address *page_directory_mapping,
	mali_dma_addr *page_table, mali_io_address *page_table_mapping,
	mali_dma_addr *data_page, mali_io_address *data_page_mapping);

struct mali_page_directory *mali_mmu_pagedir_alloc(void);
void mali_mmu_pagedir_free(struct mali_page_directory *pagedir);

void mali_mmu_pagedir_diag(struct mali_page_directory *pagedir, u32 fault_addr);

#endif /* __MALI_MMU_PAGE_DIRECTORY_H__ */
