/*
 * Copyright (C) 2010, 2013-2014, 2016 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation, and any use by you of this program is subject to the terms of such GNU license.
 *
 * A copy of the license is included with the program, and can also be obtained from Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/**
 * @file mali_osk_atomics.c
 * Implementation of the OS abstraction layer for the kernel device driver
 */

#include "mali_osk.h"
#include <asm/atomic.h>
#include "mali_kernel_common.h"

void _mali_osk_atomic_dec(_mali_osk_atomic_t *atom)
{
	atomic_dec((atomic_t *)&atom->u.val);
}

u32 _mali_osk_atomic_dec_return(_mali_osk_atomic_t *atom)
{
	return atomic_dec_return((atomic_t *)&atom->u.val);
}

void _mali_osk_atomic_inc(_mali_osk_atomic_t *atom)
{
	atomic_inc((atomic_t *)&atom->u.val);
}

u32 _mali_osk_atomic_inc_return(_mali_osk_atomic_t *atom)
{
	return atomic_inc_return((atomic_t *)&atom->u.val);
}

void _mali_osk_atomic_init(_mali_osk_atomic_t *atom, u32 val)
{
	MALI_DEBUG_ASSERT_POINTER(atom);
	atomic_set((atomic_t *)&atom->u.val, val);
}

u32 _mali_osk_atomic_read(_mali_osk_atomic_t *atom)
{
	return atomic_read((atomic_t *)&atom->u.val);
}

void _mali_osk_atomic_term(_mali_osk_atomic_t *atom)
{
	MALI_IGNORE(atom);
}

u32 _mali_osk_atomic_xchg(_mali_osk_atomic_t *atom, u32 val)
{
	return atomic_xchg((atomic_t *)&atom->u.val, val);
}
