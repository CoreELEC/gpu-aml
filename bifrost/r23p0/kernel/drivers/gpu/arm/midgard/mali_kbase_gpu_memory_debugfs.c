/*
 *
 * (C) COPYRIGHT 2012-2017, 2019 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
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
 * SPDX-License-Identifier: GPL-2.0
 *
 */

#include <mali_kbase.h>
#include <device/mali_kbase_device.h>

#ifdef CONFIG_DEBUG_FS
static int kbasep_gpu_memory_detail_seq_show(struct seq_file *sfile, void *data)
{
	struct list_head *entry;
	const struct list_head *kbdev_list;
	seq_printf(sfile, "  %-25s  %-17s  %-12s  %-12s  %-12s  %-10s  %-25s\n"\
				"==============================================================================================================================\n",
				"Name (:Kbytes)", "context", "pid(tgid/tid)", "total_gpu_mem", "max_gpu_mem", "texture", "buf_mem(byte)");
	kbdev_list = kbase_device_get_list();
	list_for_each(entry, kbdev_list) {
		struct kbase_device *kbdev = NULL;
		struct kbase_context *kctx;
		kbdev = list_entry(entry, struct kbase_device, entry);
		mutex_lock(&kbdev->kctx_list_lock);
		list_for_each_entry(kctx, &kbdev->kctx_list, kctx_list_link) {
			struct task_struct *pTaskSt = pid_task(find_vpid(kctx->tgid), PIDTYPE_PID);
			seq_printf(sfile, "\n %-27s  %-15p  %5u/%-10u  %-10lu  %-15lu  %-10llu  %-10llu \n",
					pTaskSt->comm, (void *)kctx, kctx->tgid, kctx->pid,
					(atomic_read(&(kctx->used_pages)) * PAGE_SIZE / 1024),
					(kctx->max_used_pages * PAGE_SIZE) / 1024, kctx->tex_mem / 1024, kctx->buf_mem);
		}
		seq_printf(sfile, "    %-50s  %-15lu\n", "gpu_mem_total", (atomic_read(&kbdev->memdev.used_pages) * PAGE_SIZE) / 1024);
		mutex_unlock(&kbdev->kctx_list_lock);
	}
	kbase_device_put_list(kbdev_list);
	return 0;
}
#endif
#ifdef CONFIG_DEBUG_FS
/** Show callback for the @c gpu_memory debugfs file.
 *
 * This function is called to get the contents of the @c gpu_memory debugfs
 * file. This is a report of current gpu memory usage.
 *
 * @param sfile The debugfs entry
 * @param data Data associated with the entry
 *
 * @return 0 if successfully prints data in debugfs entry file
 *         -1 if it encountered an error
 */
#ifdef MALI_GPU_MEMORY_DEBUGFS
static int kbasep_gpu_memory_seq_show(struct seq_file *sfile, void *data)
{
	struct list_head *entry;
	const struct list_head *kbdev_list;

	seq_printf(sfile, "  %-25s  %-10s  %-10s  %-15s\n"\
		"===================================================================\n",
		"Name (:Kbytes)", "pid", "gpu_mem", "max_gpu_mem");

	kbdev_list = kbase_device_get_list();
	list_for_each(entry, kbdev_list) {
		struct kbase_device *kbdev = NULL;
		struct kbase_context *kctx;

		kbdev = list_entry(entry, struct kbase_device, entry);

		/* output the total memory usage and cap for this device */
		//seq_printf(sfile, "%-16s  %10u\n", kbdev->devname,    atomic_read(&(kbdev->memdev.used_pages)));

		mutex_lock(&kbdev->kctx_list_lock);

		list_for_each_entry(kctx, &kbdev->kctx_list, kctx_list_link) {
			struct task_struct *pTaskSt = pid_task(find_vpid(kctx->tgid), PIDTYPE_PID);
			seq_printf(sfile, "%-27s  %-10u  %-10lu  %-15lu  %-10u\n",
					pTaskSt->comm, kctx->tgid,
					(atomic_read(&(kctx->used_pages)) * PAGE_SIZE) / 1024,
					(kctx->max_used_pages* PAGE_SIZE) / 1024,
					kctx->pid );
		}

		seq_printf(sfile, "  %-20s  %-15lu\n", "gpu_mem_total", (atomic_read(&kbdev->memdev.used_pages) * PAGE_SIZE) / 1024);

		mutex_unlock(&kbdev->kctx_list_lock);
	}
	kbase_device_put_list(kbdev_list);
	return 0;
}
#else
static int kbasep_gpu_memory_seq_show(struct seq_file *sfile, void *data)
{
	struct list_head *entry;
	const struct list_head *kbdev_list;

	kbdev_list = kbase_device_get_list();
	list_for_each(entry, kbdev_list) {
		struct kbase_device *kbdev = NULL;
		struct kbase_context *kctx;

		kbdev = list_entry(entry, struct kbase_device, entry);
		/* output the total memory usage and cap for this device */
		seq_printf(sfile, "%-16s %-16s %10u\n",
				kbdev->devname,
				"total used_pages",
				atomic_read(&(kbdev->memdev.used_pages)));
		mutex_lock(&kbdev->kctx_list_lock);
		seq_puts(sfile, "----------------------------------------------------\n");
		seq_printf(sfile, "%-16s %-16s %-16s\n",
				"kctx", "pid", "used_pages");
		seq_puts(sfile, "----------------------------------------------------\n");
		list_for_each_entry(kctx, &kbdev->kctx_list, kctx_list_link) {
			/* output the memory usage and cap for each kctx
			* opened on this device */
			seq_printf(sfile, "%p %10u %10u\n",
				"kctx",
				kctx->tgid,
				atomic_read(&(kctx->used_pages)));
		}
		mutex_unlock(&kbdev->kctx_list_lock);
	}
	kbase_device_put_list(kbdev_list);
	return 0;
}
#endif

/*
 *  File operations related to debugfs entry for gpu_memory
 */
static int kbasep_gpu_memory_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, kbasep_gpu_memory_seq_show, NULL);
}

static int kbasep_gpu_memory_detail_debugfs_open(struct inode *in, struct file *file)
{
	return single_open(file, kbasep_gpu_memory_detail_seq_show, in->i_private);
}
static const struct file_operations kbasep_gpu_memory_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = kbasep_gpu_memory_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations kbasep_gpu_memory_detail_debugfs_fops = {
	.owner = THIS_MODULE,
	.open = kbasep_gpu_memory_detail_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
/*
 *  Initialize debugfs entry for gpu_memory
 */
void kbasep_gpu_memory_debugfs_init(struct kbase_device *kbdev)
{
	debugfs_create_file("gpu_memory", S_IRUGO,
			kbdev->mali_debugfs_directory, NULL,
			&kbasep_gpu_memory_debugfs_fops);
#ifdef MALI_GPU_MEMORY_DEBUGFS
	debugfs_create_file("gpu_memory", S_IRUGO,
			kbdev->mali_debugfs_directory_gpu, NULL,
			&kbasep_gpu_memory_debugfs_fops);
	debugfs_create_file("gpu_memory_detail", S_IRUGO,
			kbdev->mali_debugfs_directory_gpu, NULL,
			&kbasep_gpu_memory_detail_debugfs_fops);
#endif
	return;
}

#else
/*
 * Stub functions for when debugfs is disabled
 */
void kbasep_gpu_memory_debugfs_init(struct kbase_device *kbdev)
{
	return;
}
#endif
