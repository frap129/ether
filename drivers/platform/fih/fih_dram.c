#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define FIH_PROC_DIR   "AllHWList"
#define FIH_PROC_PATH  "AllHWList/draminfo"
#define FIH_PROC_SIZE  32

static char fih_proc_data[FIH_PROC_SIZE] = "unknown";

static void fih_dram_setup(void)
{
	snprintf(fih_proc_data, sizeof(fih_proc_data), "SK hynix 3 GB");
}

static int fih_proc_read_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", fih_proc_data);
	return 0;
}

static int draminfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_proc_read_show, NULL);
};

static struct file_operations draminfo_file_ops = {
	.owner   = THIS_MODULE,
	.open    = draminfo_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int __init fih_proc_init(void)
{
	fih_dram_setup();

	if (proc_create(FIH_PROC_PATH, 0, NULL, &draminfo_file_ops) == NULL) {
		pr_err("fail to create proc/%s\n", FIH_PROC_PATH);
		return (1);
	}
	return (0);
}

static void __exit fih_proc_exit(void)
{
	remove_proc_entry(FIH_PROC_PATH, NULL);
}

module_init(fih_proc_init);
module_exit(fih_proc_exit);
