#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include "fih_ramtable.h"

enum {
	FIH_BSET_BLOG = 0, /* show bootloader log on screen */
	FIH_BSET_FTM,      /* default enter factory mode */
	FIH_BSET_KEY,      /* no check long press and usb plugin when fih_key_func enable */
	FIH_BSET_KLOG,     /* set kernel log into uart */
	FIH_BSET_RLOG,     /* set recovery log into kernel log */
	FIH_BSET_MAX,      /* checksum */
};

#define FIH_BSET_LENGTH  (FIH_BSET_MAX + 1)

#define FIH_BSET_MEM_ADDR  FIH_BSET_BASE
#define FIH_BSET_MEM_SIZE  FIH_BSET_SIZE /* 16 Byte */

static int fih_bset_proc_read_rlog_show(struct seq_file *m, void *v)
{
	int len;
	char *tmp;
	char buf = 0;

	tmp = (char *)ioremap(FIH_BSET_MEM_ADDR, FIH_BSET_MEM_SIZE);
	if (tmp == NULL) {
		pr_err("%s: ioremap fail\n", __func__);
		return (0);
	}

	/* check bset is good */
	for (len=0; len<FIH_BSET_MAX; len++) {
		buf += tmp[len];
	}
	if (buf != tmp[FIH_BSET_MAX]) {
		memset(tmp, 0x00, FIH_BSET_LENGTH);
	}

	seq_printf(m, "%d\n", tmp[FIH_BSET_RLOG]);

	iounmap(tmp);

	return 0;
}

static int rlog_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_bset_proc_read_rlog_show, NULL);
};

static struct file_operations rlog_file_ops = {
	.owner   = THIS_MODULE,
	.open    = rlog_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int fih_bset_proc_read_all_show(struct seq_file *m, void *v)
{
	int len;
	char *tmp;
	char buf = 0;

	tmp = (char *)ioremap(FIH_BSET_MEM_ADDR, FIH_BSET_MEM_SIZE);
	if (tmp == NULL) {
		pr_err("%s: ioremap fail\n", __func__);
		return (0);
	}

	/* check bset is good */
	for (len=0; len<FIH_BSET_MAX; len++) {
		buf += tmp[len];
	}
	if (buf != tmp[FIH_BSET_MAX]) {
		memset(tmp, 0x00, FIH_BSET_LENGTH);
	}

	for (len=0; len<FIH_BSET_MAX; len++) {
		seq_printf(m, "%d\n", tmp[len]);
	}

	iounmap(tmp);

	return 0;
}

static int all_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_bset_proc_read_all_show, NULL);
};

static struct file_operations all_file_ops = {
	.owner   = THIS_MODULE,
	.open    = all_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static struct {
		char *name;
		struct file_operations *ops;
} *p, fih_bset[] = {
	{"bset/rlog", &rlog_file_ops},
	{"bset/all", &all_file_ops},
	{NULL}, };

static int __init fih_bset_init(void)
{
	if (FIH_BSET_MEM_SIZE < FIH_BSET_LENGTH) {
#if 0
		pr_err("%s: WARNNING!! FIH_BSET_MEM_SIZE (%d) < FIH_BSET_LENGTH (%d)",
			__func__, FIH_BSET_MEM_SIZE, FIH_BSET_LENGTH);
#endif
		return (1);
	}

	proc_mkdir("bset", NULL);
	for (p = fih_bset; p->name; p++) {
		if (proc_create(p->name, 0, NULL, p->ops) == NULL) {
			pr_err("fail to create proc/%s\n", p->name);
		}
	}

	return (0);
}

static void __exit fih_bset_exit(void)
{
	for (p = fih_bset; p->name; p++) {
		remove_proc_entry(p->name, NULL);
	}
}

module_init(fih_bset_init);
module_exit(fih_bset_exit);
