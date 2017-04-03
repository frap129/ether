#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <fih/share/cda.h>
#include "fih_ramtable.h"

#define FIH_CDA_DATA_ST  struct manuf_data

#define FIH_CDA_ST_ADDR  FIH_CDA_BASE
#define FIH_CDA_ST_SIZE  FIH_CDA_SIZE

static int fih_cda_proc_read_skuid_show(struct seq_file *m, void *v)
{
	FIH_CDA_DATA_ST *cda;

	cda = (FIH_CDA_DATA_ST *)ioremap(FIH_CDA_ST_ADDR, sizeof(FIH_CDA_DATA_ST));
	if (cda == NULL) {
		pr_err("%s: ioremap fail\n", __func__);
		return (0);
	}

	seq_printf(m, "%s\n", cda->SKUflag.SKUID);

	iounmap(cda);

	return 0;
}

static int skuid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_cda_proc_read_skuid_show, NULL);
};

static struct file_operations skuid_file_ops = {
	.owner   = THIS_MODULE,
	.open    = skuid_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static struct {
		char *name;
		struct file_operations *ops;
} *p, fih_cda[] = {
	{"cda/skuid", &skuid_file_ops},
	{NULL}, };

static int __init fih_cda_init(void)
{
	if (FIH_CDA_ST_SIZE < sizeof(FIH_CDA_DATA_ST)) {
#if 0		
/*jimi*/
		pr_err("%s: WARNNING!! FIH_CDA_ST_SIZE (%ld) < sizeof(FIH_CDA_DATA_ST) (%ld)",
			__func__, FIH_CDA_ST_SIZE, sizeof(FIH_CDA_DATA_ST));
#endif
		return (1);
	}

	proc_mkdir("cda", NULL);
	for (p = fih_cda; p->name; p++) {
		if (proc_create(p->name, 0, NULL, p->ops) == NULL) {
			pr_err("fail to create proc/%s\n", p->name);
		}
	}

	return (0);
}

static void __exit fih_cda_exit(void)
{
	for (p = fih_cda; p->name; p++) {
		remove_proc_entry(p->name, NULL);
	}
}

module_init(fih_cda_init);
module_exit(fih_cda_exit);
