#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "fih_sd_status.h"

#define FIH_PROC_DIR   "AllHWList"
#define FIH_PROC_PATH  "AllHWList/sd_status"
#define FIH_PROC_SIZE  FIH_SD_STATUS_SIZE

static char fih_proc_data[FIH_PROC_SIZE] = "0";

void fih_sd_status_setup(char *info)
{
        memset(fih_proc_data, 0, sizeof(fih_proc_data));
        snprintf(fih_proc_data, sizeof(fih_proc_data), "%s", info);
}


static int fih_sd_status_read_proc(struct seq_file *m, void *v)
{
        seq_printf(m, "%s\n", fih_proc_data);
        return 0;
}

static int fih_sd_status_open(struct inode *inode, struct file *file)
{
        return single_open(file, fih_sd_status_read_proc, NULL);
}


static const struct file_operations fih_sd_status_operations = {
        .open           = fih_sd_status_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = seq_release,
};


static int __init fih_proc_init(void)
{
//      proc_mkdir(FIH_PROC_DIR, NULL);
        if (proc_create(FIH_PROC_PATH,0644,NULL, &fih_sd_status_operations) == NULL) {
                proc_mkdir(FIH_PROC_DIR, NULL);
                if (proc_create(FIH_PROC_PATH, 0, NULL, &fih_sd_status_operations) == NULL) {
                        pr_err("fail to create proc/%s\n", FIH_PROC_PATH);
                        return (1);
                }
        }
        return (0);
}

static void __exit fih_proc_exit(void)
{
        remove_proc_entry(FIH_PROC_PATH, NULL);
}

module_init(fih_proc_init);
module_exit(fih_proc_exit);
