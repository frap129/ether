/*
 * Virtual file interface for FIH LCM, 20151103
 * KuroCHChung@fih-foxconn.com
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#define FIH_PROC_DIR_LCM0 "AllHWList/LCM0"
#define FIH_PROC_PATH_CABC "CABC_settings"
#define FIH_PROC_PATH_CE "color_mode"
#define FIH_PROC_FULL_PATH_CABC "AllHWList/LCM0/CABC_settings"
#define FIH_PROC_FULL_PATH_CE "AllHWList/LCM0/color_mode"

extern int fih_get_cabc (void);
extern int fih_set_cabc (int cabc);
extern int fih_get_ce (void);
extern int fih_set_ce (int ce);

static int fih_lcm_show_cabc_settings(struct seq_file *m, void *v)
{
    seq_printf(m, "%d\n", fih_get_cabc());
    return 0;
}

static int fih_lcm_open_cabc_settings(struct inode *inode, struct  file *file)
{
    return single_open(file, fih_lcm_show_cabc_settings, NULL);
}

static ssize_t fih_lcm_write_cabc_settings(struct file *file, const char __user *buffer,
                    size_t count, loff_t *offp)
{
    char *buf;
    unsigned int res;

    if (count < 1)
        return -EINVAL;

    buf = kmalloc(count, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    if (copy_from_user(buf, buffer, count))
        return -EFAULT;

    res = fih_set_cabc(simple_strtoull(buf, NULL, 0));
    if (res < 0)
    {
        kfree(buf);
        return res;
    }

    kfree(buf);

    /* claim that we wrote everything */
    return count;
}

static struct file_operations cabc_file_ops = {
    .owner   = THIS_MODULE,
    .write   = fih_lcm_write_cabc_settings,
    .read    = seq_read,
    .open    = fih_lcm_open_cabc_settings,
    .release = single_release
};

static int fih_lcm_show_ce_settings(struct seq_file *m, void *v)
{
    seq_printf(m, "%d\n", fih_get_ce());
    return 0;
}

static int fih_lcm_open_ce_settings(struct inode *inode, struct  file *file)
{
    return single_open(file, fih_lcm_show_ce_settings, NULL);
}

static ssize_t fih_lcm_write_ce_settings(struct file *file, const char __user *buffer,
                    size_t count, loff_t *offp)
{
    char *buf;
    unsigned int res;

    if (count < 1)
        return -EINVAL;

    buf = kmalloc(count, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    if (copy_from_user(buf, buffer, count))
        return -EFAULT;

    res = fih_set_ce(simple_strtoull(buf, NULL, 0));
    if (res < 0)
    {
        kfree(buf);
        return res;
    }

    kfree(buf);

    /* claim that we wrote everything */
    return count;
}

static struct file_operations ce_file_ops = {
    .owner   = THIS_MODULE,
    .write   = fih_lcm_write_ce_settings,
    .read    = seq_read,
    .open    = fih_lcm_open_ce_settings,
    .release = single_release
};

static int __init fih_proc_init(void)
{
    struct proc_dir_entry *lcm0_dir;
    lcm0_dir = proc_mkdir (FIH_PROC_DIR_LCM0, NULL);

    if(proc_create(FIH_PROC_PATH_CABC, 0, lcm0_dir, &cabc_file_ops) == NULL)
    {
        pr_err("fail to create proc/%s\n", FIH_PROC_PATH_CABC);
        return (1);
    }

    if(proc_create(FIH_PROC_PATH_CE, 0, lcm0_dir, &ce_file_ops) == NULL)
    {
        pr_err("fail to create proc/%s\n", FIH_PROC_PATH_CE);
        return (1);
    }

    return (0);
}

static void __exit fih_proc_exit(void)
{
    remove_proc_entry(FIH_PROC_FULL_PATH_CABC, NULL);
    remove_proc_entry(FIH_PROC_FULL_PATH_CE, NULL);
}

module_init(fih_proc_init);
module_exit(fih_proc_exit);