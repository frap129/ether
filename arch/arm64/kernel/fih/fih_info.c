#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <fih/hwid.h>

static int fih_info_proc_read_project_show(struct seq_file *m, void *v)
{
	char msg[8];

	switch (fih_hwid_fetch(FIH_HWID_PRJ)) {
		case FIH_PRJ_NBQ: strcpy(msg, "NBQ"); break;
                case FIH_PRJ_FAG: strcpy(msg, "FAG"); break;
                case FIH_PRJ_VZW: strcpy(msg, "VZW"); break;
		default: strcpy(msg, "N/A"); break;
	}
	seq_printf(m, "%s\n", msg);

	return 0;
}

static int devmodel_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_info_proc_read_project_show, NULL);
};

static struct file_operations devmodel_file_ops = {
	.owner   = THIS_MODULE,
	.open    = devmodel_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int fih_info_proc_read_hw_rev_show(struct seq_file *m, void *v)
{
	char msg[32];

	switch (fih_hwid_fetch(FIH_HWID_REV)) {
		case FIH_REV_EVB:  strcpy(msg, "EVB"); break;
		case FIH_REV_EVB2: strcpy(msg, "EVB2"); break;
		case FIH_REV_EVB3: strcpy(msg, "EVB3"); break;
		case FIH_REV_EVT:  strcpy(msg, "EVT"); break;
                case FIH_REV_EVT_PRE1_5:  strcpy(msg, "PRE_EVT1.5"); break;
                case FIH_REV_EVT1C:  strcpy(msg, "EVT1C"); break;
		case FIH_REV_EVT2: strcpy(msg, "EVT2"); break;
		case FIH_REV_EVT3: strcpy(msg, "EVT3"); break;
		case FIH_REV_DVT:  strcpy(msg, "DVT"); break;
		case FIH_REV_DVT2: strcpy(msg, "DVT2"); break;
		case FIH_REV_DVT3: strcpy(msg, "DVT3"); break;
		case FIH_REV_PVT:  strcpy(msg, "PVT"); break;
		case FIH_REV_PVT2: strcpy(msg, "PVT2"); break;
		case FIH_REV_PVT3: strcpy(msg, "PVT3"); break;
		case FIH_REV_MP:   strcpy(msg, "MP"); break;
		case FIH_REV_MP2:  strcpy(msg, "MP2"); break;
		case FIH_REV_MP3:  strcpy(msg, "MP3"); break;
		default: strcpy(msg, "N/A"); break;
	}
	seq_printf(m, "%s\n", msg);

	return 0;
}
static int baseband_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_info_proc_read_hw_rev_show, NULL);
};

static struct file_operations baseband_file_ops = {
	.owner   = THIS_MODULE,
	.open    = baseband_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int fih_info_proc_read_rf_band_show(struct seq_file *m, void *v)
{
	char msg[192];/* FIH,Jimi,2015/09/09 modify fof log band */

	switch (fih_hwid_fetch(FIH_HWID_RF)) {
		/* GSM + WCDMA */
		case FIH_RF_G_850_900_1700_1900_2100_W_1_2_4_5_8:
			strcpy(msg, "G_850_900_1700_1900_2100^W_1_2_4_5_8"); break;
		case FIH_RF_G_850_1900_2100_W_1_2_5:
			strcpy(msg, "G_850_1900_2100^W_1_2_5"); break;
		case FIH_RF_G_900_2100_W_1_8:
			strcpy(msg, "G_900_2100^W_1_8"); break;
		case FIH_RF_G_850_900_1800_1900_W_1_2_8:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_8"); break;
		case FIH_RF_G_850_900_1800_1900_W_1_2_5_8:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8"); break;
		/* GSM + CDMA + EVDO */
		case FIH_RF_G_850_900_1800_1900_C_0_1_10:
			strcpy(msg, "G_850_900_1800_1900^C_0_1_10"); break;
		case FIH_RF_G_850_900_1800_1900_C_0_1:
			strcpy(msg, "G_850_900_1800_1900^C_0_1"); break;
		case FIH_RF_G_850_900_1800_1900_C_0_6:
			strcpy(msg, "G_850_900_1800_1900^C_0_6"); break;
		/* CDMA + EVDO */
		case FIH_RF_C_0_1_10_CD_1:
			strcpy(msg, "C_0_1_10^CD_1"); break;
		/* CDMA + EVDO + LTE */
		case FIH_RF_C_0_1_10_L_25:
			strcpy(msg, "C_0_1_10^L_25"); break;
		/* GSM + EDGE + UMTS + TD-SCDMA + TD-LTE + FDD-LTE */
		case FIH_RF_G_850_900_1800_1900_W_1_2_5_8_T_34_39_40_L_1_3_7_38_39_40_41_LD_1_3_7_38_39_40_41:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^T_34_39_40^L_1_3_7_38_39_40_41^LD_1_3_7_38_39_40_41"); break;
       /* FIH,Jimi,2015/08/19 add for version bandinfo{*/
#if 0
		/* GSM + TDSCDMA */
		case FIH_RF_G_850_900_1800_1900_T_34_39_40:
			strcpy(msg, "G_850_900_1800_1900^T_34_39_40"); break;
#else
                case FIH_BAND_G_850_900_1800_1900_W_1_2_3_4_5_6_9_C_0_1_19_L_1_2_3_4_5_7_12_17_19_20_28:
                        strcpy(msg, "G_850_900_1800_1900^W_1_2_3_4_5_6_9^C_0_1_19^L_1_2_3_4_5_7_12_17_19_20_28"); break;
                case FIH_BAND_G_850_900_1800_1900_W_1_2_3_4_5_6_8_9_19_C_0_1_L_1_2_3_4_5_7_8_12_17_19_20_28:
                        //strcpy(msg, "FIH_BAND_G_850_900_1800_1900^W_1_2_3_4_5_6_8_9_19^C_0_1^L_1_2_3_4_5_7_8_12_17_19_20_28"); break;
                        strcpy(msg, "G_850_900_1800_1900^W_1_2_3_4_5_6_8_9_19^C_0_1^L_1_2_3_4_5_7_8_12_17_19_20_28^WD_1_2_3_4_5_6_8_9_19^CD_0_1^LD_1_2_3_4_5_7_8_12_17_19_20_28"); break;
#endif
        /* FIH,Jimi,2015/08/19 add for version bandinfo}*/
                case FIH_BAND_G_850_900_1800_1900_W_1_2_3_4_C_0_1_L_1_2_3_4_7_13_20_25_26_41:
                        strcpy(msg, "G_850_900_1800_1900^W_1_2_3_4^C_0_1^L_1_2_3_4_7_13_20_25_26_41"); break;
		/* GSM + WCDMA + TDSCDMA */
		case FIH_RF_G_850_900_1800_1900_W_1_2_5_8_T_34_39_40:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^T_34_39_40"); break;
		/* GSM + EDGE + UMTS + TD-LTE + FDD-LTE + CA */
		case FIH_RF_G_900_1800_1900_W_1_5_8_9_C_1_10_L_1_3_8_25_26_CA_1_8:
			strcpy(msg, "G_900_1800_1900^W_1_5_8_9^C_1_10_L_1_3_8_25_26^CA_1_8"); break;
		case FIH_RF_G_850_900_1800_1900_W_1_2_5_8_9_C_0_1_10_L_1_3_5_8_25_26_41:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8_9^C_0_1_10^L_1_3_5_8_25_26_41"); break;
		/* GSM + WCDMA + TDSCDMA + LTE */
		case FIH_RF_G_900_1800_1900_W_1_2_8_L_3_7_38_39_40_41:
			strcpy(msg, "G_900_1800_1900^W_1_2_8^L_3_7_38_39_40_41"); break;
		case FIH_RF_G_850_1800_1900_W_1_2_5_C_0_1_L_4_25:
			strcpy(msg, "G_850_1800_1900^W_1_2_5^C_0_1^L_4_25"); break;
		/* GSM + WCDMA + TDSCDMA + LTE */
		case FIH_RF_G_850_900_1800_1900_W_1_2_5_8_T_34_39_40_L_1_3_7_8_17_38_39_40_41:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^T_34_39_40^L_1_3_7_8_17_38_39_40_41"); break;
		/* GSM + WCDMA + LTE + SVLTE */
		case FIH_RF_G_850_900_1800_1900_W_1_2_5_8_C_0_L_1_3_7:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^C_0^L_1_3_7"); break;
		case FIH_RF_G_850_900_1800_1900_W_1_2_5_C_0_L_1_3_28A_28B_41:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5^C_0^L_1_3_28A_28B_41"); break;
		case FIH_RF_G_850_900_1800_1900_W_1_2_C_0_L_1_3_8_28A_28B_41:
			strcpy(msg, "G_850_900_1800_1900^W_1_2^C_0^L_1_3_8_28A_28B_41"); break;
		case FIH_RF_G_850_900_1800_1900_W_1_2_5_C_0_L_1_3_41:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5^C_0^L_1_3_41"); break;
		case FIH_RF_G_850_900_1800_1900_W_1_2_C_0_L_1_3_8_28A_28B:
			strcpy(msg, "G_850_900_1800_1900^W_1_2^C_0^L_1_3_8_28A_28B"); break;
		/* GSM + WCDMA + CDMA + LTE + SRLTE */
		case FIH_BAND_G_850_900_1800_1900_W_1_2_5_8_C_0_1_T_34_39_L_1_3_7_8_28_38_39_40_41:
			strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^C_0_1^T_34_39^L_1_3_7_8_28_38_39_40_41"); break;
                /* FIH,Jimi,2014/12/17 add for version bandinfo*/
                case FIH_BAND_G_Q_W_1_2_5_8_T_34_39_L_1_3_7_8_20_28_38_39_40_41_CA:
                        strcpy(msg, "G_850_900_1800_1900^W_1_2_5_8^T_34_39^L_1_3_7_8_20_28_38_39_40_41_CA"); break;
		/* NO BAND */
		case FIH_RF_NONE: strcpy(msg, "NONE"); break;
		default: strcpy(msg, "UNKNOWN\n"); break;
	}
	seq_printf(m, "%s\n", msg);

	return 0;
}
static int bandinfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_info_proc_read_rf_band_show, NULL);
};

static struct file_operations bandinfo_file_ops = {
	.owner   = THIS_MODULE,
	.open    = bandinfo_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static int fih_info_proc_read_hwcfg_show(struct seq_file *m, void *v)
{
	struct st_hwid_table tb;

	fih_hwid_read(&tb);
	/* mpp */
	seq_printf(m, "r1=%d\n", tb.r1);
	seq_printf(m, "r2=%d\n", tb.r2);
	seq_printf(m, "r3=%d\n", tb.r3);
	/* info */
	seq_printf(m, "prj=%d\n", tb.prj);
	seq_printf(m, "rev=%d\n", tb.rev);
	seq_printf(m, "rf=%d\n", tb.rf);
	/* device tree */
	seq_printf(m, "dtm=%d\n", tb.dtm);
	seq_printf(m, "dtn=%d\n", tb.dtn);
	/* driver */
	seq_printf(m, "btn=%d\n", tb.btn);
	seq_printf(m, "uart=%d\n", tb.uart);

	return 0;
}

static int hwcfg_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_info_proc_read_hwcfg_show, NULL);
};

static struct file_operations hwcfg_file_ops = {
	.owner   = THIS_MODULE,
	.open    = hwcfg_proc_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release
};

static struct {
		char *name;
		struct file_operations *ops;
} *p, fih_info[] = {
	{"devmodel", &devmodel_file_ops},
	{"baseband", &baseband_file_ops},
	{"bandinfo", &bandinfo_file_ops},
	{"hwcfg", &hwcfg_file_ops},
	{NULL}, };

static int __init fih_info_init(void)
{
        /* FIH,Jimi,2014/12/02 add for version*/ 
        /* FIH, initial hwid mechanism */
        fih_hwid_setup();

	for (p = fih_info; p->name; p++) {
		if (proc_create(p->name, 0, NULL, p->ops) == NULL) {
			pr_err("fail to create proc/%s\n", p->name);
		}
	}

	return (0);
}

static void __exit fih_info_exit(void)
{
	for (p = fih_info; p->name; p++) {
		remove_proc_entry(p->name, NULL);
	}
}

module_init(fih_info_init);
module_exit(fih_info_exit);
