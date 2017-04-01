dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb
dtb-$(CONFIG_ARCH_XGENE) += apm-mustang.dtb

targets += dtbs

#used for cleaning - not building
ifeq ($(CONFIG_MACH_FIH),y)
subdir- := fih
else
subdir- := qcom
endif

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := dts/*.dtb *.dtb
