ifeq ($(CONFIG_OF),y)

dtb-$(CONFIG_MACH_FIH_NBQ) += \
	msm8992_nbq_evt.dtb \
	msm8992_nbq_evt15.dtb \
	msm8992_nbq_evt1c.dtb \
	msm8992_nbq_dvt.dtb \
	msm8992_nbq_dvt2.dtb \
	msm8992_nbq_pvt.dtb

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

targets += dtbs
targets += $(addprefix ../, $(DTB_LIST))
endif

$(obj)/../%.dtb: $(src)/%.dts FORCE
	$(call if_changed_dep,dtc)

dtbs: $(addprefix $(obj)/../,$(DTB_LIST))

clean-files := *.dtb
