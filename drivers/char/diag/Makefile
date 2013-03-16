obj-$(CONFIG_DIAG_CHAR) := diagchar.o
obj-$(CONFIG_DIAG_SDIO_PIPE) += diagfwd_sdio.o
obj-$(CONFIG_DIAG_HSIC_PIPE) += diagfwd_hsic.o
diagchar-objs := diagchar_core.o diagchar_hdlc.o diagfwd.o diagmem.o diagfwd_cntl.o
ifeq ($(CONFIG_HUAWEI_FEATURE_PHUDIAG),y)
diagchar-objs += phudiagchar_core.o phudiagfwd.o
endif
