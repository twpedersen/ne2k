
DIRS = kernel tools
ALL_DIRS = $(DIRS) test

FATFORMAT    = tools/fat/fatformat
FATCOPY      = tools/fat/fatcopy
FATDEL       = tools/fat/fatdel
FATSYS       = tools/fat/fatsys
LOADER       = tools/loader/load.exe
DISK_IMAGE   = image/disk_image
KERNEL_IMG   = tos.img
BOOT_STAGE_1 = tools/boot/stage1.bin
BOOT_STAGE_2 = tools/boot/stage2.bin
DEMO_IMG     = image/demo_tos.img


all:
	for i in $(DIRS); do $(MAKE) -C $$i || exit 1; done
	rm -f $(DISK_IMAGE)
	$(FATFORMAT) $(DISK_IMAGE) size=DISC
	$(FATCOPY) $(DISK_IMAGE) $(BOOT_STAGE_2) /`basename $(BOOT_STAGE_2)`
	$(FATCOPY) $(DISK_IMAGE) $(KERNEL_IMG) /`basename $(KERNEL_IMG)`
	$(FATSYS)  $(DISK_IMAGE) $(BOOT_STAGE_1)


demo:
	$(MAKE) -C tools
	rm -f $(DISK_IMAGE)
	$(FATFORMAT) $(DISK_IMAGE) size=DISC
	$(FATCOPY) $(DISK_IMAGE) $(BOOT_STAGE_2) /`basename $(BOOT_STAGE_2)`
	$(FATCOPY) $(DISK_IMAGE) $(DEMO_IMG) /tos.img
	$(FATSYS)  $(DISK_IMAGE) $(BOOT_STAGE_1)

tests frame-test run_ref:
	$(MAKE) -C kernel
	$(MAKE) -C test $@
	$(MAKE) -C tools
	rm -f $(DISK_IMAGE)
	$(FATFORMAT) $(DISK_IMAGE) size=DISC
	$(FATCOPY) $(DISK_IMAGE) $(BOOT_STAGE_2) /`basename $(BOOT_STAGE_2)`
	$(FATCOPY) $(DISK_IMAGE) $(KERNEL_IMG) /`basename $(KERNEL_IMG)`
	$(FATSYS)  $(DISK_IMAGE) $(BOOT_STAGE_1)

host-tests:
	$(MAKE) -C test $@

clean-kernel:
	$(MAKE) -C kernel clean
	$(MAKE) -C test clean
	rm -f $(DISK_IMAGE) $(KERNEL_IMG)

clean:
	for i in $(ALL_DIRS); do $(MAKE) -C $$i clean || exit 1; done
	rm -f $(DISK_IMAGE)
	rm -f image/tos.img
	rm -f $(KERNEL_IMG)
	rm -f ttc.jar

depend:
	for i in $(DIRS); do $(MAKE) -C $$i .depend || exit 1; done

.depend:
	for i in $(DIRS); do $(MAKE) -C $$i .depend || exit 1; done


