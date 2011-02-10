
DIRS = fat boot ttc


all:
	for i in $(DIRS); do $(MAKE) -C $$i || exit 1; done

clean:
	for i in $(DIRS); do $(MAKE) -C $$i clean || exit 1; done


depend:
	for i in $(DIRS); do $(MAKE) -C $$i .depend || exit 1; done

.depend:
	for i in $(DIRS); do $(MAKE) -C $$i .depend || exit 1; done
