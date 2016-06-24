all: beacon_sender

beacon_sender: beacon_sender.c
	gcc -lmraa beacon_sender.c -o beacon_sender

install: beacon_sender
	cp startup.sh /etc/init.d/
	chmod +x /etc/init.d/startup.sh
	update-rc.d startup.sh defaults

.PHONY: install