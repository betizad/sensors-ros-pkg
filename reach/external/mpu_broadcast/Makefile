CC = g++

all:
	$(CC) $(INCLUDES) MPUbroadcast.cpp MPU9250.cpp SPIdev.h -o mpu_broadcast

install: all
	mkdir -p /usr/bin
	mv -f mpu_broadcast /usr/bin/mpu_broadcast
	cp -f mpu_broadcast.service /etc/systemd/system/multi-user.target.wants/
	chmod 664 /etc/systemd/system/multi-user.target.wants/mpu_broadcast.service

clean:
	rm mpu_broadcast
