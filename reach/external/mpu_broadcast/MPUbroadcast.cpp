/*
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Read accelerometer, gyroscope and magnetometer values from
MPU9250 inertial measurement unit over SPI on Raspberry Pi + Navio.

Navio's onboard MPU9250 is connected to the SPI bus on Raspberry Pi
and can be read through /dev/spidev0.1

To run this example navigate to the directory containing it and run following commands:
make
./AccelGyroMag
*/

#include "MPU9250.h"
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

static volatile int runme=1;

void interrupt(int pholder)
{
   runme=0;
}

int main()
{
    signal(SIGINT, interrupt);
    MPU9250 imu;
    imu.initialize();

    float ax, ay, az, gx, gy, gz, mx, my, mz;

    if (imu.doSelfTest())
    {
        printf("MPU9250 self test passed\n");
    } else {
        printf("MPU9250 self test failed");
        return 0;
    }

    int sock, status, sinlen;
    struct sockaddr_in sock_in;
    int yes = 1;

    sinlen = sizeof(struct sockaddr_in);
    memset(&sock_in, 0, sinlen);

    sock = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    sock_in.sin_addr.s_addr = htonl(INADDR_ANY);
    sock_in.sin_port = htons(0);
    sock_in.sin_family = AF_INET;

    status = bind(sock, (struct sockaddr *)&sock_in, sinlen);
    printf("Bind Status = %d\n", status);

    status = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(int) );
    printf("Setsockopt Status = %d\n", status);

    inet_aton("192.168.2.255", &sock_in.sin_addr);
    sock_in.sin_port = htons(35050); /* port number */
    sock_in.sin_family = AF_INET;

    float imudata[9]={0};
    while(runme)
    {
        imu.getMotion9(&imudata[0], &imudata[1], &imudata[2], 
                       &imudata[3], &imudata[4], &imudata[5],
                       &imudata[6], &imudata[7], &imudata[8]);
        //printf("Data: ");
        //for(int i=0; i<9; ++i) printf("%f ",imudata[i]);
        //printf("\n");
        status = sendto(sock, imudata, sizeof(imudata), 0, (struct sockaddr *)&sock_in, sinlen);
        //printf("sendto Status = %d\n", status);
        //sleep(2);
        usleep(8*1000);
    }

    close(sock);

    return 1;
}

