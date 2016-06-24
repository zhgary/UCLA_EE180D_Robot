
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <mraa.h>
#include <mraa/pwm.h>
#include <mraa/gpio.h>
#include "LSM9DS0.h"
#include <pthread.h>

union du{
	double d;
	char s[8];
};

float a_res, g_res, m_res;

mraa_gpio_context trig[5];
mraa_gpio_context echo[5];

mraa_uart_context uart;
mraa_i2c_context accel, gyro, mag;

pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

void *print_message_distance_function( void *ptr );
void *print_message_9DOF_function( void *ptr );


double get_distance(mraa_gpio_context trigger, mraa_gpio_context echo)
{
	struct timeval trigTime, startTime, endTime;
	double time_taken;
	double distance;

	gettimeofday(&trigTime, NULL);
	gettimeofday(&startTime, NULL);
	gettimeofday(&endTime, NULL);

	mraa_gpio_write(trigger, 0);
	usleep(5);
	mraa_gpio_write(trigger, 1);
	usleep(11);
	mraa_gpio_write(trigger, 0);
	while (mraa_gpio_read(echo) == 0)
	{
		gettimeofday(&startTime, NULL);
		if (1000000.0 * (startTime.tv_sec - trigTime.tv_sec) + startTime.tv_usec - trigTime.tv_usec >= 100000.0)
		return 5000.;
	}

	while (mraa_gpio_read(echo) == 1)
	{
		gettimeofday(&endTime, NULL);
		if (1000000.0 * (endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec >= 150000.0)
		return 5000.;
	}

	time_taken = 1000000.0 * (endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
	distance = (time_taken + 0.00) / 58.82;
	while (time_taken < 30000 && time_taken > 0){
		gettimeofday(&endTime, NULL);
		time_taken = 1000000.0 * (endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
	}
	return distance;
}



int main()
{
	uart = mraa_uart_init(0);

	if(uart == NULL)
	{
		fprintf(stderr, "UART failed to setup!\n");
		return EXIT_FAILURE;
	}

	mraa_uart_set_mode(uart, 8, MRAA_UART_PARITY_NONE, 1);
	mraa_uart_set_flowcontrol(uart,1,0);

	accel = accel_init();
	set_accel_scale(accel, A_SCALE_2G);
	set_accel_ODR(accel, A_ODR_100);
	a_res = calc_accel_res(A_SCALE_2G);

	gyro = gyro_init();
	set_gyro_scale(gyro, G_SCALE_245DPS);
	set_gyro_ODR(accel, G_ODR_190_BW_70);
	g_res = calc_gyro_res(G_SCALE_245DPS);

	/* 	mag = mag_init();
	set_mag_scale(mag, M_SCALE_2GS);
	set_mag_ODR(mag, M_ODR_125);
	m_res = calc_mag_res(M_SCALE_2GS); */
	int i;
	for (i=0;i<5;i++)
	{
		trig[i]=NULL;
		echo[i]=NULL;
	}

	trig[0] = mraa_gpio_init(32);
	echo[0] = mraa_gpio_init(46);
	trig[1] = mraa_gpio_init(31);
	echo[1] = mraa_gpio_init(45);
	trig[2] = mraa_gpio_init(33);
	echo[2] = mraa_gpio_init(47);
	trig[3] = mraa_gpio_init(25);
	echo[3] = mraa_gpio_init(13);
	trig[4] = mraa_gpio_init(21);
	echo[4] = mraa_gpio_init(00);


	for (i=0;i<5;i++)
	{
		if (trig[i] == NULL || echo[i] == NULL)
		{
			fprintf(stderr, "Initialize failed");
			return 1;
		}
		else
		{
			printf("begin");
		}
		mraa_gpio_dir(trig[i], MRAA_GPIO_OUT);
		mraa_gpio_dir(echo[i], MRAA_GPIO_IN);
	}
	printf("\n");



	pthread_t thread1, thread2;
	const char *message1 = "Thread 1";
	const char *message2 = "Thread 2";
	int  iret1, iret2;

    //create thread 1
	iret1 = pthread_create( &thread1, NULL, print_message_9DOF_function, (void*) message1);
	if(iret1)
	{
		fprintf(stderr,"Error - pthread_create() return code: %d\n",iret1);
		exit(EXIT_FAILURE);
	}

    //create thread 2
	iret2 = pthread_create( &thread2, NULL, print_message_distance_function, (void*) message2);
	if(iret2)
	{
		fprintf(stderr,"Error - pthread_create() return code: %d\n",iret2);
		exit(EXIT_FAILURE);
	}

	printf("pthread_create() for thread 1 returns: %d\n",iret1);
	printf("pthread_create() for thread 2 returns: %d\n",iret2);
	
    //wait for threads to finish, but this will never happen
	pthread_join(thread1, NULL);
	pthread_join(thread2, NULL);

    return 0;
}

void *print_message_9DOF_function( void *ptr )
{
	printf("9DOF thread init\n");

	data_t accel_data, gyro_data, mag_data;
	int16_t temperature;

	while(1) {
		accel_data = read_accel(accel, a_res);
		gyro_data = read_gyro(gyro, g_res);
		//mag_data = read_mag(mag, m_res);
		temperature = read_temp(accel);
		//printf("X: %f\t Y: %f\t Z: %f\t||", accel_data.x, accel_data.y, accel_data.z);
		//printf("\tX: %f\t Y: %f\t Z: %f\t||", gyro_data.x, gyro_data.y, gyro_data.z);
		//printf("\tX: %f\t Y: %f\t Z: %f\t||", mag_data.x, mag_data.y, mag_data.z);
		//printf("\t%ld\n", temperature);

		char buffer[34];
		buffer[0] = 0xFF; //this is a buffer for 9DOF.
		buffer[1] = 0xFF; //this is a buffer for 9DOF.
		buffer[2] = 0xFF; //this is a buffer for 9DOF.
		buffer[3] = 0xFF; //this is a buffer for 9DOF.
		buffer[4] = accel_data.x.s[0];
		buffer[5] = accel_data.x.s[1];
		buffer[6] = accel_data.x.s[2];
		buffer[7] = accel_data.x.s[3];
		buffer[8] = accel_data.y.s[0];
		buffer[9] = accel_data.y.s[1];
		buffer[10] = accel_data.y.s[2];
		buffer[11] = accel_data.y.s[3];
		buffer[12] = accel_data.z.s[0];
		buffer[13] = accel_data.z.s[1];
		buffer[14] = accel_data.z.s[2];
		buffer[15] = accel_data.z.s[3];
		buffer[16] = gyro_data.x.s[0];
		buffer[17] = gyro_data.x.s[1];
		buffer[18] = gyro_data.x.s[2];
		buffer[19] = gyro_data.x.s[3];
		buffer[20] = gyro_data.y.s[0];
		buffer[21] = gyro_data.y.s[1];
		buffer[22] = gyro_data.y.s[2];
		buffer[23] = gyro_data.y.s[3];
		buffer[24] = gyro_data.z.s[0];
		buffer[25] = gyro_data.z.s[1];
		buffer[26] = gyro_data.z.s[2];
		buffer[27] = gyro_data.z.s[3];
		buffer[28] = (temperature & 0xFF);
		buffer[29] = (temperature >> 8) & 0xFF;
		buffer[30] = 0xDE;
		buffer[31] = 0xAD;
		buffer[32] = 0xBE;
		buffer[33] = 0xEF;

		printf("9\n");
		pthread_mutex_lock(&mutex1);
		mraa_uart_write(uart, buffer, sizeof(buffer));
		pthread_mutex_unlock(&mutex1);

		usleep(50000);
	}

	return;
}

void *print_message_distance_function( void *ptr )
{
	printf("Distance thread init\n");
	union du result[5];

	while(1)
	{
		int i;
		for (i=0;i<5;i++)
		{
			//printf(":");
			//result1[i] = result[i];
			result[i].d = get_distance(trig[i], echo[i]);
			//resultA[i] = (result[i] + result1[i])/2;
			//if (resultA[i] >= 0 && resultA[i] < 10) canRun = 0;
			//printf("%.2f\t", resultA[i]);
		}
		char buffer[48];
		buffer[0] = 0xAA; //this is a buffer for distance.
		buffer[1] = 0xAA; //this is a buffer for distance.
		buffer[2] = 0xAA; //this is a buffer for distance.
		buffer[3] = 0xAA; //this is a buffer for distance.
		buffer[4] = result[0].s[0];
		buffer[5] = result[0].s[1];
		buffer[6] = result[0].s[2];
		buffer[7] = result[0].s[3];
		buffer[8] = result[0].s[4];
		buffer[9] = result[0].s[5];
		buffer[10] = result[0].s[6];
		buffer[11] = result[0].s[7];
		buffer[12] = result[1].s[0];
		buffer[13] = result[1].s[1];
		buffer[14] = result[1].s[2];
		buffer[15] = result[1].s[3];
		buffer[16] = result[1].s[4];
		buffer[17] = result[1].s[5];
		buffer[18] = result[1].s[6];
		buffer[19] = result[1].s[7];
		buffer[20] = result[2].s[0];
		buffer[21] = result[2].s[1];
		buffer[22] = result[2].s[2];
		buffer[23] = result[2].s[3];
		buffer[24] = result[2].s[4];
		buffer[25] = result[2].s[5];
		buffer[26] = result[2].s[6];
		buffer[27] = result[2].s[7];
		buffer[28] = result[3].s[0];
		buffer[29] = result[3].s[1];
		buffer[30] = result[3].s[2];
		buffer[31] = result[3].s[3];
		buffer[32] = result[3].s[4];
		buffer[33] = result[3].s[5];
		buffer[34] = result[3].s[6];
		buffer[35] = result[3].s[7];
		buffer[36] = result[4].s[0];
		buffer[37] = result[4].s[1];
		buffer[38] = result[4].s[2];
		buffer[39] = result[4].s[3];
		buffer[40] = result[4].s[4];
		buffer[41] = result[4].s[5];
		buffer[42] = result[4].s[6];
		buffer[43] = result[4].s[7];
		buffer[44] = 0xDE;
		buffer[45] = 0xAD;
		buffer[46] = 0xBE;
		buffer[47] = 0xEF;

		printf("Distance\n");
		pthread_mutex_lock(&mutex1);
		mraa_uart_write(uart, buffer, sizeof(buffer));
		pthread_mutex_unlock(&mutex1);


		//usleep(50000);
		//   fflush(0);
	}

}


