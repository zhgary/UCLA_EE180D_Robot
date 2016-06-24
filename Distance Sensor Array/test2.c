#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <mraa/pwm.h>
#include <mraa/gpio.h>

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
        if (1000000.0 * (startTime.tv_sec - trigTime.tv_sec) + startTime.tv_usec - trigTime.tv_usec >= 1500000.0)
            return -1.;
    }

    while (mraa_gpio_read(echo) == 1)
    {
        gettimeofday(&endTime, NULL);
        if (1000000.0 * (endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec >= 500000.0)
            return -1.;
    }
    
    time_taken = 1000000.0 * (endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
    distance = (time_taken + 0.00) / 58.82;
    while (time_taken < 30000 && time_taken > 0){
            gettimeofday(&endTime, NULL);
            time_taken = 1000000.0 * (endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
    }
    return distance;
}


int main() {
    mraa_gpio_context trig[5], echo[5];
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
        mraa_gpio_dir(trig[i], MRAA_GPIO_OUT);
        mraa_gpio_dir(echo[i], MRAA_GPIO_IN);
    }
    printf("\n");

    while (1)
    {
        for (i=0;i<5;i++)
        {
            printf(":");
            fflush(stdout);
            double result = get_distance(trig[i], echo[i]);
            printf("%.2f\t", result);
        }
        printf("\n");
        usleep(1000);
    }
}
