#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <mraa.h>
#include <mraa/gpio.h>

union u{
	float f;
	char s[4];
};

union du{
	double d;
	char s[8];
};

int main(){
	printf("Program begin\n");
	mraa_uart_context uart;
	uart = mraa_uart_init(0);

	if (uart == NULL) {
		fprintf(stderr, "UART failed to setup\n");
		return EXIT_FAILURE;
	}
	mraa_uart_set_mode(uart, 8, MRAA_UART_PARITY_NONE, 1);
	mraa_uart_set_flowcontrol(uart, 1, 0);

	unsigned char buffer[150];
	int bufferSize = 150;
	unsigned char* bufferPosition;
	unsigned char* bufferEnd;
	int dataAmount=0;

	unsigned char stuff[50];
	unsigned char* stuffPosition=stuff;
	unsigned char* stuffEnd=stuff+50;

	int state=0;
	int type=0;

	int dataAvailable = 0;
	int i, j;
	int counter1, counter2, counter3;
	counter1=0;
	counter2=0;
	counter3=0;
	while(1)
	{
		//printf("loop\n");
		dataAvailable = mraa_uart_data_available(uart, 0);
		if(dataAvailable == 1)
		{
			dataAmount = mraa_uart_read(uart, buffer, bufferSize);
			bufferEnd = buffer+dataAmount;
			printf("Read %d bytes:", dataAmount);
			for(bufferPosition=buffer;bufferPosition<bufferEnd;bufferPosition++)
			{
				*bufferPosition = *bufferPosition & 0xFF;
				printf(" %02X", (unsigned int) *bufferPosition);
				if(state == 0){
					if(*bufferPosition==0xFF){
						counter1++;
						counter2=0;
						printf("(%d)", counter1);
					}
					else if(*bufferPosition==0xAA){
						counter1=0;
						counter2++;
						printf("(%d)", counter2);
					}
					else{
						counter1=0;
						counter2=0;
					}

					if(counter1==4){
						printf("G");
						counter3=0;
						state=1;
						type=1;
					}
					if(counter2==4){
						printf("D");
						counter3=0;
						state=1;
						type=2;
					}
				}
				else
				{
					switch(counter3)
					{
						case 0:
							if (*bufferPosition==0xDE)
							{
								counter3=1;
							}
							break;
						case 1:
							if (*bufferPosition==0xAD)
							{
								counter3=2;
							}
							break;
						case 2:
							if (*bufferPosition==0xBE)
							{
								counter3=3;
							}
							break;
						case 3:
							if (*bufferPosition==0xEF)
							{
								state=2;
								counter3=0;
								counter1=0;
								counter2=0;
								//stop reading crap
							}
							break;
					}
					*stuffPosition++ = *bufferPosition;
				}
				if(state == 2){
					printf("message was %d bytes long\n",stuffPosition-stuff);
					if (type==1){
						union u temp;
						int16_t tempint;
						for(i = 0; i<4; i++)
							temp.s[i] = stuff[i];
						printf("Accelerometer x: %f\n", temp.f);

						for(i = 4; i<8; i++)
							temp.s[i-4] = stuff[i];
						printf("Accelerometer y: %f\n", temp.f);

						for(i = 8; i<12; i++)
							temp.s[i-8] = stuff[i];
						printf("Accelerometer z: %f\n", temp.f);

						for(i = 12; i<16; i++)
							temp.s[i-12] = stuff[i];
						printf("Gyroscope x: %f\n", temp.f);

						for(i = 16; i<20; i++)
							temp.s[i-16] = stuff[i];
						printf("Gyroscope y: %f\n", temp.f);

						for(i = 20; i<24; i++)
							temp.s[i-20] = stuff[i];
						printf("Gyroscope z: %f\n", temp.f);

						tempint = stuff[24] & 0xFF;
						tempint = tempint | (stuff[25] < 8);
						//temp.s[i-25] = buffer[i];
						printf("Temperature: %d\n\n", tempint);
					}
					else{
						union du temp;
						for(i=0; i<8; i++)
							temp.s[i] = stuff[i];
						printf("Distance 90L: %f\n", temp.d);
						for(i=8; i<16; i++)
							temp.s[i-8] = stuff[i];
						printf("Distance 45L: %f\n", temp.d);
						for(i=16; i<24; i++)
							temp.s[i-16] = stuff[i];
						printf("Distance 00M: %f\n", temp.d);
						for(i=24; i<32; i++)
							temp.s[i-24] = stuff[i];
						printf("Distance 45R: %f\n", temp.d);
						for(i=32; i<40; i++)
							temp.s[i-32] = stuff[i];
						printf("Distance 90R: %f\n", temp.d);
						printf("\n");
					}
					type=0;
					state=0;
					stuffPosition=stuff;
				}
			}
			printf("\n");
		}
	}
}


