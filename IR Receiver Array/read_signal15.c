#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <mraa/gpio.h>

#define N_REC 6

//anything with N_REC means that there is a copy of the variable for each receiver
static long double time_tolerance = 0.2; //temporal leeway for interpreting biphase mark encoding
static long double half_period = 4000; //half of the interval of the data frequency
int dataIncrementP[N_REC] = {-1,-1,-1,-1,-1,-1}; //indicates which buffer producer is writing to
int dataIncrementC[N_REC] = {-1,-1,-1,-1,-1,-1}; //indicates which buffer consumer is writing to
unsigned char* dataPtr[N_REC]; //producer loop tells IR receiver code which buffer to write to
unsigned char data[3][N_REC][10]; //3 sets of buffers for messages received from 6 receivers

int current_receiver = 0; //which IR receiver is currently being polled by wait_for_next_transition
long double prev_transition[N_REC], current_transition[N_REC]; //timestamps for when beacon edges transition low to high or high to low
int r_fsm[N_REC] = {0}; //each beacon has a state machine for interpreting IR receiver transitions
//0 = waiting for start sequence
//1 = waiting for start sequence end
//2 = waiting for next transition
//3 = high bit waiting for next transition
int r_bit_state[N_REC] = {1}; //stores whether current bit is high or low bit; bits are inverted (0 = high)
int r_prev_bit_state[N_REC] = {0}; //whether last bit is high or low bit
long r_bufPos[N_REC] = {0}; //write position of binary buffer for each receiver, used by wait_for_next_sequence
long readResult[3][N_REC]; //tracks length of all binary buffers
int MRU_buffer[N_REC]={0}; //indicates which buffer was most recently written to; code tries not to write to this buffer


long double dabs(long double val)
{
	if (val < 0.0) return -val;
	else return val;
}

long double wait_for_next_transition(mraa_gpio_context ir_receiver[], int prev_state[], char* debounceBuf, int bufsize, int debounce_thres, int* receiverID)
{
    static int counter[N_REC];
    memset(counter, 0, N_REC*sizeof(int));
    struct timeval currentTime;
    while(1)
    {
        int reading;
        reading=mraa_gpio_read(ir_receiver[current_receiver]);
        if (reading != prev_state[current_receiver])
        counter[current_receiver]++;
        else if (counter[current_receiver] != 0)
        counter[current_receiver]--;
        if (counter[current_receiver] == debounce_thres)
        {
            gettimeofday(&currentTime,NULL);
            *receiverID=current_receiver;
            r_prev_bit_state[current_receiver]=prev_state[current_receiver];
            prev_state[current_receiver]=!prev_state[current_receiver];
            return 1000000.0 * (long double)currentTime.tv_sec + (long double)currentTime.tv_usec;
        }
        current_receiver=(current_receiver+1)%N_REC;
    }
}

long wait_for_next_sequence(mraa_gpio_context ir_receiver[], unsigned char** bitBuf, int bufSize, int* receiverID)
{
    char debounceBuf[8];
    int i;
    for (i=0; i<8; i++) debounceBuf[i] = 1;
    int recTransID;
    long double transition_time;
    int continue_loop=1;
    long rVal;
    //multiple finite state machine loop
    while(continue_loop)
    {
        //printf("wait for transition\n");
        transition_time = wait_for_next_transition(ir_receiver,r_bit_state,debounceBuf,8,5,&recTransID);
        prev_transition[recTransID]=current_transition[recTransID];
        current_transition[recTransID]=transition_time;
        int* r_state=&r_fsm[recTransID];
        switch(*r_state)
        {
        case 0:
            //printf("s\n");
            if (r_prev_bit_state[recTransID] == 1)
            {
                //printf("s\n");
                *r_state=1;
            }
            break;
        case 1:
            //printf("n\n");
            if (dabs(current_transition[recTransID] - prev_transition[recTransID] - 8 * half_period) < 8 * half_period * time_tolerance)
            {
                //printf("n\n");
                *r_state=2;
            }
            else
            {
                *r_state=0;
            }
            break;
        case 2:
            //printf("w\n");
            if (dabs(current_transition[recTransID] - prev_transition[recTransID] - half_period - 100 ) < time_tolerance * half_period * 3.5)
            {
                *r_state=3;
                //printf("h\n");
            }
            else if (current_transition[recTransID] - prev_transition[recTransID] - half_period - 100 < (1 + time_tolerance) * 2 * half_period)
            {
                //printf("f\n");
                if ((r_bufPos[recTransID] & 7) == 0)
                bitBuf[recTransID][r_bufPos[recTransID] >> 3] = 0;
                bitBuf[recTransID][r_bufPos[recTransID] >> 3] = bitBuf[recTransID][r_bufPos[recTransID] >> 3] & ~(1 << (r_bufPos[recTransID] & 7));
                if ((++r_bufPos[recTransID]) >> 3 == bufSize)
                {
                    *receiverID=recTransID;
                    rVal=r_bufPos[recTransID];
                    r_bufPos[recTransID]=0;
                    continue_loop=0;
                }
            }
            else
            {
                //printf("e\n");
                *r_state=0;
                *receiverID=recTransID;
                rVal=r_bufPos[recTransID];
                r_bufPos[recTransID]=0;
                continue_loop=0;
            }
            break;
        case 3:
            if (dabs(current_transition[recTransID] - prev_transition[recTransID] - half_period - 100 ) < time_tolerance * half_period * 3.5)
            {
                *r_state=2;
                //printf("h\n");
                if ((r_bufPos[recTransID] & 7) == 0)
                bitBuf[recTransID][r_bufPos[recTransID] >> 3] = 0;
                bitBuf[recTransID][r_bufPos[recTransID] >> 3] = bitBuf[recTransID][r_bufPos[recTransID] >> 3] | (1 << (r_bufPos[recTransID] & 7));
                if ((++r_bufPos[recTransID]) >> 3 == bufSize)
                {
                    *receiverID=recTransID;
                    rVal=r_bufPos[recTransID];
                    r_bufPos[recTransID]=0;
                    continue_loop=0;
                }
            }
            else
            {
                //printf("c\n");
                //crash?

                *r_state=0;
                r_bufPos[recTransID]=0;
                return -1;
            }
            break;
        }
    }
    return rVal;
}

int main(){
	printf("Program Start\n");
	int expectedSize=32;
	long i;
	int same=1;
	long result;
	int j;
	int recSeqID;
	
	mraa_init();
	
	mraa_gpio_context ir_receiver[N_REC];
    ir_receiver[0] = mraa_gpio_init(25); //gp129
    ir_receiver[1] = mraa_gpio_init(13); //gp128
    ir_receiver[2] = mraa_gpio_init(21); //gp183
    ir_receiver[3] = mraa_gpio_init(31); //gp44
    ir_receiver[4] = mraa_gpio_init(45); //gp45
    ir_receiver[5] = mraa_gpio_init(32); //gp46
	
	for(i = 0; i<N_REC; i++)
	{
		mraa_gpio_dir(ir_receiver[i], MRAA_GPIO_IN);
		dataPtr[i]=data[0][i];
	}
	i = 0;

	while(1)
	{
		printf("Loop Start \n");
		result = wait_for_next_sequence(ir_receiver, dataPtr, 10, &recSeqID);
		
		printf("cr%d\n", recSeqID);
		//print out the number of values
		printf("%ld\n", result);

		if(result == -1){
			continue;
		}
		
		readResult[MRU_buffer[recSeqID]][recSeqID]=result;
		
		int resultID[2]={-1};
		int Nresult=0;
		int nextMRU=-1;
		//compare buffers for a match
		//designate next buffer to write to
		for(j=0; j<3; j++)
		{
			if (dataIncrementC[recSeqID] != j && dataIncrementP[recSeqID] != j) //buffer is eligible for match if producer isn't waiting on consumer to read the buffer
			{
				resultID[Nresult++]=j;
				if (MRU_buffer[recSeqID] != j)
				{
					nextMRU = j;
					dataPtr[recSeqID] = data[j][recSeqID];
				}
			}
		}
		if (Nresult==2) //there are two buffers available for matching
		{
			int RID0=resultID[0];
			int RID1=resultID[1];
			if(readResult[RID0][recSeqID] == readResult[RID1][recSeqID])
			{
				same = 1;
				for (j=0; j!=(readResult[0][recSeqID]/8); j++){
					printf("Rec:%d BufA:%d BufB:%d Pos:%u dataA:%c dataB:%c\n", recSeqID, RID0, RID1, j, data[RID0][recSeqID][j], data[RID1][recSeqID][j]);
					if(data[RID0][recSeqID][j] != data[RID1][recSeqID][j]){
						printf("difference detected\n");
						same=0;
						break;
					}
				}
				if(same == 1){
					if (MRU_buffer[recSeqID]==RID0)
					{
						dataIncrementP[recSeqID]=RID0;
					}
					else
					{
						dataIncrementP[recSeqID]=RID1;
					}
					printf("MATCH: %ld\n", readResult[0][recSeqID]);
					printf("\n");
				}
			}
		}
		MRU_buffer[recSeqID]=nextMRU;
	}
	return 0;
}

