#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <stdint.h>
#include <mraa.h>
#include <mraa/pwm.h>
#include <mraa/gpio.h>
#include <stdlib.h>
#include <poll.h>
#include <pthread.h>
#include "lego_robot.h"
#include "paths.h"

#define N_REC 6


//unions for converting bytes to float or double
union u{
    float f;
    char s[4];
};

union du{
    double d;
    char s[8];
};

//input state and current state of robot
static int stateI;
static int stateC;

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
int r_resultID[2]={-1}; //for each receiver, a temporary variable to select two buffers of the set of three for matching comparison
double r_lastTimeP[N_REC]={0}; //timestamp for most recently received message
double r_lastTimeC[N_REC]={0}; //timestamp for most recently read message

double motor_reset_timestamp = 0;
const double motor_reset_period_u = 100000;
const double motor_reset_duty_u = 60000;

//nodeStruct* nodeDB[10] = {0};
int numNodes = 0;

//mraa contexts
mraa_gpio_context ir_receiver[N_REC]; // use gpio 44 45 46 183 182 129 (128 too if needed)
mraa_uart_context uart;

int maze[9]={0, 0, 0, 0, 1, 0, 0, 1, 0};
int mazeIndex=0;
int totalTurns=9;
char currentBeacon=0;

FILE * fp;
void open_file(){
    fp = fopen("timeData.txt", "a+");
}

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
        transition_time = wait_for_next_transition(ir_receiver,r_bit_state,debounceBuf,8,5,&recTransID);
        prev_transition[recTransID]=current_transition[recTransID];
        current_transition[recTransID]=transition_time;
        int* r_state=&r_fsm[recTransID];
        switch(*r_state)
        {
        case 0:
            if (r_prev_bit_state[recTransID] == 1)
            {
                *r_state=1;
            }
            break;
        case 1:
            if (dabs(current_transition[recTransID] - prev_transition[recTransID] - 8 * half_period) < 8 * half_period * time_tolerance)
            {
                *r_state=2;
            }
            else
            {
                *r_state=0;
            }
            break;
        case 2:
            if (dabs(current_transition[recTransID] - prev_transition[recTransID] - half_period - 100 ) < time_tolerance * half_period * 3.5)
            {
                *r_state=3;
            }
            else if (current_transition[recTransID] - prev_transition[recTransID] - half_period - 100 < (1 + time_tolerance) * 2 * half_period)
            {
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
                *r_state=0;
                r_bufPos[recTransID]=0;
                return -1;
            }
            break;
        }
    }
    return rVal;
}

void *receiverLoop(void *ptr)
{
    int expectedSize=32;
    long i;
    int same=1;
    long result;
    int j;
    int recSeqID;

    while(1)
    {
        result = wait_for_next_sequence(ir_receiver, dataPtr, 10, &recSeqID);

        if(result == -1){
            continue;
        }

        readResult[MRU_buffer[recSeqID]][recSeqID]=result;

        r_resultID[0]=-1;
        r_resultID[1]=-1;
        int Nresult=0;
        int nextMRU=-1;
        //compare buffers for a match
        //designate next buffer to write to
        for(j=0; j<3; j++)
        {
            if (dataIncrementC[recSeqID] != j && dataIncrementP[recSeqID] != j) //buffer is elegible for match if producer isn't waiting on consumer to read the buffer
            {
                r_resultID[Nresult]=j;
                Nresult++;
                if (MRU_buffer[recSeqID] != j)
                {
                    nextMRU = j;
                    dataPtr[recSeqID] = data[j][recSeqID];
                }
                if (Nresult == 2)
                break;
            }
        }
        if (Nresult==2) //there are two buffers available for matching
        {
            int RID0=r_resultID[0];
            int RID1=r_resultID[1];
            if(readResult[RID0][recSeqID] == readResult[RID1][recSeqID])
            {
                same = 1;
                for (j=0; j!=(readResult[0][recSeqID]/8); j++){
                    if(data[RID0][recSeqID][j] != data[RID1][recSeqID][j]){
                        //printf("difference detected\n");
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
                    struct timeval r_currentTime;
                    gettimeofday(&r_currentTime, NULL);
                    r_lastTimeP[recSeqID]=1000000.0 * r_currentTime.tv_sec + r_currentTime.tv_usec;
                    //printf("M%d,%d\n", dataIncrementP[recSeqID], nextMRU);
                    //printf("MATCH: %ld\n", readResult[0][recSeqID]);
                    //printf("\n");
                }
            }
        }
        MRU_buffer[recSeqID]=nextMRU;
    }

}

void *controlFunc(void *ptr)
{
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

    struct timeval currentTime;
    double cTime;

    double d_time[4]; //distance sensor timestamp, past 4 occurrences
    double i_time; //9dof sensor timestamp
    double s_time; //FSM timestamp

    union u accelData[3];
    union u gyroData[3];
    union du distData[4][5];

    int diffC = 0;
    int cDS[4] = {0};
    char dDetected[4];
    char inRange;

    counter1=0;
    counter2=0;
    counter3=0;

    int lastDir = 0;

    unsigned char IRData[N_REC][10];
    int IRDataLen[N_REC];

    int diffR = 0;
    unsigned char IRRecent[8][10];
    int IRRecentID[8];
    int IRRecentLen[8];
    double IRTime[8];
    int rIR[8] = {0};

    /*
    pathStruct* currentPath = NULL;
    nodeStruct* currentNode = NULL;
    nodeStruct* previousNode = NULL;

    nodeGate* currNodeEntryGate = NULL;
    nodeGate* prevNodeExitGate = NULL;
    nodeGate* prevNodeEntryGate = NULL;
    */

    int gateAdvance = 0;

    int nodeWallDetect[2] = {0};
    double wallDetectDebounceTime = -1;

    while(1)
    {

        //printf("loop\n");
        dataAvailable = mraa_uart_data_available(uart, 0);
        int tempRecent[N_REC] = {-1,-1,-1,-1,-1,-1};
        int tempRecentSlot = -1;
        for (i=0;i<N_REC;i++)
        {
            if (r_lastTimeC[i] != r_lastTimeP[i])
            {
                dataIncrementC[i]=dataIncrementP[i];
                printf("Receivers:%c%c%c%c%c%c, Timestamp:%f\nMessage:",i==0 ? '*' : '-',i==1 ? '*' : '-',i==2 ? '*' : '-',i==3 ? '*' : '-',i==4 ? '*' : '-',i==5 ? '*' : '-', r_lastTimeP[i]);
                for (j=0; j <readResult[dataIncrementC[i]][i]/8; j++){
                    printf("%c", data[dataIncrementC[i]][i][j]);
                    IRData[i][j] = data[dataIncrementC[i]][i][j];
                }
                IRDataLen[i] = readResult[dataIncrementC[i]][i];
                printf("\n");
                dataIncrementC[i]=-1;
                r_lastTimeC[i] = r_lastTimeP[i];

                for (j=0; j<N_REC; j++)
                {
                    if (tempRecent[j] == -1)
                    {
                        break;
                    }
                    else if (r_lastTimeC[i] > r_lastTimeC[tempRecent[j]])
                    {
                        break;
                    }
                }
                tempRecentSlot = j;
                j = j+1;
                for (; j<N_REC;j++)
                {
                    int temp = tempRecent[j];
                    tempRecent[j] = tempRecent[j-1];
                    if (temp == -1)
                    {
                        break;
                    }
                }
                tempRecent[tempRecentSlot] = i;
            }
        }

        for (i=0;i<N_REC;i++)
        {
            if (tempRecent[i] == -1)
            {
                break;
            }
            else
            {
                for (j=7;j>0;j--)
                {
                    rIR[j]=rIR[j-1];
                }
                rIR[0] = (rIR[0] + 1) % 8;
                IRRecentLen[rIR[0]] = IRDataLen[tempRecent[i]];
                IRTime[rIR[0]] = r_lastTimeC[tempRecent[i]];
                IRRecentID[rIR[0]] = tempRecent[i];
                for (j=0; j <IRRecentLen[rIR[0]]/8; j++)
                {
                    IRRecent[rIR[0]][j] = IRData[tempRecent[i]][j];
                }
            }
        }

        if (dataAvailable == 1)
        {
            dataAmount = mraa_uart_read(uart, buffer, bufferSize);
            bufferEnd = buffer+dataAmount;
            for(bufferPosition=buffer;bufferPosition<bufferEnd;bufferPosition++)
            {
                *bufferPosition = *bufferPosition & 0xFF;
                if(state == 0){
                    if(*bufferPosition==0xFF){
                        counter1++;
                        counter2=0;
                    }
                    else if(*bufferPosition==0xAA){
                        counter1=0;
                        counter2++;
                    }
                    else{
                        counter1=0;
                        counter2=0;
                    }

                    if(counter1==4){
                        counter3=0;
                        state=1;
                        type=1;
                    }
                    if(counter2==4){
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
                        if (*bufferPosition==0xDE){counter3=1;}
                        break;
                    case 1:
                        if (*bufferPosition==0xAD){counter3=2;}
                        break;
                    case 2:
                        if (*bufferPosition==0xBE){counter3=3;}
                        break;
                    case 3:
                        if (*bufferPosition==0xEF)
                        {
                            state=2;
                            counter3=0;
                            counter1=0;
                            counter2=0;
                            //stop reading
                        }
                        break;
                    }
                    *stuffPosition++ = *bufferPosition; //copy char from buffer to stuff
                }
                if(state == 2){
                    if (type==1){
                        gettimeofday(&currentTime, NULL);
                        i_time = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec; //timestamp
                        // int16_t tempint;
                        for(i = 0; i<4; i++) //accel x
                        accelData[0].s[i] = stuff[i];
                        for(i = 4; i<8; i++) //accel y
                        accelData[1].s[i-4] = stuff[i];
                        for(i = 8; i<12; i++) //accel z
                        accelData[2].s[i-8] = stuff[i];
                        for(i = 12; i<16; i++) //gyro x
                        gyroData[0].s[i-12] = stuff[i];
                        for(i = 16; i<20; i++) //gyro y
                        gyroData[1].s[i-16] = stuff[i];
                        for(i = 20; i<24; i++) //gyro z
                        gyroData[2].s[i-20] = stuff[i];
                    }
                    else{
                        for (i=1;i<4;i++)
                        {
                            cDS[i]=cDS[i-1];
                        }
                        cDS[0] = (cDS[0] + 1) % 4;
                        gettimeofday(&currentTime, NULL);
                        d_time[cDS[0]] = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec; //timestamp
                        union du temp;
                        dDetected[cDS[0]] = 0;
                        inRange = 0;
                        for(i=0; i<8; i++)
                        distData[cDS[0]][0].s[i] = stuff[i];
                        inRange = inRange | (distData[cDS[0]][0].d < 20 && distData[cDS[0]][0].d > 0);
                        dDetected[cDS[0]] = dDetected[cDS[0]] | (distData[cDS[0]][0].d < 1000 && distData[cDS[0]][0].d > 0);
                        for(i=8; i<16; i++)
                        distData[cDS[0]][1].s[i-8] = stuff[i];
                        inRange = inRange | (distData[cDS[0]][1].d < 20 && distData[cDS[0]][1].d > 0) << 1;
                        dDetected[cDS[0]] = dDetected[cDS[0]] | (distData[cDS[0]][1].d < 1000 && distData[cDS[0]][1].d > 0) << 1;
                        for(i=16; i<24; i++)
                        distData[cDS[0]][2].s[i-16] = stuff[i];
                        inRange = inRange | (distData[cDS[0]][2].d < 20 && distData[cDS[0]][2].d > 0) << 2;
                        dDetected[cDS[0]] = dDetected[cDS[0]] | (distData[cDS[0]][2].d < 1000 && distData[cDS[0]][2].d > 0) << 2;
                        for(i=24; i<32; i++)
                        distData[cDS[0]][3].s[i-24] = stuff[i];
                        inRange = inRange | (distData[cDS[0]][3].d < 20 && distData[cDS[0]][3].d > 0) << 3;
                        dDetected[cDS[0]] = dDetected[cDS[0]] | (distData[cDS[0]][3].d < 1000 && distData[cDS[0]][3].d > 0) << 3;
                        for(i=32; i<40; i++)
                        distData[cDS[0]][4].s[i-32] = stuff[i];
                        inRange = inRange | (distData[cDS[0]][4].d < 20 && distData[cDS[0]][4].d > 0) << 4;
                        dDetected[cDS[0]] = dDetected[cDS[0]] | (distData[cDS[0]][4].d < 1000 && distData[cDS[0]][4].d > 0) << 4;
                        //printf("\n");
                    }
                    type=0;
                    state=0;
                    stuffPosition=stuff;
                }
            }
            //printf("\n");
        }

        if (stateI != 0)
        {
            stateC = stateI;
            stateI = 0;
            gettimeofday(&currentTime, NULL);
            s_time = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
        }
        switch (stateC)
        {
        case -1:
            set_A(0);
            set_B(0);
            stateC = 0;
        case 1:
            gettimeofday(&currentTime, NULL);
            cTime = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
            if (cTime - s_time > 1000000.0)
            {
                set_A(0);
                set_B(0);
                stateC = 0;
            }
            else
            {
                set_A(0.5);
                set_B(0.5);
            }
            break;
        case 2:
            gettimeofday(&currentTime, NULL);
            cTime = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
            if (cTime - s_time > 1000000.0)
            {
                set_A(0);
                set_B(0);
                stateC = 0;
            }
            else
            {
                set_A(-0.5);
                set_B(-0.5);
            }
            break;
        case 3:
            gettimeofday(&currentTime, NULL);
            cTime = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
            if (cTime - s_time > 1000000.0)
            {
                set_A(0);
                set_B(0);
                stateC = 0;
            }
            else
            {
                set_A(-1);
                set_B( 1);
            }
            break;
        case 4:
            gettimeofday(&currentTime, NULL);
            cTime = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
            if (cTime - s_time > 1000000.0)
            {
                set_A(0);
                set_B(0);
                stateC = 0;
            }
            else
            {
                set_A(1);
                set_B(-1);
            }
            break;
        case 5:
        case 6:
            gettimeofday(&currentTime, NULL);
            cTime = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
            if (cTime - i_time > 1500000.0 || cTime - d_time[cDS[0]] > 1500000.0)
            {
                printf("end\n");
                set_A(0);
                set_B(0);
                stateC = 0;
            }
            else
            {
                float A = 0;
                float B = 0;
                float deriv = 0;
                float dir;
                if ((dDetected[cDS[0]] & 0b10001) && (dDetected[cDS[1]] & 0b10001))
                {
                    float dPos = (distData[cDS[0]][0].d - distData[cDS[0]][1].d) - (distData[cDS[1]][0].d - distData[cDS[1]][1].d);
                    float dTime = d_time[cDS[0]] - d_time[cDS[1]];

                    deriv = dPos/dTime;
                }

                int goToBeaconMode = -1;
                if (distData[cDS[0]][0].d < 10 && distData[cDS[0]][0].d > 0)
                {
                    if (r_lastTimeC[0] > d_time[cDS[0]])
                    {
                        printf("beacon left\n");
                        goToBeaconMode = 0;
                    }
                }
                if (distData[cDS[0]][1].d < 10 && distData[cDS[0]][1].d > 0)
                {
                    if (r_lastTimeC[0] > d_time[cDS[0]])
                    {
                        printf("beacon left front left\n");
                        goToBeaconMode = 0;
                    }
                }
                if (distData[cDS[0]][2].d < 10 && distData[cDS[0]][2].d > 0)
                {
                    if (r_lastTimeC[5] > d_time[cDS[0]])
                    {
                        printf("beacon front\n");
                        goToBeaconMode = 5;
                    }
                }
                if (distData[cDS[0]][3].d < 10 && distData[cDS[0]][3].d > 0)
                {
                    if (r_lastTimeC[4] > d_time[cDS[0]])
                    {
                        printf("beacon right front\n"); 
                        goToBeaconMode = 4;
                    }
                }
                if (distData[cDS[0]][4].d < 10 && distData[cDS[0]][4].d > 0)
                {
                    if (r_lastTimeC[3] > d_time[cDS[0]])
                    {
                        printf("beacon right\n");
                        goToBeaconMode = 3;
                    }
                }

                if (goToBeaconMode != -1) //transition to beacon mode
                {
                  //print the current beacon and save the time

                  currentBeacon = ((IRRecent[rIR[0]][0] & 0xF0) >> 4);
                  gettimeofday(&currentTime, NULL);
                  cTime = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
                  double pathTime=(cTime - s_time)/1000000.0;
                  fprintf(fp, "Destination Beacon: %d, pathTime: %f\n", currentBeacon, pathTime);
                  fflush(fp);

                    char* beaconMessage = IRData[goToBeaconMode];
                    int beaconMessageLen = IRDataLen[goToBeaconMode];
                    if (beaconMessageLen == 8)
                    {
                        /*
                        int n_gates = (int)((*beaconMessage) & 0xF);
                        int b_index = -1;
                        for (i=0;i<numNodes;i++) //fetch node from list
                        {
                            if (nodeDB[i]->broadcast == *beaconMessage)
                            {
                                b_index = i;
                            }
                        }
                        if (b_index == -1) //make new node if not found in list
                        {
                            b_index = i;
                            nodeDB[b_index] = newNode(*beaconMessage, n_gates);
                        }
                        currentNode = nodeDB[b_index];
                        if (currentPath != NULL) //if we are currently characterizing a path, connect it with the node;
                        {
                            currNodeEntryGate = finishPath(&currentPath,currentNode);
                            if (currentPath == NULL) //path got discarded because a connection already exists
                            {
                                nodeGate* prevGate;
                                nodeGate* prevNodeExitGateCW = prevNodeExitGate->cwGate;

                                currentPath = currNodeEntryGate->path;
                                if (currentPath->A == currNodeEntryGate)
                                {
                                    prevGate = currentPath->B;
                                }
                                else if (currentPath->B == currNodeEntryGate)
                                {
                                    prevGate = currentPath->A;
                                }
                                else
                                {
                                    printf("this should never happen/n");
                                    exit(1);
                                }

                                if (prevGate->ccGate != NULL)
                                {
                                    printf("this should never happen/n");
                                    exit(1);
                                }

                                prevNodeExitGateCC->cwGate = prevGate;
                                prevGate->ccGate = prevNodeExitGateCC;

                                previousNode->gateCon[prevNodeExitGate->conNum] = NULL;
                                previousNode->u_gates--;
                                prevNodeExitGate.path = NULL;
                                prevNodeExitGate.ccGate = NULL;
                                prevNodeExitGate.conNum = -1;
                            }
                        }

                        gateAdvance = 0;

                        autoComplete(previousNode);
                        if (autoComplete(currentNode))
                        {
                            //Dijkstra's algorithm to get to next node?
                            for (i=0;i<numNodes;i++)
                            {
                                if (!nodeDB[i]->complete)
                                {
                                }
                            }
                        }
                        else
                        {
                            nodeGate* tempGate;
                            nodeGate* prevGate;
                            prevGate = currNodeEntryGate;
                            tempGate = currNodeEntryGate->ccGate;
                            while (tempGate != NULL && tempGate != currNodeEntryGate)
                            {
                                gateAdvance++
                                prevGate = tempGate;
                                tempGate = tempGate->ccGate;
                            }
                            if (gateAdvance+1 >= currentNode->n_gates)
                            {
                                printf("this should never happen/n");
                                exit(1);
                            }

                        }*/

                        if(mazeIndex == totalTurns)
                        {
                            printf("We are done!\n");
                            stateC=0;
                        }
                        else{
                            gateAdvance = maze[mazeIndex];
                            mazeIndex++;
                            nodeWallDetect[0] = 0;
                            nodeWallDetect[1] = 0;
                            gettimeofday(&currentTime, NULL);
                            s_time = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
                            stateC = 7;
                        }
                    }
                }

                switch (inRange)
                {
                case 0b10001:
                    dir = (distData[cDS[0]][0].d - distData[cDS[0]][4].d)/20. - 100 * deriv;
                    if (dir > 1) dir = 1;
                    else if (dir < -1) dir = -1;

                    if (dir > 0)
                    {
                        A = 0.95*(1-dir);
                        B = 0.95;
                    }
                    else
                    {
                        A = 0.95;
                        B = 0.95*(1+dir);
                    }
                    break;
                case 0b01101:
                case 0b11101:
                case 0b11100:
                case 0b11000:
                case 0b01000:
                case 0b01100:
                    if (distData[cDS[0]][3].d < 10)
                    {
                        A = distData[cDS[0]][3].d < 3 ? -1 : -1.3+distData[cDS[0]][3].d/10;
                    }
                    else
                    {
                        A = 0;
                    }
                    B = 1;
                    break;
                case 0b10110:
                case 0b10111:
                case 0b00111:
                case 0b00011:
                case 0b00010:
                case 0b00110:
                    A = 1;
                    if (distData[cDS[0]][1].d < 10)
                    {
                        B = distData[cDS[0]][1].d < 3 ? -1 : -1.3+distData[cDS[0]][1].d/10;
                    }
                    else
                    {
                        B = 0;
                    }
                    break;

                case 0b11001:
                case 0b01001:
                    if (distData[cDS[0]][0].d > distData[cDS[0]][3].d)
                    {
                        A = 0;
                        B = 1;
                    }
                    else
                    {
                        A = 0.95;
                        B = 0.95;
                    }
                    break;
                case 0b10011:
                case 0b10010:
                    if (distData[cDS[0]][4].d > distData[cDS[0]][1].d)
                    {
                        A = 1;
                        B = 0;
                    }
                    else
                    {
                        A = 0.95;
                        B = 0.95;
                    }
                    break;

                case 0b10100:
                case 0b00101:
                case 0b10000:
                case 0b00001:
                    if (distData[cDS[0]][0].d > distData[cDS[0]][4].d)
                    {
                        A = 0;
                        B = 1;
                    }
                    else
                    {
                        A = 1;
                        B = 0;
                    }
                    break;
                case 0b11011:
                case 0b01010:
                case 0b11010:
                case 0b01011:
                case 0b10101:
                case 0b01110:
                case 0b11111:
                case 0b00100:
                    if (distData[cDS[0]][2].d < 10)
                    {
                        A = lastDir;
                        B = -lastDir;
                    }
                    else
                    {
                        A = 0.95;
                        B = 0.95;
                    }
                    break;
                default:
                    if (diffC != cDS[0]) printf("!\n");
                    break;
                }
                if (A > B)
                lastDir = 1;
                else
                lastDir = -1;

                if (diffC != cDS[0])
                {
                    printf("%u, %f, %f, %f, %f, %f, %f,%f, %f\n",inRange, distData[cDS[0]][0],distData[cDS[0]][1],distData[cDS[0]][2],distData[cDS[0]][3],distData[cDS[0]][4],A,B,deriv);
                    diffC = cDS[0];
                }

                
                if (cTime - motor_reset_timestamp > motor_reset_duty_u)
                {
                    A = 0;
                    B = 0;
                }
                if (cTime - motor_reset_timestamp > motor_reset_period_u)
                {
                    motor_reset_timestamp = cTime;
                }
                
                if (stateC == 5)
                {
                    set_A(A);
                    set_B(B);
                }
                else
                {
                    set_A(0);
                    set_B(0);
                }
                usleep(1000);
            }
            break;
        case 7: //beacon mode
        case 8: //beacon mode without drive
        {
            gettimeofday(&currentTime, NULL);
            double uTime = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
            float A = 0;
            float B = 0;
            float deriv = 0;
            float dir;

            double thresholdTime = 750000;

            int distanceSensorChecked[5] = {0,0,0,0,0};
            int distanceInRange = -1;

            for (i=0; i<3; i++)
            {
                if (IRTime[rIR[0]]-IRTime[rIR[i]]>1250000 || distanceInRange != -1)
                {
                    break;
                }
                switch (IRRecentID[rIR[i]])
                {
                case 0:
                    if (!distanceSensorChecked[0])
                    {
                        if (diffR != rIR[0] || diffC != cDS[0]) printf(" 0,0,%f,",distData[cDS[0]][0].d);
                        if (distData[cDS[0]][0].d<10)
                        {
                            distanceInRange = 0;
                            A = -1;
                            B = distData[cDS[0]][0].d/10;
                        }
                        distanceSensorChecked[0] = 1;
                    }
                    else if (!distanceSensorChecked[1])
                    {
                        if (diffR != rIR[0] || diffC != cDS[0]) printf(" 0,1,%f,",distData[cDS[0]][1].d);
                        if (distData[cDS[0]][1].d<10)
                        {
                            distanceInRange = 1;
                            A = -1;
                            B = distData[cDS[0]][1].d/10;
                        }
                        distanceSensorChecked[1] = 1;
                    }
                    break;
                case 5:
                    if (!distanceSensorChecked[2])
                    {
                        if (diffR != rIR[0] || diffC != cDS[0]) printf(" 5,2,%f,",distData[cDS[0]][2].d);
                        if (distData[cDS[0]][2].d<10)
                        {
                            distanceInRange = 2;
                            A = -1;//distData[cDS[0]][2].d/10-1;
                            B = 1;
                        }
                        distanceSensorChecked[4] = 1;
                    }
                    else if (!distanceSensorChecked[1])
                    {
                        if (diffR != rIR[0] || diffC != cDS[0]) printf(" 5,1,%f,",distData[cDS[0]][1].d);
                        if (distData[cDS[0]][1].d<10)
                        {
                            distanceInRange = 1;
                            A = -1;
                            B = 0;//distData[cDS[0]][1].d/10;
                        }
                        distanceSensorChecked[1] = 1;
                    }
                    break;
                case 4:
                    if (!distanceSensorChecked[3])
                    {
                        if (diffR != rIR[0] || diffC != cDS[0]) printf(" 4,3,%f,",distData[cDS[0]][3].d);
                        if (distData[cDS[0]][3].d<10)
                        {
                            distanceInRange = 3;
                            A = -1;//distData[cDS[0]][3].d/5-1;
                            B = 1;
                        }
                        distanceSensorChecked[3] = 1;
                    }
                    break;
                case 3:
                    if (!distanceSensorChecked[4])
                    {
                        if (diffR != rIR[0] || diffC != cDS[0]) printf(" 3,4,%f,",distData[cDS[0]][4].d);
                        if (distData[cDS[0]][4].d<15)
                        {
                            distanceInRange = 4;
                            if (distData[cDS[0]][4].d>10)
                            {
                                A = 1;
                                B = 0;
                            }
                            else
                            {
                                A = 0.65;
                                B = 0.65;
                            }
                        }
                        distanceSensorChecked[4] = 1;
                    }
                case 2:
                    if (!distanceSensorChecked[4])
                    {
                        if (diffR != rIR[0] || diffC != cDS[0]) printf(" 2,4,%f,",distData[cDS[0]][4].d);
                        if (distData[cDS[0]][4].d<10)
                        {
                            distanceInRange = 4;
                            A = 0.65;
                            B = 0.65;
                        }
                        distanceSensorChecked[4] = 1;
                    }
                    break;
                default:
                    if (diffR != rIR[0] || diffC != cDS[0]) printf(" +");
                    A = 0;
                    B = 0;
                    break;
                }
            }
            if(distanceInRange == -1){
                switch (IRRecentID[rIR[0]]){
                case 0:
                    A=-1;
                    B=1;
                    break;
                case 5:
                case 4:
                    A=-1;
                    B=1;
                    break;
                case 3:
                    A=1;
                    B=0;
                    break;
                case 2:
                    A=1;
                    B=-1;
                    break;
                case 1:
                    A=1;
                    B=-1;
                    break;
                default:
                    A=0;
                    B=0;
                    break;
                }
                if (diffR != rIR[0] || diffC != cDS[0])
                {
                    printf(" NDD%d", IRRecentID[rIR[0]]);
                }
            }
            if (diffR != rIR[0] || diffC != cDS[0])
            {
                printf("\nWall %d %d %f", nodeWallDetect[0], nodeWallDetect[1], distData[cDS[0]][0]);
            }

            if (A >= B) //if we are going in the correct direction, check the opposite wall
            {
                double tempTime;
                j = 0;
                for (i=0;i<4;i++)
                {
                    if (distData[cDS[i]][0].d < 30)
                    {
                        j++;
                    }
                    else
                    {
                        j--;
                    }
                }

                if (j == 4 && nodeWallDetect[0] == 0)
                {
                    if (wallDetectDebounceTime == -1)
                    {
                        gettimeofday(&currentTime, NULL);
                        wallDetectDebounceTime = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
                    }
                    else
                    {
                        gettimeofday(&currentTime, NULL);
                        tempTime = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
                        if (tempTime - wallDetectDebounceTime > thresholdTime)
                        {
                            if (nodeWallDetect[1] == 1) gateAdvance--;
                            nodeWallDetect[1] = 0;
                            nodeWallDetect[0] = 1;
                        }
                    }
                }
                else if (j == -4 && nodeWallDetect[0] == 1)
                {
                    if (wallDetectDebounceTime == -1)
                    {
                        gettimeofday(&currentTime, NULL);
                        wallDetectDebounceTime = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
                    }
                    else
                    {
                        gettimeofday(&currentTime, NULL);
                        tempTime = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
                        if (tempTime - wallDetectDebounceTime > thresholdTime)
                        {
                            if (gateAdvance == 0)
                            {
                                gettimeofday(&currentTime, NULL);
                                s_time = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
                                stateC = 9;
                                currentBeacon = ((IRRecent[rIR[0]][0] & 0xF0) >> 4);
                                fprintf(fp, "Departing Beacon: %d, ", currentBeacon);
                            }
                            nodeWallDetect[1] = 1;
                            nodeWallDetect[0] = 0;
                        }
                    }
                }
                else
                {
                    wallDetectDebounceTime = -1;
                }
            }
            else
            {
                wallDetectDebounceTime = -1;
            }

            if (uTime - motor_reset_timestamp > motor_reset_duty_u)
            {
                A = 0;
                B = 0;
            }
            if (uTime - motor_reset_timestamp > motor_reset_period_u)
            {
                motor_reset_timestamp = uTime;
            }
            if (diffR != rIR[0] || diffC != cDS[0]){
                printf(" %f|%f", A, B);
            }
            if (diffR != rIR[0] || diffC != cDS[0])
            {
                printf("\n");
                diffR = rIR[0];
                diffC = cDS[0];
            }
            if (stateC == 7)
            {
                set_A(A);
                set_B(B);
            }
            else
            {
                set_A(0);
                set_B(0);
            }
            break;
        }
        case 9: //exit beacon mode
        {
            gettimeofday(&currentTime, NULL);
            double uTime = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
            float A = 0;
            float B = 0;
            float deriv = 0;
            float dir;

            switch (IRRecentID[rIR[0]]){
            case 0:
                A=1;
                B=-1;
                break;
            default:
                A=-0.5;
                B=1;
                break;
            }

            j = 5;

            for (i=0; i<5; i++)
            {
                /*
                if (IRTime[rIR[0]]-IRTime[rIR[i]]>1500000)
                {
                    break;
                }
                */
                if (IRRecentID[rIR[i]] != 1) j--;
            }

            if (j == 5)
            {
                gettimeofday(&currentTime, NULL);
                s_time = 1000000.0 * currentTime.tv_sec + currentTime.tv_usec;
                stateC = 5;
            }

            if (uTime - motor_reset_timestamp > motor_reset_duty_u)
            {
                A = 0;
                B = 0;
            }
            if (uTime - motor_reset_timestamp > motor_reset_period_u)
            {
                motor_reset_timestamp = uTime;
            }


            if (diffR != rIR[0] || diffC != cDS[0])
            {
                printf("exit beacon %d\n", j);
                diffR = rIR[0];
                diffC = cDS[0];
            }
            set_A(A);
            set_B(B);
            break;
        }
        default:
            set_A(0);
            set_B(0);
            stateC = 0;
            break;
        }
    }



}

int main(){
    printf("Program Start\n");
    open_file();

    stateI = 0;
    stateC = 0;
    char* inputStr = NULL;
    size_t inputN = 10;

    long i;
    pthread_t thread1, thread2;
    int iret1, iret2;
    const char* message1 = "Thread 1";
    const char* message2 = "Thread 2";

    inputStr = malloc(inputN);

    mraa_init();

    uart = mraa_uart_init(0);

    initialize(); //for motors

    if (uart == NULL) {
        fprintf(stderr, "UART failed to setup\n");
        return EXIT_FAILURE;
    }
    mraa_uart_set_mode(uart, 8, MRAA_UART_PARITY_NONE, 1);
    mraa_uart_set_flowcontrol(uart, 1, 0);

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


    iret1 = pthread_create(&thread1, NULL, receiverLoop, (void*) message1);
    if(iret1)
    {
        fprintf(stderr,"Error - pthread_create() return code: %d\n",iret1);
        exit(EXIT_FAILURE);
    }

    printf("pthread_create() for thread 1 returns: %d\n",iret1);

    iret2 = pthread_create(&thread2, NULL, controlFunc, (void*) message2);
    if(iret2)
    {
        fprintf(stderr,"Error - pthread_create() return code: %d\n",iret2);
        exit(EXIT_FAILURE);
    }

    printf("pthread_create() for thread 2 returns: %d\n",iret2);

    while (1)
    {
        getline(&inputStr, &inputN, stdin);
        if (inputStr[0]=='x')
        {
            stateI = -1;
        }
        if (inputStr[0]=='f')
        {
            stateI = 1;
        }
        if (inputStr[0]=='b')
        {
            stateI = 2;
        }
        if (inputStr[0]=='l')
        {
            stateI = 3;
        }
        if (inputStr[0]=='r')
        {
            stateI = 4;
        }
        if (inputStr[0]=='p')
        {
            stateI = 5;
        }
        if (inputStr[0]=='d')
        {
            stateI = 6;
        }
        if (inputStr[0]=='c')
        {
            stateI = 7;
        }
    }

    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);

    return 0;
}
