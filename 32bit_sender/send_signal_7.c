#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <mraa/pwm.h>
#include <mraa/aio.h>

int switchState(int state, mraa_pwm_context pwmPinContext)
{
  state = !state;
  mraa_pwm_write(pwmPinContext, 0.5*state);
  return state;
}

int main(){
  char pattern[1024];
  char currentChar;
  int msgLen = 32;
  int i, j, k, sendBit;
  int previousState=0;
  int bitTime=4000; // each bit lasts 8000uS

  mraa_pwm_context pwm;
  pwm = mraa_pwm_init(1);

  uint16_t value;


  if (pwm == NULL){
    fprintf(stderr, "Failed to initialize\n");
    return 1;
  }

  mraa_pwm_enable(pwm,1);
  mraa_pwm_period_us(pwm, 26);

  while(1){
    //start bit is high for 3 bits
    printf("Enter a 4 byte pattern to send: ");
    scanf("%s", pattern);

    //send pattern 10 times
    for(i=0; i<10; i++){
      //start bit
      mraa_pwm_enable(pwm,1);
      mraa_pwm_period_us(pwm, 26);
      mraa_pwm_write(pwm, .5);
      usleep(8*bitTime);
      previousState=1;

      //pattern
      for(j=0; j<(msgLen/8); j++){
        currentChar=pattern[j];
        for(k=0; k<8; k++){
          sendBit= (currentChar >> k) & 1;
          if(sendBit == 1){
            previousState=switchState(previousState,pwm);
            usleep(bitTime);
            previousState=switchState(previousState,pwm);
            usleep(bitTime);
          }
          else{
            previousState=switchState(previousState,pwm);
            usleep(2*bitTime);
          }
        }
      }
      //end of pattern

      //End Bit
      previousState=switchState(previousState,pwm);
      usleep(4*bitTime);
      previousState=switchState(previousState,pwm);
      usleep(bitTime);

      //Wait 1 second before sending again
      mraa_pwm_write(pwm, 0);
      previousState=0;
      //mraa_pwm_enable(pwm, 0);
      usleep(100000);


    }
  }
  return 0;
}
