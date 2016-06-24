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

int main(int argc, char *argv[]){
  unsigned char a = (unsigned char)(argv[1][0]) - 48;
  unsigned char b = (unsigned char)(argv[1][1]) - 48;
  unsigned char pattern = a << 4 | b;
  int k, sendBit;
  int previousState=0;
  int halfPeriod=4000; // each bit lasts 8000uS

  mraa_pwm_context pwm;
  pwm = mraa_pwm_init(0);

  uint16_t value;


  if (pwm == NULL){
    fprintf(stderr, "Failed to initialize\n");
    return 1;
  }

  mraa_pwm_enable(pwm,1);
  mraa_pwm_period_us(pwm, 26);

  while(1){
      //start bit
      mraa_pwm_enable(pwm,1);
      mraa_pwm_period_us(pwm, 26);
      mraa_pwm_write(pwm, .5);
      usleep(8*halfPeriod);
      previousState=1;

      for(k=0; k<8; k++){
        sendBit= (pattern >> k) & 1;
        if(sendBit == 1){
          previousState=switchState(previousState,pwm);
          usleep(halfPeriod);
          previousState=switchState(previousState,pwm);
          usleep(halfPeriod);
        }
        else{
          previousState=switchState(previousState,pwm);
          usleep(2*halfPeriod);
        }
      }
      //end of pattern

      //End Bit
      previousState=switchState(previousState,pwm);
      usleep(4*halfPeriod);
      previousState=switchState(previousState,pwm);
      usleep(halfPeriod);

      //Wait before sending again, or send immediately
      mraa_pwm_write(pwm, 0);
      previousState=0;
      //mraa_pwm_enable(pwm, 0);
      //usleep(100000);

  }
  return 0;
}


