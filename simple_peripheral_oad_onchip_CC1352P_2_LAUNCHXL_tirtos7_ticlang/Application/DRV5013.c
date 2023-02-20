
#include <DRV5013.h>
#include <ti/drivers/GPIO.h>



extern int Speed_Counter=0;
static void DRV5013_INT_Callback(uint_least8_t index);
/* Timer buffer */

int timerBufIndex = 0;
double timerBuf[10] = {0};
uint8_t check_if_open=0;

void DRV5013_init(void)
{
	Speed_Counter=0;

    GPIO_setConfig(SPEED_INT, GPIO_CFG_INPUT | GPIO_CFG_IN_INT_FALLING);


    /* Install Button callback */
    GPIO_setCallback(SPEED_INT, DRV5013_INT_Callback);

    /* Enable interrupts */
    GPIO_disableInt(SPEED_INT);

    GPIO_write(SPEED_INT,0);

}


static void DRV5013_INT_Callback(uint_least8_t index)
{
  if (index == SPEED_INT)
  {
      GPIO_clearInt(SPEED_INT);
	  Speed_Counter++;
  }
}

void DRV5013_Power_Up(void)
{
	if(check_if_open==0)
	{
		/* Start timer */
	    GPIO_enableInt(SPEED_INT);
		check_if_open=1;
	}
}

void DRV5013_Power_Down(void)
{
	if(check_if_open==1)
	{
		check_if_open=0;
		GPIO_disableInt(SPEED_INT);
		GPIO_write(SPEED_INT,0);
	}
}
