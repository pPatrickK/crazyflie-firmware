/**
 * .___   _____  .____          /\ ___________.____   __      __
 * |   | /     \ |    |        / / \_   _____/|    | /  \    /  \
 * |   |/  \ /  \|    |       / /   |    __)  |    | \   \/\/   /
 * |   /    Y    \    |___   / /    |     \   |    |__\        /
 * |___\____|__  /_______ \ / /     \___  /   |_______ \__/\  /
 *             \/        \/ \/          \/            \/    \/
 *
 * Dshot motor driver for IML/FLW brushless drones.
 *
 * From timer interrupt handlers, call TIM_IRQHandler() defined here.
 *
 * This code is using the stdperiph lib of ST, blabla.
 */

#include "motors.h"

#ifdef MOTORS_DSHOT

#include <stdbool.h>

/* ST includes */
#include "stm32fxxx.h"

#include "pm.h"

//FreeRTOS includes
#include "task.h"

//Logging includes
#include "log.h"

// Test Nils/Tobbe includes
#include "nvicconf.h"

void motorsPlayTone(uint16_t frequency, uint16_t duration_msec);
void motorsPlayMelody(uint16_t *notes);
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio);

#ifdef PLATFORM_CF1
#error "Crazyflie 1 not supported!"
#endif

#include "motors_def_cf2.c"

const MotorPerifDef** motorMap;  /* Current map configuration */

const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

const uint16_t testsound[NBR_OF_MOTORS] = {A4, A5, F5, D5 };

static bool isInit = false;

// The telemetry bit is a parameter to motorsSetRatio() but we do not want to
// change the function interface to keep maximum interoperability. Therefore,
// we use a global variable that can be set (should be 0 normally) before
// calling motorsSetRatio().
static bool telemetry_bit = 0;

/* Private functions */

// no-one needs private functions, everything's public here

/* Public functions */

// Desired dshot bit-period is 6.68 us
// 6.68 us == 1/TIM_CLOCK_HZ * (prescaler+1)
// prescaler == ( 6.68us / (1/TIM_CLOCK_HZ) ) - 1 == 6.68 us * TIM_CLOCK_HZ - 1
//
// Inductrix D-Shot Timings:
// Period 6.6us
// Short  2.4us
// Long   4.9us
//
// In our experience, this was unstable if the period was too short, while it worked whenever
// the period was slightly too long. Therefore, we just add a safety 6 on the prescaler :)
#define DSHOT_PERIOD (TIM_CLOCK_HZ / 149700 + 5)

void wait(int delay) {
	/*
	static bool unused = true;
	for (int i = 0; i < delay * 5700; i++)
		unused = !unused;
	*/
	vTaskDelay(delay / portTICK_PERIOD_MS);
}

//Initialization. Will set all motors ratio to 0%
void motorsInit(const MotorPerifDef** motorMapSelect)
{
	// The beginning of the code is copied from motors.c while at the appropriate positions,
	// dshot-relevant code was added. Diffing motors.c and this file should tell you more.

  int i;
  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  if (isInit)
  {
    motorsDeInit(motorMap);
  }

  motorMap = motorMapSelect;

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    //Clock the gpio and the timers
    MOTORS_RCC_GPIO_CMD(motorMap[i]->gpioPerif, ENABLE);
    MOTORS_RCC_TIM_CMD(motorMap[i]->timPerif, ENABLE);

    // Configure the GPIO for the timer output
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = MOTORS_GPIO_MODE;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef PLATFORM_CF2
    GPIO_InitStructure.GPIO_OType = motorMap[i]->gpioOType;
#endif
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

    //Map timers to alternate functions
    MOTORS_GPIO_AF_CFG(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, motorMap[i]->gpioAF);

    //Timer configuration
    TIM_TimeBaseStructure.TIM_Period = DSHOT_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(motorMap[i]->tim, &TIM_TimeBaseStructure);

    // PWM channels configuration (All identical!)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = motorMap[i]->timPolarity;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    // Configure Output Compare for PWM
    motorMap[i]->ocInit(motorMap[i]->tim, &TIM_OCInitStructure);
    motorMap[i]->preloadConfig(motorMap[i]->tim, TIM_OCPreload_Enable);

    MOTORS_TIM_DBG_CFG(motorMap[i]->timDbgStop, ENABLE);
    //Enable the timer PWM outputs
    TIM_CtrlPWMOutputs(motorMap[i]->tim, ENABLE);

    // Only for dshot: Enable TIM interrupt(s)
    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = motorMap[i]->timIrq;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Initially, the timer and its corresponding interrupt should be disabled
	TIM_Cmd(motorMap[i]->tim, DISABLE);
	TIM_ITConfig(motorMap[i]->tim, TIM_IT_Update, DISABLE);
  }

  isInit = true;

  wait(2000);

  for (int i = 0; i < 1500; i++) {
		  telemetry_bit = 0;
		  motorsSetRatio(1, 0);
			motorsSetRatio(0, 0);
		  motorsSetRatio(2, 0);
		  motorsSetRatio(3, 0);
		  telemetry_bit = 0;
	    wait(1);
  }

  wait(10);
  //Send a 1 as throttle value, 48 is the first throttle bit
  for (int i = 0; i < 1500; i++) {
  		telemetry_bit = 1;
  		motorsSetRatio(1, 48 << 5);
			motorsSetRatio(0, 48 << 5);
  		motorsSetRatio(2, 48 << 5);
  		motorsSetRatio(3, 48 << 5);
  		telemetry_bit = 0;
  	  wait(1);
  }

  // Test dshot implementation
	// (speed controlled with a sawtooth signal, blocks forever)
	//---------------------------------------------------------------------------------------------THRUST TEST
  /*uint16_t dshot = 100 << 5;
  while (true) {
      for (uint32_t i = 0; i < 200000; i++) {
          isInit = !isInit;
      }

      dshot = dshot + (1 << 5);
      if (dshot > 1 << 14)
          dshot = 100 << 5;
	  //dshot = (2000 << 5);
      //motorsSetRatio(0, dshot);
      motorsSetRatio(1, dshot);
			motorsSetRatio(0, dshot);
      motorsSetRatio(2, dshot);
      motorsSetRatio(3, dshot);
  }*/

	// Test dshot implementation 2

	/*uint16_t goal = (256 << 5);
	uint16_t dshot = 0;
	dshot = (128 << 5);
	while (true) {
		for (uint32_t i = 0; i < 400000; i++) {

		}
			if (dshot < goal) {
				dshot = dshot + (1 << 5);
			}
			//dshot = (768 << 5);
			motorsSetRatio(0, dshot);
			motorsSetRatio(1, dshot);
			motorsSetRatio(2, dshot);
			motorsSetRatio(3, dshot);
		}*/

//------------------------------------------------------------------------------------------------------THRUST TEST ENDE
}

void motorsDeInit(const MotorPerifDef** motorMapSelect)
{
  int i;
  GPIO_InitTypeDef GPIO_InitStructure;

  for (i = 0; i < NBR_OF_MOTORS; i++)
  {
    // Configure default
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = motorMap[i]->gpioPin;
    GPIO_Init(motorMap[i]->gpioPort, &GPIO_InitStructure);

#ifdef PLATFORM_CF1
    //Map timers to alternate functions
    GPIO_PinRemapConfig(motorMap[i]->gpioAF , DISABLE);
#else
    //Map timers to alternate functions
    GPIO_PinAFConfig(motorMap[i]->gpioPort, motorMap[i]->gpioPinSource, 0x00);
#endif

    //Deinit timer
    TIM_DeInit(motorMap[i]->tim);
  }
}

bool motorsTest(void)
{
    // A few motor functions commented out for debugging/testing purpose
    /*
  int i;

  for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
  {
    if (motorMap[i]->drvType == BRUSHED)
    {
#ifdef ACTIVATE_STARTUP_SOUND
      motorsBeep(MOTORS[i], true, testsound[i], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
      vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
      motorsBeep(MOTORS[i], false, 0, 0);
      vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#else
      motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
      vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
      motorsSetRatio(MOTORS[i], 0);
      vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
#endif
    }
  }
  */

  return isInit;
}

static uint16_t motorRatios[NBR_OF_MOTORS] = {0};
static uint16_t motorCompare[16][NBR_OF_MOTORS];
static uint8_t motorCompareCnt[NBR_OF_MOTORS];

void motorsSetRatio(uint32_t currId, uint16_t ithrust)
{
  if (!isInit)
	  return;

  ASSERT(currId < NBR_OF_MOTORS);
	/*uint16_t ratio;
	#ifdef ENABLE_THRUST_BAT_COMPENSATED
		float thrust = ((float)ithrust / 65536.0f) * 60;
		float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
		float supply_voltage = pmGetBatteryVoltage();
		float percentage = volts / supply_voltage;
		percentage = percentage > 1.0f ? 1.0f : percentage;
		ratio = percentage * UINT16_MAX;
		motorRatios[currId] = ratio;
	#endif
	*/
  // ithrust is 16 bit, dshot uses 11 bit, anyways, those 11 bit are the most significant
  // bits in the dshot data. Therefore, we do not need to scale the value because with the
  // appropriate mask (later), this will happen automatically.
  motorRatios[currId] = ithrust;

  // As most probably, multiple motors will use the same timer, we have to set their values
  // at the same time. Therefore, we start ALL timers when the last motor value was updated.
  if (currId != NBR_OF_MOTORS-1)
	  return;

  // Possible race conditions: It is not exactly clear what will happen when the timer interrupts
  // are still active while this functions writes to the variables that will be used in the
  // corresponding interrupt handlers.

  // Dshot (nanoseconds)
  // period = 6680
  // 1 = 5000 high + 1680 low (75% high == 3/4 == 6/8)
  // 0 = 2500 high + 4180 low (37.5% high == 3/8)

  for (uint8_t id = 0; id < NBR_OF_MOTORS; id++) {
      uint16_t mask = 0x8000;
      for (uint8_t bit = 0; bit < 11; bit++) {
          motorCompare[bit][id] = motorRatios[id] & mask ? 1 : 0;
          mask >>= 1;
      }
      // set telemetry bitmotorMap
      motorCompare[11][id] = telemetry_bit;
      // crc
      for (uint8_t i = 0; i<4; i++) {
          uint16_t crc_sum = 0;
          crc_sum = motorCompare[i][id] ^ motorCompare[i+4][id];
          crc_sum = (crc_sum ^ motorCompare[i+8][id]) & 0xf;
          motorCompare[i+12][id] = crc_sum;
      }

      for (uint8_t bit = 0; bit < 16; bit++)
          //motorCompare[bit][id] = motorCompare[bit][id] ? DSHOT_PERIOD * 6 / 8 : DSHOT_PERIOD * 3 / 8;
    	  motorCompare[bit][id] = motorCompare[bit][id] ? DSHOT_PERIOD * 49 / 66 : DSHOT_PERIOD * 24 / 66;

      motorCompareCnt[id] = 0;
  }

  for (uint8_t i = 0; i < NBR_OF_MOTORS; i++)
  {
      TIM_Cmd(motorMap[i]->tim, ENABLE);
      TIM_ITConfig(motorMap[i]->tim, TIM_IT_Update, ENABLE);
  }
}

int motorsGetRatio(uint32_t id)
{
    return motorRatios[id];
}

void TIM_IRQHandler(TIM_TypeDef *tim) {
	TIM_ClearITPendingBit(tim, TIM_IT_Update);

    // For each motor ...
    for (uint8_t id = 0; id < NBR_OF_MOTORS; id++) {
    	//if (id < 3) continue;
        // Check if this motor is handled by this timer
        if (motorMap[id]->tim == tim) {
            uint8_t cnt = motorCompareCnt[id]; // which bit is currently handled
            if (cnt < 16) { // 16 bits dshot
			    			motorMap[id]->setCompare(motorMap[id]->tim, motorCompare[cnt][id] );
            }
            else if (cnt == 16) { // disable output
                motorMap[id]->setCompare(motorMap[id]->tim, 0);
            }
            else { // disable timer / interrupts
                TIM_Cmd(motorMap[id]->tim, DISABLE);
                TIM_ITConfig(motorMap[id]->tim, TIM_IT_Update, DISABLE);
            }
            motorCompareCnt[id] = (cnt+1);
        }
    }
}

/* Set PWM frequency for motor controller
 * This function will set all motors into a "beep"-mode,
 * each of the motor will turned on with a given ratio and frequency.
 * The higher the ratio the higher the given power to the motors.
 * ATTENTION: To much ratio can push your crazyflie into the air and hurt you!
 * Example:
 *     motorsBeep(true, 1000, (uint16_t)(72000000L / frequency)/ 20);
 *     motorsBeep(false, 0, 0); *
 * */
void motorsBeep(int id, bool enable, uint16_t frequency, uint16_t ratio)
{
    /*
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  ASSERT(id < NBR_OF_MOTORS);

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  if (enable)
  {
    TIM_TimeBaseStructure.TIM_Prescaler = (5 - 1);
    TIM_TimeBaseStructure.TIM_Period = (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency);
  }
  else
  {
    TIM_TimeBaseStructure.TIM_Period = motorMap[id]->timPeriod;
    TIM_TimeBaseStructure.TIM_Prescaler = motorMap[id]->timPrescaler;
  }

  // Timer configuration
  TIM_TimeBaseInit(motorMap[id]->tim, &TIM_TimeBaseStructure);
  motorMap[id]->setCompare(motorMap[id]->tim, ratio);
  */
}


// Play a tone with a given frequency and a specific duration in milliseconds (ms)
void motorsPlayTone(uint16_t frequency, uint16_t duration_msec)
{
    /*
  motorsBeep(MOTOR_M1, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M2, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M3, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  motorsBeep(MOTOR_M4, true, frequency, (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / frequency)/ 20);
  vTaskDelay(M2T(duration_msec));
  motorsBeep(MOTOR_M1, false, frequency, 0);
  motorsBeep(MOTOR_M2, false, frequency, 0);
  motorsBeep(MOTOR_M3, false, frequency, 0);
  motorsBeep(MOTOR_M4, false, frequency, 0);
  */
}

// Plays a melody from a note array
void motorsPlayMelody(uint16_t *notes)
{
    /*
  int i = 0;
  uint16_t note;      // Note in hz
  uint16_t duration;  // Duration in ms

  do
  {
    note = notes[i++];
    duration = notes[i++];
    motorsPlayTone(note, duration);
  } while (duration != 0);
  */
}

LOG_GROUP_START(pwm)
LOG_ADD(LOG_UINT32, m1_pwm, &motorRatios[0])
LOG_ADD(LOG_UINT32, m2_pwm, &motorRatios[1])
LOG_ADD(LOG_UINT32, m3_pwm, &motorRatios[2])
LOG_ADD(LOG_UINT32, m4_pwm, &motorRatios[3])
LOG_GROUP_STOP(pwm)

#endif // MOTORS_DSHOT
