/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "std_cpp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId ApplicationTaskHandle;
osThreadId RobotSystemTaskHandle;
osThreadId ControlTaskHandle;
osThreadId StateCoreTaskHandle;
osThreadId Coroutine_0_TasHandle;
osThreadId Coroutine_1_TasHandle;
osThreadId Coroutine_2_TasHandle;
osThreadId Coroutine_3_TasHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Application(void const * argument);
void RobotSystem(void const * argument);
void Control(void const * argument);
void StateCore(void const * argument);
void Coroutine_0(void const * argument);
void Coroutine_1(void const * argument);
void Coroutine_2(void const * argument);
void Coroutine_3(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ApplicationTask */
  osThreadDef(ApplicationTask, Application, osPriorityBelowNormal, 0, 512);
  ApplicationTaskHandle = osThreadCreate(osThread(ApplicationTask), NULL);

  /* definition and creation of RobotSystemTask */
  osThreadDef(RobotSystemTask, RobotSystem, osPriorityNormal, 0, 256);
  RobotSystemTaskHandle = osThreadCreate(osThread(RobotSystemTask), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, Control, osPriorityAboveNormal, 0, 256);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of StateCoreTask */
  osThreadDef(StateCoreTask, StateCore, osPriorityIdle, 0, 512);
  StateCoreTaskHandle = osThreadCreate(osThread(StateCoreTask), NULL);

  /* definition and creation of Coroutine_0_Tas */
  osThreadDef(Coroutine_0_Tas, Coroutine_0, osPriorityIdle, 0, 128);
  Coroutine_0_TasHandle = osThreadCreate(osThread(Coroutine_0_Tas), NULL);

  /* definition and creation of Coroutine_1_Tas */
  osThreadDef(Coroutine_1_Tas, Coroutine_1, osPriorityIdle, 0, 128);
  Coroutine_1_TasHandle = osThreadCreate(osThread(Coroutine_1_Tas), NULL);

  /* definition and creation of Coroutine_2_Tas */
  osThreadDef(Coroutine_2_Tas, Coroutine_2, osPriorityIdle, 0, 128);
  Coroutine_2_TasHandle = osThreadCreate(osThread(Coroutine_2_Tas), NULL);

  /* definition and creation of Coroutine_3_Tas */
  osThreadDef(Coroutine_3_Tas, Coroutine_3, osPriorityIdle, 0, 128);
  Coroutine_3_TasHandle = osThreadCreate(osThread(Coroutine_3_Tas), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Application */
/**
* @brief Function implementing the ApplicationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Application */
void Application(void const * argument)
{
  /* USER CODE BEGIN DEFAULT_TASK_FUNCTION */
  /* Infinite loop */
  ApplicationCpp();
  /* USER CODE END DEFAULT_TASK_FUNCTION */
}

/* USER CODE BEGIN Header_RobotSystem */
/**
* @brief Function implementing the RobotSystemTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RobotSystem */
void RobotSystem(void const * argument)
{
  /* USER CODE BEGIN RobotSystem */
  /* Infinite loop */
  RobotSystemCpp();
  /* USER CODE END RobotSystem */
}

/* USER CODE BEGIN Header_Control */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control */
void Control(void const * argument)
{
  /* USER CODE BEGIN Control */
  /* Infinite loop */
  ControlCpp();
  /* USER CODE END Control */
}

/* USER CODE BEGIN Header_StateCore */
/**
* @brief Function implementing the StateCoreTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StateCore */
void StateCore(void const * argument)
{
  /* USER CODE BEGIN StateCore */
  /* Infinite loop */
  StateCoreCpp();
  /* USER CODE END StateCore */
}

/* USER CODE BEGIN Header_Coroutine_0 */
/**
* @brief Function implementing the Coroutine_0_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Coroutine_0 */
void Coroutine_0(void const * argument)
{
  /* USER CODE BEGIN Coroutine_0 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Coroutine_0 */
}

/* USER CODE BEGIN Header_Coroutine_1 */
/**
* @brief Function implementing the Coroutine_1_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Coroutine_1 */
void Coroutine_1(void const * argument)
{
  /* USER CODE BEGIN Coroutine_1 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Coroutine_1 */
}

/* USER CODE BEGIN Header_Coroutine_2 */
/**
* @brief Function implementing the Coroutine_2_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Coroutine_2 */
void Coroutine_2(void const * argument)
{
  /* USER CODE BEGIN Coroutine_2 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Coroutine_2 */
}

/* USER CODE BEGIN Header_Coroutine_3 */
/**
* @brief Function implementing the Coroutine_3_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Coroutine_3 */
void Coroutine_3(void const * argument)
{
  /* USER CODE BEGIN Coroutine_3 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Coroutine_3 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
