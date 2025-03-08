#include "OSAL_Task.h"
#include "hal_drivers.h"

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void osal_InitTasks( void )
{
		uint8_t taskID = 0;	
		//OSAL_Init(taskID++);
		Hal_Init( taskID++ );
		//MT_Init( taskID++ );	
		//APP_Init( taskID++ );	
}
