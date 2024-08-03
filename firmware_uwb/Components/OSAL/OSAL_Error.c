#include <stdio.h>
#include <stdlib.h>
#include "OSAL_Error.h"


void osal_error_output(char *API,int err_para)
{
		_dbg_printf("\n********************************************************\nosal API:<%s> \nosal EC :<%x>\n********************************************************\n", API, err_para);
		if(err_para >= osal_error_alert_queue_full)
				return;
		//exit(1);		
}
