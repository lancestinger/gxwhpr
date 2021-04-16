#include "cJSON.h"
#include "json/Cjson_run.h"
#include "drv/uart/uart_drv.h"
#include "comm/project_def.h"
#include <stdio.h>
#include <stdlib.h>
#include "EH_uwb/EH_uwb.h"
#include "EH_uwb/EH_uwb_parse.h"
#include "fml/gnss/nmea.h"


char* encode_Json(void) // 构建
{
    cJSON * pJsonRoot = NULL; // 创建父节点
    pJsonRoot = cJSON_CreateObject();
    if (NULL == pJsonRoot)
    {
		ERR_PRINT(("pJsonRoot Creat ERROR!!\r\n"));
		cJSON_Delete(pJsonRoot);
        return NULL;
    }
    cJSON_AddStringToObject(pJsonRoot, "title","position");
    cJSON_AddStringToObject(pJsonRoot, "date",Sys_Date);
	cJSON_AddStringToObject(pJsonRoot, "utc",Sys_UTC);
	cJSON_AddNumberToObject(pJsonRoot, "lat",NMEA_Data_ptr.lat);
	cJSON_AddNumberToObject(pJsonRoot, "lon",NMEA_Data_ptr.lon);
	cJSON_AddNumberToObject(pJsonRoot, "alt",NMEA_Data_ptr.alt);
	cJSON_AddNumberToObject(pJsonRoot, "mode",NMEA_Data_ptr.RTK_mode);
	
	//char * p = cJSON_Print(pJsonRoot);//带格式换行空格
	char * p = cJSON_PrintUnformatted(pJsonRoot);
    if (NULL == p)
    {
		ERR_PRINT(("Cjson p Creat ERROR!!\r\n"));
        cJSON_Delete(pJsonRoot);
        return NULL;
    }
    cJSON_Delete(pJsonRoot);
    return p;
		
}


