#include "update_apl.h"
#include <stdio.h>
#include "rl_net.h"
#include "project_def.h"
#include "server/server_apl.h"


/*------------------------------文件变量------------------------------*/
static U64 thread_update_stk[SIZE_4K / 8];
static const osThreadAttr_t thread_update_attr = {
  .stack_mem  = &thread_update_stk[0],
  .stack_size = sizeof(thread_update_stk),
  .priority = osPriorityNormal,//osPriorityBelowNormal6
};


UPDATE_HANDLE update_handle_g;


//! [code_FTP_Client_UIF]

// \brief Request parameters for FTP client session.
// \param[in]     request       request code.
// \param[out]    buf           output buffer to write the data to.
// \param[in]     buf_len       length of the output buffer in bytes.
// \return        number of bytes written to output buffer.
uint32_t netFTPc_Process (netFTPc_Request request, char *buf, uint32_t buf_len) 
{
    uint32_t len = 0;
    int32_t i;
	 
    switch (request) 
    {
        case netFTPc_RequestUsername:
        //0 Username to login to FTP server
        GLOBAL_PRINT(("FTP#Username to login to FTP server\r\n"));
        len = sprintf (buf, update_handle_g.userName);
        break;
    
        case netFTPc_RequestPassword:
        //1 Password to login to FTP server
        GLOBAL_PRINT(("FTP#Password to login to FTP server\r\n"));
        len = sprintf (buf, update_handle_g.password);
        break;
    
        case netFTPc_RequestDirectory:
        //2 Working directory path on server
        GLOBAL_PRINT(("FTP#Working directory path on server\r\n"));
        len = sprintf (buf, update_handle_g.workDir);
        break;
    
        case netFTPc_RequestName:
        //3 Filename for PUT, GET, APPEND, DELETE and RENAME commands
        // Directory name for MKDIR and RMDIR commands
        GLOBAL_PRINT(("FTP#Filename for PUT, GET, APPEND, DELETE and RENAME commands\r\n"));
        len = sprintf (buf, update_handle_g.fileName);
        break;
    
        case netFTPc_RequestNewName:
        //4 New name for a RENAME command
        GLOBAL_PRINT(("FTP#New name for a RENAME command\r\n"));
        /* Example
        len = sprintf (buf, "renamed.log");
        */
        break;
    
        case netFTPc_RequestListMask:
        //5 File filter/mask for LIST command (wildcards allowed)
        GLOBAL_PRINT(("FTP#File filter/mask for LIST command (wildcards allowed)\r\n"));
        /* Example
        len = sprintf (buf, "");
        */
        break;
    
        case netFTPc_RequestList:
        //6 Received data if LIST command is given
        GLOBAL_PRINT(("FTP#Received data if LIST command is given\r\n"));
        for (i = 0; i < buf_len; i++) 
        {
            //putchar(buf[i]);
            GLOBAL_PRINT(("%c",buf[i]));
        }

        break;
    
        case netFTPc_RequestLocalFilename:
        //7 Local filename
        GLOBAL_PRINT(("FTP#Local filename\r\n"));
        len = sprintf (buf, ARM_UPLOAD_FILE_NAME_TEMP);
        break;
    }
    return (len);
}
 
// \brief Notify the user application when FTP client operation ends.
// \param[in]     event         FTP client notification event.
// \return        none.
void netFTPc_Notify (netFTPc_Event event) 
{
    switch (event) 
    {
        case netFTPc_EventSuccess:
        //1 File operation successful
        GLOBAL_PRINT(("FTP@File operation successful\r\n"));
        update_handle_g.updateSta = updateSta_EventSuccess;
		GLOBAL_PRINT(("updateSta_EventSuccess SET OK!!\r\n"));
        break;
    
        case netFTPc_EventTimeout:
        //1 Timeout on file operation
        GLOBAL_PRINT(("FTP@Timeout on file operation\r\n"));
        update_handle_g.updateSta = updateSta_EventTimeout;
        break;
    
        case netFTPc_EventLoginFailed:
        //2 Login error, username/password invalid
        GLOBAL_PRINT(("FTP@Login error, username/password invalid\r\n"));
        update_handle_g.updateSta = updateSta_EventLoginFailed;
        break;
    
        case netFTPc_EventAccessDenied:
        //3 File access not allowed
        GLOBAL_PRINT(("FTP@File access not allowed\r\n"));
        update_handle_g.updateSta = updateSta_EventAccessDenied;
        break;
    
        case netFTPc_EventFileNotFound:
        //4 File not found
        GLOBAL_PRINT(("FTP@File not found\r\n"));
        update_handle_g.updateSta = updateSta_EventFileNotFound;
        break;
    
        case netFTPc_EventInvalidDirectory:
        //5 Working directory path not found
        GLOBAL_PRINT(("FTP@Working directory path not found\r\n"));
        update_handle_g.updateSta = updateSta_EventInvalidDirectory;
        break;
    
        case netFTPc_EventLocalFileError:
        //6 Local file open/write error
        GLOBAL_PRINT(("FTP@Local file open/write error\r\n"));
        update_handle_g.updateSta = updateSta_EventLocalFileError;
        break;
    
        case netFTPc_EventError:
        //7 Generic FTP client error
        GLOBAL_PRINT(("FTP@Generic FTP client error\r\n"));
        update_handle_g.updateSta = updateSta_EventError;
        break;
    }
}
//! [code_FTP_Client_UIF]

/*****************************************************************************
 函 数 名  : ftp_download_file
 功能描述  : FTP文件下载函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年1月
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
static void ftp_download_file(void)
{
    const NET_ADDR4 addr = { NET_ADDR_IP4, 0, update_handle_g.ftpServerAddr[0], 
                update_handle_g.ftpServerAddr[1], update_handle_g.ftpServerAddr[2], update_handle_g.ftpServerAddr[3] };

    GLOBAL_TRACE(("ftp server:%d.%d.%d.%d\r\n",update_handle_g.ftpServerAddr[0], 
                update_handle_g.ftpServerAddr[1], update_handle_g.ftpServerAddr[2], update_handle_g.ftpServerAddr[3]));
    if(netFTPc_Connect ((NET_ADDR *)&addr, netFTP_CommandGET) == netOK)
    {
        GLOBAL_TRACE(("FTP client started\r\n"));
    }
    else 
    {
        GLOBAL_TRACE(("FTP client is busy\r\n"));
    }
}


/*****************************************************************************
 函 数 名  : _update_thread
 功能描述  : 升级线程
 输入参数  : void* arg  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年1月
    作    者   : sunj
    修改内容   : 新生成函数

*****************************************************************************/
static void _update_thread(void* arg)
{
	FILE_UPLOAD_Enum temp_type;
	static U8 flag=0;
	
	while(1)
	{
        switch(update_handle_g.updateSta)
        {
            case updateSta_EventIdle:
				if(!flag){
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
					flag = TRUE;
				}else{
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
					flag = FALSE;
				}
            break;

            case updateSta_EventDownloadStart:
                ftp_download_file();
                update_handle_g.updateSta = updateSta_EventDownloading;
            break;

            case updateSta_EventDownloading:
                //计算下载进度()
            break;

            case updateSta_EventSuccess://下载文件完成进行校验和文件类型判断
            
                if(file_upload_check((U8*)ARM_UPLOAD_FILE_NAME_TEMP, &temp_type) == FILE_CHECK_SUC)
                {//校验成功
                    fdelete(ARM_UPLOAD_FILE_NAME, NULL);	//删除文件
                    frename(ARM_UPLOAD_FILE_NAME_TEMP, ARM_UPLOAD_FILE_NAME);
                    rtc_bakup_write(BKUP_BOOT_UPGRADE);
                    NOTE_PRINT(("升级文件为ARM!!!!\r\n"));
                    upload_hpr_update_feedback_to_server(serOriData, E_UPDATE_Success);
                }
                else
                {
                    /* code */
                    upload_hpr_update_feedback_to_server(serOriData, E_UPDATE_CheckFail);
                }
                
                update_handle_g.updateSta = updateSta_EventIdle;
            break;

            case updateSta_EventTimeout:
                upload_hpr_update_feedback_to_server(serOriData, E_UPDATE_Timeout);
                update_handle_g.updateSta = updateSta_EventIdle;
            break;

            case updateSta_EventLoginFailed:
                upload_hpr_update_feedback_to_server(serOriData, E_UPDATE_LoginFailed);
                update_handle_g.updateSta = updateSta_EventIdle;
            break;

            case updateSta_EventAccessDenied:
                upload_hpr_update_feedback_to_server(serOriData, E_UPDATE_AccessDenied);
                update_handle_g.updateSta = updateSta_EventIdle;            
            break;

            case updateSta_EventFileNotFound://文件未找到
                upload_hpr_update_feedback_to_server(serOriData, E_UPDATE_FileNotFound);
                update_handle_g.updateSta = updateSta_EventIdle;
            break;

            case updateSta_EventInvalidDirectory:
                upload_hpr_update_feedback_to_server(serOriData, E_UPDATE_InvalidDirectory);
                update_handle_g.updateSta = updateSta_EventIdle;
            break;

            case updateSta_EventLocalFileError:
                upload_hpr_update_feedback_to_server(serOriData, E_UPDATE_LocalFileError);
                update_handle_g.updateSta = updateSta_EventIdle;
            break;

            case updateSta_EventError:
                upload_hpr_update_feedback_to_server(serOriData, E_UPDATE_Error);
                update_handle_g.updateSta = updateSta_EventIdle;            
            break;

            default:
            break;
        }
        delay_ms(1000);
		//GLOBAL_PRINT(("updateSta: %d\r\n",update_handle_g.updateSta));
	}
}

/*****************************************************************************
 函 数 名  : update_apl_init
 功能描述  : FTP升级线程初始化函数
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年1月
    作    者   : sunj
    修改内容   : 创建

*****************************************************************************/
void update_apl_init(void)
{
    osThreadId_t thread_update_id = 0;

    GLOBAL_MEMSET(&update_handle_g, 0x0, sizeof(update_handle_g));

    thread_update_id = osThreadNew(_update_thread, NULL, &thread_update_attr);
    GLOBAL_HEX(thread_update_id);
}

