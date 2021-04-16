#ifndef _UPDATE_APL_H__
#define _UPDATE_APL_H__

#include "project_def.h"

typedef enum
{
    updateSta_EventIdle,
    updateSta_EventDownloadStart,
    updateSta_EventDownloading,
    updateSta_EventSuccess,                 ///< File operation successful
    updateSta_EventTimeout,                 ///< Timeout on file operation
    updateSta_EventLoginFailed,             ///< Login error, username/password invalid
    updateSta_EventAccessDenied,            ///< File access not allowed
    updateSta_EventFileNotFound,            ///< File not found
    updateSta_EventInvalidDirectory,        ///< Working directory path not found
    updateSta_EventLocalFileError,          ///< Local file read/write error
    updateSta_EventError                    ///< Generic FTP client error 
}UPDATE_STA_Event;

typedef enum
{
    E_UPDATE_Success = 0,             ///< File operation successful
    E_UPDATE_Timeout,                 ///< Timeout on file operation
    E_UPDATE_LoginFailed,             ///< Login error, username/password invalid
    E_UPDATE_AccessDenied,            ///< File access not allowed
    E_UPDATE_FileNotFound,            ///< File not found
    E_UPDATE_InvalidDirectory,        ///< Working directory path not found
    E_UPDATE_LocalFileError,          ///< Local file read/write error
    E_UPDATE_Error,                   ///< Generic FTP client error
    E_UPDATE_CheckFail,               ///< У��ʧ�� 
}E_UPDATE_STATUS;

typedef struct
{
    U8 ftpServerAddr[4];    //FTP��������ַ
    char aFtpServerAddr[20];//FTP��������ַIP�ַ���
    char userName[12];      //�û���
    char password[12];      //����
    char workDir[20];       //����Ŀ¼
    char fileName[40];      //�ļ�����
    U8 updateSta;           //����״̬
    U8 reserved[3];         //Ԥ��

}UPDATE_HANDLE;

extern UPDATE_HANDLE update_handle_g;

extern void update_apl_init(void);

#endif
