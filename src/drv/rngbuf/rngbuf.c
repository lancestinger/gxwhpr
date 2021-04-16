/******************************************************************************
**
**                   
**
*******************************************************************************
** File Name      : RngBuf.c
** Version        : v1.0
** Author         : wenzhiquan
** Created        : 2014.9.25
** Modification   : Created file
** Description    : 环形缓冲区
******************************************************************************/

/* Header File */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "RngBuf.h"
#include "pubdef.h"

/* Macro Define */
#define min(X,Y) (((X) < (Y)) ? (X) : (Y))

/* Global variable and struct/enum/union define */

/* Function List*/
/*****************************************************************************
 Function			:rng_create
 Description		:创建缓冲区
 Date			:2014/9/25
 Author			:wenzhiquan
 Modification		:Created
*****************************************************************************/
RNGBUF_t * rng_create(int nbytes)
{
	char * buffer;
	RNGBUF_t * pBuf = (RNGBUF_t *)malloc(sizeof (RNGBUF_t));

	if (!pBuf)
	{
		return(NULL);
	}
	buffer = (char *)malloc(++nbytes);
	if (buffer == NULL)
	{
		free(pBuf);
		return(NULL);
	}
	
	pBuf->bufSize = nbytes;
	pBuf->buf = buffer;
	rng_flush(pBuf);
	
	return(pBuf);
}


/*****************************************************************************
 Function			:rng_delete
 Description		:销毁缓冲区
 Date			:2014/9/25
 Author			:wenzhiquan
 Modification		:Created
*****************************************************************************/
void rng_delete( RNGBUF_t * pBuf)
{
	if(!pBuf)return;
	free(pBuf->buf);
	free(pBuf);
}


/*****************************************************************************
 Function			:rng_flush
 Description		:清空缓冲区
 Date			:2014/9/25
 Author			:wenzhiquan
 Modification		:Created
*****************************************************************************/
void rng_flush( RNGBUF_t * pBuf)
{
	if(!pBuf)return;
    pBuf->pWritePos = 0;
    pBuf->pReadPos = 0;
}


/*****************************************************************************
 Function			:rng_get_buf
 Description		:向缓冲区取数据
 Date			:2014/9/25
 Author			:wenzhiquan
 Modification		:Created
*****************************************************************************/
int rng_get_buf( RNGBUF_t * pBuf,char *buffer, int maxbytes)
{
	if(!pBuf)return FALSE;
	int bytesgot = 0;
	int pWritePos = pBuf->pWritePos;
	int bytes2;
	int pRngTmp = 0;

	if (pWritePos >= pBuf->pReadPos)
	{
		/* pWritePos has not wrapped around */

		bytesgot = min(maxbytes, pWritePos - pBuf->pReadPos);
		memcpy(buffer,&pBuf->buf[pBuf->pReadPos],bytesgot);
		pBuf->pReadPos += bytesgot;
	}
	else
	{
		/* pWritePos has wrapped around.  Grab chars up to the end of the
		* buffer, then wrap around if we need to. */

		bytesgot = min(maxbytes, pBuf->bufSize - pBuf->pReadPos);
		memcpy( buffer,&pBuf->buf [pBuf->pReadPos], bytesgot);
		pRngTmp = pBuf->pReadPos + bytesgot;

		/* If pReadPos is equal to bufSize, we've read the entire buffer,
		* and need to wrap now.  If bytesgot < maxbytes, copy some more chars
		* in now. */

		if (pRngTmp == pBuf->bufSize)
		{
			bytes2 = min(maxbytes - bytesgot, pWritePos);
			memcpy( buffer + bytesgot,pBuf->buf, bytes2);
			pBuf->pReadPos = bytes2;
			bytesgot += bytes2;
		}
		else
			pBuf->pReadPos = pRngTmp;
	}
	return(bytesgot);
}


/*****************************************************************************
 Function			:rng_put_buf
 Description		:向缓冲区中放数据(非覆盖)
 Date			:2014/9/25
 Author			:wenzhiquan
 Modification		:Created
*****************************************************************************/
int rng_put_buf( RNGBUF_t * pBuf,char *buffer,int nbytes)
{
	if(!pBuf)return FALSE;
	int bytesput = 0;
	int pReadPos = pBuf->pReadPos;
	int bytes2;
	int pRngTmp = 0;

	if (pReadPos > pBuf->pWritePos)
	{
		/* pReadPos is ahead of pWritePos.  We can fill up to two bytes
		* before it */

		bytesput = min (nbytes, pReadPos - pBuf->pWritePos - 1);
		memcpy(&pBuf->buf [pBuf->pWritePos],buffer, bytesput);
		pBuf->pWritePos += bytesput;
	}
	else if (pReadPos == 0)
	{
		/* pReadPos is at the beginning of the buffer.  We can fill till
		* the next-to-last element */

		bytesput = min(nbytes, pBuf->bufSize - pBuf->pWritePos - 1);
		memcpy(&pBuf->buf [pBuf->pWritePos],buffer,  bytesput);
		pBuf->pWritePos += bytesput;
	}
	else
	{
		/* pReadPos has wrapped around, and its not 0, so we can fill
		* at least to the end of the ring buffer.  Do so, then see if
		* we need to wrap and put more at the beginning of the buffer. */

		bytesput = min(nbytes, pBuf->bufSize - pBuf->pWritePos);
		memcpy(&pBuf->buf [pBuf->pWritePos],buffer,  bytesput);
		pRngTmp = pBuf->pWritePos + bytesput;

		if (pRngTmp == pBuf->bufSize)
		{
			/* We need to wrap, and perhaps put some more chars */

			bytes2 = min (nbytes - bytesput, pReadPos - 1);
			memcpy( pBuf->buf,buffer + bytesput, bytes2);
			pBuf->pWritePos = bytes2;
			bytesput += bytes2;
		}
		else
			pBuf->pWritePos = pRngTmp;
	}
	return(bytesput);
}


/*****************************************************************************
 Function			:rng_is_empty
 Description		:缓冲区是否空
 Date			:2014/9/25
 Author			:wenzhiquan
 Modification		:Created
*****************************************************************************/
int rng_is_empty(RNGBUF_t * pBuf)
{
	if(!pBuf)return FALSE;
	return(pBuf->pWritePos == pBuf->pReadPos);
}


/*****************************************************************************
 Function			:rng_is_full
 Description		:缓冲区是否满
 Date			:2014/9/25
 Author			:wenzhiquan
 Modification		:Created
*****************************************************************************/
int rng_is_full(RNGBUF_t * pBuf)
{
	if(!pBuf)return FALSE;
	int n = pBuf->pWritePos - pBuf->pReadPos + 1;
	return((n == 0) || (n == pBuf->bufSize));
}


/*****************************************************************************
 Function			:rng_get_free_bytes
 Description		:获取缓冲区剩余空间
 Date			:2014/9/25
 Author			:wenzhiquan
 Modification		:Created
*****************************************************************************/
int rng_get_free_bytes(RNGBUF_t * pBuf)
{
	if(!pBuf)return FALSE;
	int n = pBuf->pReadPos - pBuf->pWritePos - 1;
	if (n < 0)
		n += pBuf->bufSize;
	return(n);
}

/*****************************************************************************
 Function			:rng_get_used_bytes
 Description		:获取缓冲区当前使用量
 Date			:2014/9/25
 Author			:wenzhiquan
 Modification		:Created
*****************************************************************************/
int rng_get_used_bytes(RNGBUF_t * pBuf)
{
	if(!pBuf)return FALSE;
	int n = pBuf->pWritePos - pBuf->pReadPos;

	if (n < 0)
		n += pBuf->bufSize;

	return(n);
}


