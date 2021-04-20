#ifndef _MQTTCLIENT_H__
#define _MQTTCLIENT_H__

#include "pubDef.h"

typedef struct MQTTClient
{
    // unsigned int next_packetid,
    // command_timeout_ms;
    // U32 buf_size,
    // readbuf_size;
    // unsigned char *buf,
    // *readbuf;
    // unsigned int keepAliveInterval;
    // char ping_outstanding;
    int isconnected;
//     int cleansession;

//     struct MessageHandlers
//     {
//         const char* topicFilter;
//         void (*fp) (MessageData*);
//     } messageHandlers[MAX_MESSAGE_HANDLERS];      /* Message handlers are indexed by subscription topic */

//     void (*defaultMessageHandler) (MessageData*);

//     Network* ipstack;
//     Timer last_sent, last_received;
// #if defined(MQTT_TASK)
//     Mutex mutex;
//     Thread thread;
// #endif
} MQTTClient;



#endif
