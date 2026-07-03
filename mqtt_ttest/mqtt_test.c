#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"

#include "hi_wifi_api.h"
#include "lwip/ip_addr.h"
#include "lwip/netifapi.h"
#include "lwip/sockets.h"

#include "MQTTClient.h"



static MQTTClient mq_client;

 unsigned char *onenet_mqtt_buf;
 unsigned char *onenet_mqtt_readbuf;
 int buf_size;


Network n;
MQTTPacket_connectData data = MQTTPacket_connectData_initializer;  

//消息回调函数
void mqtt_callback(MessageData *msg_data)
{
    size_t res_len = 0;
    uint8_t *response_buf = NULL;
    char topicname[45] = { "$crsp/" };

    LOS_ASSERT(msg_data);

    printf("topic %.*s receive a message\r\n", msg_data->topicName->lenstring.len, msg_data->topicName->lenstring.data);

    printf("message is %.*s\r\n", msg_data->message->payloadlen, msg_data->message->payload);

}

int mqtt_connect(void)
{
	int rc = 0;
    
	NetworkInit(&n);
	rc = NetworkConnect(&n, "broker.emqx.io", 1883);
	if (rc != 0) {
		printf("NetworkConnect failed: %d\r\n", rc);
		return -1;
	}

    buf_size  = 4096+1024;
    onenet_mqtt_buf = (unsigned char *) malloc(buf_size);
    onenet_mqtt_readbuf = (unsigned char *) malloc(buf_size);
    if (!(onenet_mqtt_buf && onenet_mqtt_readbuf))
    {
        printf("No memory for MQTT client buffer!");
        return -2;
    }

	MQTTClientInit(&mq_client, &n, 10000, onenet_mqtt_buf, buf_size, onenet_mqtt_readbuf, buf_size);

	#if defined(MQTT_TASK)
    MQTTStartTask(&mq_client);
	#endif


    data.keepAliveInterval = 30;
    data.cleansession = 1;
	data.clientID.cstring = "mqttx_f60b51dh";
	data.MQTTVersion = 4;
	data.username.cstring = "dinghuan";
	data.password.cstring = "2026";
	data.cleansession = 1;
	
    mq_client.defaultMessageHandler = mqtt_callback;

	//连接服务器
	rc = MQTTConnect(&mq_client, &data);
	if (rc != 0) {
		printf("MQTTConnect failed: %d\r\n", rc);
		NetworkDisconnect(&n);
		return -1;
	}

	//订阅消息，并设置回调函数
	rc = MQTTSubscribe(&mq_client, "ohossub", 0, mqtt_callback);
	if (rc != 0) {
		printf("MQTTSubscribe failed: %d\r\n", rc);
		MQTTDisconnect(&mq_client);
		NetworkDisconnect(&n);
		return -1;
	}

	while(1)
	{
		MQTTMessage message;

		message.qos = QOS0;
		message.retained = 0;
		message.payload = (void *)"openharmony";
		message.payloadlen = strlen("openharmony");

		//发送消息
		rc = MQTTPublish(&mq_client, "ohospub", &message);
		if (rc < 0)
		{
			printf("MQTTPublish failed: %d\r\n", rc);
		}

		MQTTYield(&mq_client, 1000);
		usleep(1000000);
	}

	return 0;
}


void mqtt_test(void)
{
    mqtt_connect();
}

