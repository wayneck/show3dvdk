#include "../proto/adapter_config.pb.h"

using namespace watrix::proto;

#define PACKET_TYPE  AdapterConfig_MessageType
#define PACKET_TYPE_(name)  AdapterConfig_MessageType_##name

typedef struct
{
    uint total_len;	//Buff总的大小
    uint buffer_len;	//有效数据的长度
    uint time_stamp;
    uint height;
    uint width;
    char name[64];
    PACKET_TYPE type;
}NetPacketHeader;
