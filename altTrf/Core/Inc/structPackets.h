#include "string.h"
#include "stdbool.h"
#include "stdint.h"
#ifndef structPackets_h
#define structPackets_h

#define movementPacketNum 1
#define PositionPacketNum 2
#define AccelPacketNum 3
#define HumTemPresPacketNum 4
#define requestPacketNum 20

struct paket_s
{
    char packetNum;
    char paketLen;
    char paketType;
};
typedef struct paket_s PacketStruct;

typedef struct movementData
{
    char direction;
    char speed;
    char rotation;
    bool buzzer;
} mvmdt;
static struct movement_s
{
    PacketStruct paket;
    mvmdt data;

} movementPktDefault = {
    .paket.paketLen = sizeof(mvmdt),
    .paket.paketType = 4,
    .paket.packetNum = movementPacketNum,
    .data.direction = 2,
    .data.speed = 100,
    .data.rotation = 125,
    .data.buzzer = false,
};

typedef struct movement_s movementPKT;
//-----------------------------
typedef struct positiondata
{
    char x;
    char y;
    char z;
    char heading;
    char ben;
} pstndt;
static struct position_s
{
    PacketStruct paket;
    pstndt data;

} positionPktDefault = {
    .paket.paketLen = sizeof(pstndt),
    .paket.paketType = 3,
    .paket.packetNum = PositionPacketNum,
    .data.x = 25,
    .data.y = 108,
    .data.z = 200,
    .data.heading = 80,
    .data.ben = 70,
};

typedef struct position_s positionPKT;
//------------------------------

typedef struct acceldata
{
	int8_t accelx;
	int8_t accely;
	int8_t accelz;
} accldt;
static struct accel_s
{
    PacketStruct paket;
    accldt data;

} accelPktDefault = {
    .paket.paketLen = sizeof(accldt),
    .paket.paketType = 3,
    .paket.packetNum = AccelPacketNum,
    .data.accelx = 1,
    .data.accely = 2,
    .data.accelz = 3,
};

typedef struct accel_s accelPKT;
//-------------------------------
typedef struct humtempresdata
{
    char humidity;
    float temperature;
    unsigned int pressure;
} htpdt;
static struct humtempres_s
{
    PacketStruct paket;
    htpdt data;

} humtempresPktDefault = {
    .paket.paketLen = sizeof(htpdt),
    .paket.paketType = 3,
    .paket.packetNum = HumTemPresPacketNum,
    .data.humidity = 65,
    .data.temperature = 27.4,
    .data.pressure = 19000,
};

typedef struct humtempres_s humtempresPKT;
//---------------------------
typedef struct requestdata
{
    char request;
} reqdt;
static struct request_s
{
    PacketStruct paket;
    reqdt data;

} requestPktDefault = {
    .paket.paketLen = sizeof(reqdt),
    .paket.paketType = 3,
    .paket.packetNum = requestPacketNum,
    .data.request = 1,
};

typedef struct request_s requestPKT;
//---------------------------
void GetMovementPacket(movementPKT *movement, uint8_t *bytearray);
void GetPositionPacket(positionPKT *position, uint8_t *bytearray);
void GetAccellerationPacket(accelPKT *accelleration, uint8_t *bytearray);
void GetHumTempResPacket(humtempresPKT *humtempres, uint8_t *bytearray);
void GetRequestPacket(requestPKT *request, uint8_t *bytearray);

uint8_t *CreateMovementPacketToByteArray(char direction, char speed, char rotation, bool buzzer);
uint8_t *CreatePositionPacketToByteArray();
uint8_t *CreateAccellerationPacketToByteArray(char accelx, char accely, char accelz);
uint8_t *CreateHumTempResPacketToByteArray(char humidity, float temperature, unsigned int pressure);
uint8_t *CreateRequestToByteArray(char request);
#endif
