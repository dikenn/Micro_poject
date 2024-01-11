#include "structPackets.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"

void GetMovementPacket(movementPKT *movement, uint8_t *bytearray)
{
    memcpy(&movement->data, bytearray, sizeof(mvmdt));
}
void GetPositionPacket(positionPKT *position, uint8_t *bytearray)
{
    memcpy(&position->data, bytearray, sizeof(pstndt));
}
void GetAccellerationPacket(accelPKT *accelleration, uint8_t *bytearray)
{
    memcpy(&accelleration->data, bytearray, sizeof(accldt));
}
void GetHumTempResPacket(humtempresPKT *humtempres, uint8_t *bytearray)
{
    memcpy(&humtempres->data, bytearray, sizeof(htpdt));
}
void GetRequestPacket(requestPKT *request, uint8_t *bytearray)
{
    memcpy(&request->data, bytearray, sizeof(reqdt));
}
uint8_t *CreateMovementPacketToByteArray(char direction, char speed, char rotation, bool buzzer)
{
    movementPKT movement;
    movement.paket.paketLen = movementPktDefault.paket.paketLen;
    movement.paket.paketType = movementPktDefault.paket.paketType;
    movement.paket.packetNum = movementPktDefault.paket.packetNum;
    movement.data.direction = direction;
    movement.data.speed = speed;
    movement.data.rotation = rotation;
    movement.data.buzzer = buzzer;
    uint8_t *paket = malloc(sizeof(mvmdt) + 2);
    memcpy(paket + 2, &movement.data, sizeof(mvmdt));
    paket[1] = movement.paket.packetNum;
    paket[0] = movement.paket.paketLen;
    return paket;
}

uint8_t *CreatePositionPacketToByteArray()
{
    positionPKT position;
    position.paket.paketLen = positionPktDefault.paket.paketLen;
    position.paket.paketType = positionPktDefault.paket.paketType;
    position.paket.packetNum = positionPktDefault.paket.packetNum;
    position.data.x = positionPktDefault.data.x;
    position.data.y = positionPktDefault.data.y;
    position.data.z = positionPktDefault.data.z;
    position.data.ben = positionPktDefault.data.ben;
    position.data.heading = positionPktDefault.data.heading;
    uint8_t *paket = malloc(sizeof(pstndt) + 2);
    memcpy(paket + 2, &position.data, sizeof(pstndt));
    paket[0] = position.paket.paketLen;
    paket[1] = position.paket.packetNum;
    return paket;
}

uint8_t *CreateAccellerationPacketToByteArray(char accelx, char accely, char accelz)
{
    accelPKT accelleration;
    accelleration.paket.paketLen = accelPktDefault.paket.paketLen;
    accelleration.paket.paketType = accelPktDefault.paket.paketType;
    accelleration.paket.packetNum = accelPktDefault.paket.packetNum;
    accelleration.data.accelx = accelx;
    accelleration.data.accely = accely;
    accelleration.data.accelz = accelz;
    uint8_t *paket = malloc(sizeof(accldt) + 2);
    memcpy(paket + 2, &accelleration.data, sizeof(accldt));
    paket[0] = accelleration.paket.paketLen;
    paket[1] = accelleration.paket.packetNum;
    return paket;
}

uint8_t *CreateHumTempResPacketToByteArray(char humidity,float temperature,unsigned int pressure)
{
    humtempresPKT humtempres;
    humtempres.paket.paketLen = humtempresPktDefault.paket.paketLen;
    humtempres.paket.paketType = humtempresPktDefault.paket.paketType;
    humtempres.paket.packetNum = humtempresPktDefault.paket.packetNum;
    humtempres.data.humidity = humtempresPktDefault.data.humidity;
    humtempres.data.temperature = humtempresPktDefault.data.temperature;
    humtempres.data.pressure = humtempresPktDefault.data.pressure;
    uint8_t *paket = malloc(sizeof(htpdt) + 2);
    memcpy(paket + 2, &humtempres.data, sizeof(htpdt));
    paket[0] = humtempres.paket.paketLen;
    paket[1] = humtempres.paket.packetNum;
    return paket;
}
uint8_t *CreateRequestToByteArray(char request){
    requestPKT req;
    req.paket.paketLen = requestPktDefault.paket.paketLen;
    req.paket.paketType = requestPktDefault.paket.paketType;
    req.paket.packetNum = requestPktDefault.paket.packetNum;
    req.data.request = request;
    uint8_t *paket = malloc(sizeof(reqdt) + 2);
    memcpy(paket + 2, &req.data, sizeof(reqdt));
    paket[0] = req.paket.paketLen;
    paket[1] = req.paket.packetNum;
    return paket;
}