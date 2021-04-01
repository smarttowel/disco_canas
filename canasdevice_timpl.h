#include "canasdevice.h"

#include <cstring>
#include <array>
#include "canas/canas.h"
#include "canas/slip.h"

template<typename T>
void CanasDevice::sendPacket(const T &packet)
{
    auto serializedPacket = serialize(packet);
    auto outSlipPacket = slip::toSlipEncoding(serializedPacket);
    HAL_UART_Transmit(m_uart, (uint8_t*)outSlipPacket.data(), outSlipPacket.size(), 10);
}

template<typename T, typename U>
void CanasDevice::sendValue(T &packet, U v)
{
    packet.messageCode++;
    packet.data = v;
    sendPacket(packet);
}

template<typename T>
void CanasDevice::sendIdsUavosPacket(IdsUavosType type, const char *data)
{
    T response;
    response.nodeId = NODE_ID;
    response.messageCode = type;
    memcpy(&response.data, data, canas::PAYLOAD_SIZE<typename T::PayloadType>);
    sendPacket(response);
}