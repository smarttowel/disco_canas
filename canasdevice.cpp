#include "canasdevice.h"

#include <cstring>
#include <atomic>
#include <stdio.h>
#include <vector>
#include <functional>
#include <chrono>
#include "canas/slip.h"
#include "timers.h"

using namespace canas;

void wrapper(void *params)
{
    static_cast<CanasDevice *>(params)->run();
}

CanasDevice::CanasDevice()
{
    m_ioMutex = xSemaphoreCreateMutex();
}

CanasDevice::~CanasDevice()
{
    vSemaphoreDelete(m_ioMutex);
}

void CanasDevice::start(UART_HandleTypeDef *uart, QueueHandle_t queue)
{
    m_uart = uart;
    m_queue = queue;
    xTaskCreate(wrapper, "canasdevice", 1000, this, osPriorityRealtime, nullptr);
}

void CanasDevice::run()
{
    CircularBuffer<std::byte, 500> buffer;
    DdsState ddsState = None;
    int ddsMicroblobsCount = 0;
    int ddsCurrentMicroblob = 0;
    uint16_t ddsCrc = crc16ibm::INIT_VALUE;
    while(true) {
        std::byte byte;
        while(xQueueReceive(m_queue, &byte, 0) == pdTRUE) {
            buffer.push_back(byte);
        }
        auto inSlipPacket = slip::findPacketInByteStream(buffer);
        while(inSlipPacket) {
            bool crcOk = false;
            auto rawPacket = slip::fromSlipEncoding(inSlipPacket.value(), crcOk);
            if(crcOk) {
                auto id = getIdFromRaw(rawPacket);
                if(id == ptuServiceRequest || id == ptuServiceRequestHigh) {
                    uint8_t srvCode = getSrvCodeFromRaw(rawPacket);
                    if(srvCode == TIS) {
                        auto request = deserialize<ptuTisRequest_t>(rawPacket);
                        if(request.nodeId == 0 || request.nodeId == NODE_ID) {
                            uint16_t id = request.data[0];
                            uint16_t interval = request.data[1];
                            auto &stream = m_streams[id];
                            stream.interval = interval;
                            ptuTisResponse_t response;
                            response.messageCode = OK;
                            sendPacket(response);
                            printf("%d %d\n", int(id), int(interval));
                        }
                    } else if(srvCode == IDS) {
                        auto request = deserialize<ptuIdsRequest_t>(rawPacket);
                        if(request.nodeId == 0 || request.nodeId == NODE_ID) {
                            ptuIdsResponse_t response;
                            response.data[0] = 42;
                            response.data[1] = 43;
                            response.nodeId = 1;
                            sendPacket(response);
                        }
                    } else if(srvCode == IDS_UAVOS) {
                        auto request = deserialize<ptuIdsUavosRequest_t>(rawPacket);
                        if(request.nodeId == 0 || request.nodeId == NODE_ID) {
                            std::string name = "Disco";
                            std::string description = "STM32F7Discovery";
                            uint8_t softwareVersion[3] = {1, 2, 3};
                            uint8_t hardwareVersion[3] = {4, 5, 6};
                            sendIdsUavosPacket<ptuIdsUavosResponse4_t>(NamePart1, name.c_str());
                            sendIdsUavosPacket<ptuIdsUavosResponse1_t>(NamePart2, name.c_str() + 4);
                            sendIdsUavosPacket<ptuIdsUavosResponse4_t>(DescriptionPart1, description.c_str());
                            sendIdsUavosPacket<ptuIdsUavosResponse4_t>(DescriptionPart2, description.c_str() + 4);
                            sendIdsUavosPacket<ptuIdsUavosResponse4_t>(DescriptionPart3, description.c_str() + 8);
                            sendIdsUavosPacket<ptuIdsUavosResponse4_t>(DescriptionPart4, description.c_str() + 12);
                            sendIdsUavosPacket<ptuIdsUavosResponse3_t>(SoftwareVersion, (char *)&softwareVersion);
                            sendIdsUavosPacket<ptuIdsUavosResponse3_t>(HardwareVersion, (char *)&hardwareVersion);
                        }
                    } else if(srvCode == FPS) {
                        auto request = deserialize<ptuFpsRequest_t>(rawPacket);
                        if(request.nodeId == 0 || request.nodeId == NODE_ID) {
                            ptuFpsResponse_t response;
                            response.messageCode = OK;
                            response.nodeId = NODE_ID;
                            sendPacket(response);
                        }
                    } else if(srvCode == DDS) {
                        auto nodeId = getNodeIdFromRaw(rawPacket);
                        if(nodeId == 0 || nodeId == NODE_ID) {
                            if(ddsState == None) {
                                auto request = deserialize<ptuDdsRequestInit_t>(rawPacket);
                                ddsMicroblobsCount = request.messageCode;
                                ptuDdsResponse_t response;
                                response.messageCode = request.messageCode;
                                response.nodeId = NODE_ID;
                                response.data = XON;
                                sendPacket(response);
                                ddsState = ReceivedBlob;
                                ddsCurrentMicroblob = 0;
                                ddsCrc = crc16ibm::INIT_VALUE;
                            } else if(ddsState == ReceivedBlob) {
                                uint8_t *data = (uint8_t *)rawPacket.data() + PACKET_MIN_SIZE;
                                uint8_t dlc = getDlcFromRaw(rawPacket);
                                auto msgCode = getMsgCodeFromRaw(rawPacket);
                                if(dlc > 4) {
                                    for(int i = 4; i < dlc; i++) {
                                        crc16ibm::calc(std::byte(data[i - 4]), ddsCrc);
                                    }
                                    ddsCurrentMicroblob++;
                                    if(ddsCurrentMicroblob == ddsMicroblobsCount) {
                                        ptuDdsResponseFinal_t response;
                                        response.messageCode = msgCode;
                                        response.nodeId = NODE_ID;
                                        response.data = ddsCrc;
                                        sendPacket(response);
                                        ddsState = None;
                                    }
                                } else {
                                    ptuDdsResponseFinal_t response;
                                    response.messageCode = msgCode;
                                    response.nodeId = NODE_ID;
                                    response.data = INVALID;
                                    sendPacket(response);
                                    ddsState = None;
                                }
                            }
                        }
                    } else {
                        printf("unknown srv code: %d\n", int(srvCode));
                    }
                }
                slip::truncateByteStream(buffer, inSlipPacket.value());
            } else {
                printf("crc error\n");
                slip::truncateByteStream(buffer);
            }
            inSlipPacket = slip::findPacketInByteStream(buffer);
        }
        auto now = msecsNow();
        for(auto &stream: m_streams) {
            if(now - stream.second.tp >= stream.second.interval && stream.second.interval > 0) {
                if(stream.first == ptuYawPos) {
                    sendValue(m_yawPos, float(1));
                } else if(stream.first == ptuYawSpeed) {
                    sendValue(m_yawSpeed, float(2));
                } else if(stream.first == ptuYawEngineState) {
                    sendValue(m_yawEngineState, 1);
                } else if(stream.first == ptuYawBrakes) {
                    sendValue(m_yawBrakes, 1);
                } else if(stream.first == ptuYawEngineTemp) {
                    sendValue(m_yawEngineTemp, 30);
                } else if(stream.first == ptuPitchPos) {
                    sendValue(m_pitchPos, float(3));
                } else if(stream.first == ptuPitchSpeed) {
                    sendValue(m_pitchSpeed, float(4));
                } else if(stream.first == ptuPitchEngineState) {
                    sendValue(m_pitchEngineState, 2);
                } else if(stream.first == ptuPitchBrakes) {
                    sendValue(m_pitchBrakes, 1);
                } else if(stream.first == ptuPitchEngineTemp) {
                    sendValue(m_pitchEngineTemp, 40);
                } else if(stream.first == ptuGunShutterState) {
                    sendValue(m_shutterState, 1);
                } else if(stream.first == ptuFanState) {
                    sendValue(m_fanState, 1);
                } else if(stream.first == ptuGeneralTemperature) {
                    sendValue(m_genTemp, 50);
                }
                stream.second.tp = now;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

uint32_t CanasDevice::msecsNow()
{
    return xTaskGetTickCount(); // / float(configTICK_RATE_HZ / 1000);
}