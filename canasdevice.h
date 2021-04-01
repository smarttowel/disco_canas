#pragma once

#include <vector>
#include <chrono>
#include <map>
#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "canas/canas.h"
#include "canas/circularbuffer.h"

using namespace canas;

class CanasDevice
{
public:
    static const int NODE_ID = 1;
    struct Stream {
        uint16_t interval = 0;
        uint32_t tp = 0;
    };
    enum DdsState {
        None,
        ReceivedBlob,
    };
    CanasDevice();
    ~CanasDevice();

    TaskHandle_t *getTaskHandle() const;
    void start(UART_HandleTypeDef *uart, QueueHandle_t queue);
    void run();

private:
    UART_HandleTypeDef *m_uart = nullptr;
    SemaphoreHandle_t m_ioMutex;
    QueueHandle_t m_queue;
    std::map<int, Stream> m_streams;

    //yaw
    ptuYawPos_t m_yawPos;
    ptuYawSpeed_t m_yawSpeed;
    ptuYawEngineState_t m_yawEngineState;
    ptuYawEngineTemp_t m_yawEngineTemp;
    ptuYawBrakes_t m_yawBrakes;

    //pitch
    ptuPitchPos_t m_pitchPos;
    ptuPitchSpeed_t m_pitchSpeed;
    ptuPitchEngineState_t m_pitchEngineState;
    ptuPitchEngineTemp_t m_pitchEngineTemp;
    ptuPitchBrakes_t m_pitchBrakes;

    //general
    ptuGunShutterState_t m_shutterState;
    ptuFanState_t m_fanState;
    ptuGeneralTemperature_t m_genTemp;

    template<typename T>
    void sendIdsUavosPacket(IdsUavosType type, const char *data);
    template<typename T>
    void sendPacket(const T &packet);
    template<typename T, typename U>
    void sendValue(T &packet, U v);
    uint32_t msecsNow();
};

#include "canasdevice_timpl.h"