#ifndef __DATA_PROC_DEF_H
#define __DATA_PROC_DEF_H

#include <stdint.h>

namespace AccountSystem
{

/* Recorder */
typedef enum
{
    RECORDER_CMD_START,
    RECORDER_CMD_PAUSE,
    RECORDER_CMD_CONTINUE,
    RECORDER_CMD_STOP,
} Recorder_Cmd_t;

typedef struct
{
    Recorder_Cmd_t cmd;
    uint16_t time;
} Recorder_Info_t;

/* MusicPlayer */
typedef struct
{
    const char* music;
} MusicPlayer_Info_t;

/* Storage */
typedef enum
{
    STORAGE_CMD_LOAD,
    STORAGE_CMD_SAVE,
    STORAGE_CMD_ADD,
    STORAGE_CMD_REMOVE
} Storage_Cmd_t;

typedef enum
{
    STORAGE_TYPE_UNKNOW,
    STORAGE_TYPE_INT,
    STORAGE_TYPE_FLOAT,
    STORAGE_TYPE_DOUBLE,
    STORAGE_TYPE_STRING
} Storage_Type_t;

typedef struct
{
    Storage_Cmd_t cmd;
    const char* key;
    void* value;
    uint16_t size;
    Storage_Type_t type;
} Storage_Info_t;

#define STORAGE_VALUE_REG(act, data, dataType)\
do{\
    AccountSystem::Storage_Info_t info; \
    info.cmd = AccountSystem::STORAGE_CMD_ADD; \
    info.key = #data; \
    info.value = &data; \
    info.size = sizeof(data); \
    info.type = dataType; \
    act->Notify("Storage", &info, sizeof(info)); \
}while(0)

typedef struct
{
    bool isDetect;
    float totalSizeMB;
    float freeSizeMB;
} Storage_Basic_Info_t;

/* StatusBar */
typedef struct
{
    bool showLabelRec;
    const char* labelRecStr;
} StatusBar_Info_t;

/* Motor */
typedef enum
{
    MOTOR_CMD_CHANGE_MODE,
    MOTOR_CMD_CHECKOUT_PAGE,
} Motor_Cmd_t;

typedef struct
{
    Motor_Cmd_t cmd;
    int motor_mode;
    int init_position;
    int motor_num;
} Motor_Info_t;

typedef struct {
    int running_mode;
}BotStatusInfo;

/* SysConfig */
typedef enum
{
    SYSCONFIG_CMD_LOAD,
    SYSCONFIG_CMD_SAVE,
} SysConfig_Cmd_t;

typedef struct
{
    SysConfig_Cmd_t cmd;
    bool soundEnable;
    double longitudeDefault;
    double latitudeDefault;
    char language[8];
    char mapDirPath[32];
    bool WGS84;
    char arrowTheme[8];
} SysConfig_Info_t;

/* TrackFilter */
typedef enum
{
    TRACK_FILTER_CMD_START = RECORDER_CMD_START,
    TRACK_FILTER_CMD_PAUSE = RECORDER_CMD_PAUSE,
    TRACK_FILTER_CMD_CONTINUE = RECORDER_CMD_CONTINUE,
    TRACK_FILTER_CMD_STOP = RECORDER_CMD_STOP,
} TrackFilter_Cmd_t;

typedef struct
{
    float longitude;
    float latitude;
} TrackFilter_Point_t;

typedef struct
{
    TrackFilter_Cmd_t cmd;
    TrackFilter_Point_t* points;
    uint32_t size;
    bool isActive;
} TrackFilter_Info_t;

}

#endif
