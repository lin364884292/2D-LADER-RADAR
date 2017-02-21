/**
 * @file        pro.h
 * @author      linjiawei
 * @version     V01
 * @date        2016.09.21
 * @brief       协议定义
 * @note
 *
 * @attention   COYPRIGHT INMOTION ROBOT
 **/

#ifndef _PRO_H_
#define _PRO_H_

#define u8               unsigned char
#define u16              unsigned short
#define u32              unsigned int
#define s16              short
#define LIDAR_ADDRESS    0x10
#define PARSE_LEN           2048    //>1036
#define MIN_PRO_NUM      14

typedef enum
{
    PACK_FAIL,
    PACK_OK
} ResultTypeDef;

#define NULL    0


typedef struct
{
    u8  DeviceAddr;   
    u8  FunctionCode; 
    u16 StartAddr;    
    u32 Len;
} SdkProtocolHeaderTypeDef;


//数据包头尾、控制字
#define P_HEADER     0xAA
#define P_TAIL       0x55
#define P_CTRL       0xA5
#define P_FAIL       0
#define P_SUCCESS    1


/**
 * @brief  数据包ID
 */
typedef enum
{
    PACK_LIDAR_DATA,
    PACK_SET_PIXOFFSET,      /*!< 校准像素偏移量  */
    PACK_FLASH_CONFIG,       /*!< 烧录配置到flash  */
    PACK_GET_GYROSCOPE_DATA, /*!< 获取陀螺仪数据 */
    PACK_SET_SPEED,          /*!< 设置Lidar速度 */
    PACK_FIRMWARE_UPDATE,             /*!< 更新Firmware */
    PACK_ANGLE_OFFSET,       /*!< 调整角度偏移量*/
    PACK_CONTROL_LASER,      /*!< 控制激光*/
    PACK_DEBUG_MODE,         /*!< 调试模式*/
    PACK_START_ROTATE,       /*!< 开始旋转 */
	PACK_ACK = 0xfd,
    PACK_SELF_TEST = 0xfe,   /*!< 自检模式 */
    PACK_NULL = 0xff         /*!< 复位值，表明当前没有数据包 */
} PackageIDTypeDef;

/**
 * @brief  调试模式
 */
typedef enum
{
    DEBUG_GET_STATIC_DIS,
    DEBUG_GET_MAX_PIXEL_VALUE,
    DEBUG_GET_CENTRIOD,
    DEBUG_GET_TEMP,
    DEBUG_ENTER_COM_TEST,
    DEBUG_GET_FIRMWARE_VERSION,
    DEBUG_STOP_ROTATE,
    DEBUG_START_ROTATE,
    DEBUG_GET_WHOLE_PIXEL_VALUE,
    DEBUG_GET_LASER_CURRENT,
	DEBUG_GET_CONFIG,
    DEBUG_QUIT,
    DEBUG_NULL = 0xFF,
} DebugTypeDef;



typedef struct
{
    PackageIDTypeDef DataID;
    u8               *DataInBuff;
    u32              DataInLen;
    u8               *DataOutBuff;
    u32              *DataOutLen;
} PackageDataStruct;


#endif



/************************ (C) COPYRIGHT INMOTION ROBOT *****END OF FILE****/
