
#ifndef _RANGING_H_
#define _RANGING_H_

#include "sysconfig.h"

#define _PRINTF_ENABLE 0 //调试用 0--不使用printf调试(重定向到usart)，1--使用print调试

#define MAX_DISTANCE 8000 //最大有效距离限制

//下面两个参数根据CCD有效像素数量来定
#define VALID_PIX_START	 5 //有效像素的起始位置
#define VALID_PIX_END    1027 //有效像素的结束位置

#define REF_LINE_ADJUST 70  //灰度质心法参考线调节值。提高这个值，将减少点的数量。减少这个值，将增加点的跳动。

//斜线阈值
#define OB_THRESHOLD 7

/**
  * @brief  灰度质心法返回类型
  */
typedef enum
{
    Centroid_FAILED,
    Centroid_SUCCEED
} CentroidResultType;

/**
  * @brief  像素曲线类型
  */
typedef enum
{
    OB_LINE,/*!< 斜线型  */
    W_LINE,/*!< W型  */
    ST_LINE,/*!< 直线型  */
    M_LINE,/*!< M型 */
    ERROR_LINE/*!< 错误类型  */
}PixCurveType;

/**
  * @brief  距离与像素对应表结构
  */
typedef struct
{
    float distance;
    float pixel_index;
}DISTANCE_INDEX_FLOAT;

/**
  * @brief  像素曲线坐标
  */
typedef struct
{
    u16 PixI;
    u16 PixV;
}PixCurveCoor;

/**
  * @brief  光斑连通域
  */
typedef struct
{
    u16 head;
    u16 tail;
}ConDom;



u8 GetCentroid(u16 *camera_data_array, float *valid_index);
CentroidResultType Get_Average_Centroid(u16 *data, float *valid_index);
void GetDistance(float index, float *distance);
void GetPixCurveInfo(u16 *data);
PixCurveType GetCurveType(u16 *data, PixCurveCoor *head, PixCurveCoor *tail, PixCurveCoor *mid);
PixCurveCoor GetPixVmax(void);

#endif
