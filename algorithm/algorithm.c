#include "algorithm.h"
#include "stdio.h"

static PixCurveCoor PixMax = { 0, 0 };//最大值坐标
static PixCurveCoor PixMin = { 0, 0xffff };//最小值坐标
static ConDom ConDomMax;//最大值所在连通域
static u32 PixVavg = 0;// 像素值平均值


//partial optimize
//#pragma push 
//#pragma O3
/**
  * @brief  灰度质心法求像素曲线的质心位置
  * @param  data: 指向存储像素的数组
  * @param	valid_index: 返回灰度质心位置
  * @retval 获取像素质心位置成功或者失败
  */
CentroidResultType GetCentroid(u16 *data, float *valid_index)
{
    u16 i;
    u16 translation_temp = 0;
    u32 sum = 0, weight_sum = 0;
    u16 con_width = 0;
    u16 con_head = 65535, con_tail = 65535;
    u8 type;

    u16 ref_line;

    PixCurveCoor head = { 0, 0 }, tail = { 0, 0 }, mid = { 0, 0 };

   // GetPixCurveInfo(data);
    PixVavg = 1500;
    type = ST_LINE;
    
    //type = GetCurveType(data, &head, &tail, &mid);

    {
        //曲线很平直，排除多峰现象,最大值所在连通域基本就是光斑位置
        ref_line = PixVavg + REF_LINE_ADJUST;
        for (i = VALID_PIX_START; i < VALID_PIX_END; i++) //对整个曲线使用灰度质心法,再计算质心宽度是否会大于连通域宽度
        {
            if (data[i] > ref_line)
            {
                if (data[i + 1] > ref_line)
                {
                    translation_temp = data[i] - ref_line;
                    sum += translation_temp;
                    weight_sum += i*translation_temp;
                }
            }
        }
    }

    if (sum == 0)
        *valid_index = 0xffff;
    else
        *valid_index = (float)weight_sum / (float)sum;

    return Centroid_SUCCEED;
}


/**
  * @brief  直接计算出距离
  * @param  index: 质心位置
  * @param	distance: 返回距离值
  * @retval None
  */
void GetDistance(float index, float *distance)
{
//	if (index <(float)909.3)
//	{
//       *distance =  136666.6/(909.3-index); //f =25
//	}
	if (index <(float)766.3)
	{
		*distance = 87466.6/(766.3-index); // f=16mm
	}
	else
		*distance = 0;
}


/**
  * @brief  像素曲线最大值坐标
  * @param  None
  * @retval 像素曲线最大值坐标
  */
PixCurveCoor GetPixVmax(void)
{
    return PixMax;
}

//partial optimize
//#pragma push 
//#pragma O3   

/**
  * @brief  计算像素曲线信息
  * @note   找到像素最大值，最大值所在位置以及平均值，最小值，最小值所在位置
  最大值所在的连通域
  * @param  data : 像素数据缓冲区
  * @retval None
  */
void GetPixCurveInfo(u16 *data)
{
    u16 i;
    u16 pix_max_v, pix_max_i, pix_min_v, pix_min_i;
    u16 *p, *s;


    pix_max_v = *(data + VALID_PIX_START);
    pix_min_v = 0xffff;
    PixVavg = 0;

    //优化查找时间,相对于for循环普通查找可减少约20us
    p = data + VALID_PIX_START;
    s = data + VALID_PIX_END;
    while (p != s)
    {
        u16 d = *p++; //减少寻址时间
        if (d < pix_min_v)
        {
            pix_min_v = d;
            pix_min_i = p - data - 1;
        }
        else if (d > pix_max_v)
        {
            pix_max_v = d;
            pix_max_i = p - data - 1;
        }
        PixVavg += d;
    }

    PixMax.PixV = pix_max_v;
    PixMax.PixI = pix_max_i;
    PixMin.PixV = pix_min_v;
    PixMin.PixI = pix_min_i;

    PixVavg /= (VALID_PIX_END - VALID_PIX_START);

    //开启三级优化时,若是ccd没有数据，会导致下面的循环卡死
    //所以加这一段判断语句
    if (PixMax.PixI == 0)
        return;
    if (PixMin.PixI == 0)
        return;

    //搜索最大值所在连通域起始位置
    for (i = PixMax.PixI; i > VALID_PIX_START; i--)
    {
        if (data[i] < PixVavg)
        {
            if (data[i - 1] < PixVavg)
            {
                //如果连续两个值都小于平均值，认为已找到连通域起始位置
                ConDomMax.head = i;
                break;
            }
        }
        else//如果一直没有小于平均值，起始位置会到START位置
            ConDomMax.head = i;
    }

    //搜索最大值所在连通域终止位置
    for (i = PixMax.PixI; i < VALID_PIX_END; i++)
    {
        if (data[i] < PixVavg)
        {
            if (data[i + 1] < PixVavg)
            {
                ConDomMax.tail = i;
                break;
            }
        }
        else
            ConDomMax.tail = i;
    }
}

//#pragma pop

/**
  * @brief  判断像素曲线类型
  * @param  data : 像素数据缓冲区
  * @param  head : 返回光斑在像素曲线中的开始位置
  * @param  tail : 返回光斑在像素曲线中的结束位置
  * @param  tail : 返回像素曲线中间某点的坐标
  * @retval 返回类型为以下中的一种
  @arg OB_LINE=oblique 斜的，参考线为头尾构建的斜线加调整量，置信度低
  @arg ST_LINE=straight 直的，参考线为平均值加调整量，置信度高
  @arg M_LINE=M型曲线，两端高中间低，阳光干扰严重，参考线为头尾相连斜线加调整量，置信度最低
  @arg W_LINE=曲线中间高，两端低。判断为光斑刚好落在中间，正常，灰度质心法可以工作。
  */
PixCurveType GetCurveType(u16 *data, PixCurveCoor *head, PixCurveCoor *tail, PixCurveCoor *mid)
{
//    u16 i;
//    u16 dis;
//    u8 status;

//    //如果最大值很大,且出现在2080附近,可以认为有物体接近激光雷达,可按直线类型计算
//    if ((PixMax.PixI > 1600) && (PixMax.PixV > 150))
//        return ST_LINE;

//    //两端是盲区，不应有光斑所以不做滤波
//    //选取20~99像素位置的80个像素平均值作为起始值,index为60
//    for (i = 20; i < 100; i++)
//    {
//        head->PixV += *(data + i);
//    }
//    head->PixV /= 80;
//    head->PixI = 60;
//    //选取2000~2080像素位置的80个像素平均值作为终止值，index为2040
//    for (i = 2000; i < 2080; i++)
//    {
//        tail->PixV += *(data + i);
//    }
//    tail->PixV /= 80;
//    tail->PixI = 2040;
//    //判断最小值位置与曲线中心(index=10000)的距离
//    dis = (PixMin.PixI > 1000) ? (PixMin.PixI - 1000) : (1000 - PixMin.PixI);
//    //如果距离不是很远，以最小值所在位置为中心
//    if (dis < 300)
//    {
//        for (i = PixMin.PixI - 40; i < PixMin.PixI + 40; i++)
//        {
//            mid->PixV += *(data + i);
//        }
//        mid->PixV /= 80;
//        mid->PixI = PixMin.PixI;
//    }
//    else
//    {
//        for (i = 1000 - 40; i < 1000 + 40; i++)
//        {
//            mid->PixV += *(data + i);
//        }
//        mid->PixV /= 80;
//        mid->PixI = 1000;
//    }


//    //根据head,tail,mid的情况，判断曲线类型
//    dis = (head->PixV > tail->PixV) ? (head->PixV - tail->PixV) : (tail->PixV - head->PixV);
//    //如果头尾相差过大，可直接判断为斜线
//    if (dis > OB_THRESHOLD)
//        return OB_LINE;
//    else
//    {
//        //如果头尾相差很小，再判断中间
//        dis = (head->PixV > mid->PixV) ? (head->PixV - mid->PixV) : (mid->PixV - head->PixV);
//        status = (head->PixV > mid->PixV) ? 1 : 0;
//        if ((dis > OB_THRESHOLD) && (status == 1))//中间低
//            return M_LINE;
//        if ((dis > OB_THRESHOLD) && (status == 0))//中间高
//            return W_LINE;
//        if (dis <= OB_THRESHOLD)//中间平
//            return ST_LINE;
//    }
//    return ERROR_LINE;//防编译器警告

 return ST_LINE;
}