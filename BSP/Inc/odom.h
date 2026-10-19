#ifndef ODOM_H
#define ODOM_H

#include "main.h"

/* 所有长度用 cm，轮速为方向修正后的脉冲/10ms，左转航向为正。
 * 必须标定左右轮比例、有效轮距、传感器到轮轴的前伸距离。
 * 编码器不能观测打滑，本模块不是绝对定位。见 BSP/README_odometry.md。 */
#define ODOM_ENABLE 1
/* 实车修正：轮径48mm、输出轴1040脉冲/圈（见encoder.h），1040/(pi*4.8cm)=68.97。
 * 最终用实走100cm的K读数修正。此前28mm为口述误记，不再使用。 */
#define ODOM_PULSES_PER_CM 68.97f
#define ODOM_LEFT_SCALE 1.0f
#define ODOM_RIGHT_SCALE 1.0f
#define ODOM_TRACK_WIDTH_CM 17.0f /* 左右轮接地中心间距，不是车体15cm宽度 */
/* 车体20x15cm，传感器排在前端、横向长8cm。
 * 暂按轮轴中心距车尾3cm估算前伸为20-3=17cm；若“后端3cm”是轮胎区域而非轴心，
 * 应实测轮轴中心到传感器排中心的纵向距离替换此值。8cm横向长度不用于此换算。 */
#define ODOM_SENSOR_FORWARD_CM 17.0f

#define ODOM_MAP_STEP_CM 5U
#define ODOM_MAP_MAX 250U
#define ODOM_TEACH_MIN_CM 100.0f
#define ODOM_TEACH_LOST_TICKS 30U /* 学习时连续丢线300ms，作废停车，不覆盖旧图 */

#define ODOM_RECOVERY_LOST_TICKS 50U /* 连续丢线500ms之后才考虑地图 */
#define ODOM_RECOVERY_TRIGGER_CM 15.0f
#define ODOM_RECOVERY_MAX_ERROR_CM 100.0f /* 偏离太远，定位不可信，停车 */
#define ODOM_MATCH_WINDOW 20U /* 上次正常位置前后各20段，避免追到平行赛道 */
#define ODOM_RECOVERY_SPEED 10.0f
#define ODOM_RECOVERY_STEER_MAX 10.0f
#define ODOM_RECOVERY_KP 0.35f
#define ODOM_RECOVERY_ALIGN_DEG 15.0f
#define ODOM_RECOVERY_TURN_DEG 35.0f
#define ODOM_RECOVERY_MAX_TICKS 600U /* 最多6s，包括原地转向 */
#define ODOM_RECOVERY_MAX_TRAVEL_CM 80.0f
#define ODOM_RECOVERY_ARRIVAL_CM 4.0f
#define ODOM_RECOVERY_LOOKAHEAD_CM 30.0f /* 返回附近线段前方的点，避免垂直扎回黑线 */

typedef enum { ODOM_MODE_NONE = 0, ODOM_MODE_TEACH, ODOM_MODE_RUN } OdomMode;
typedef enum
{
    ODOM_TEACH_OK = 0,
    ODOM_TEACH_LOST_LINE,
    ODOM_TEACH_MAP_FULL,
    ODOM_TEACH_COORD_RANGE
} OdomTeachError;
typedef enum
{
    ODOM_RECOVERY_IDLE = 0,
    ODOM_RECOVERY_TURN,
    ODOM_RECOVERY_DRIVE,
    ODOM_RECOVERY_FAILED
} OdomRecoveryState;

void Odom_Init(void);
/* Reset/Update/Recovery只由TIM1调用；StartTeach/Save/CancelTeach只在停车时主循环调用。 */
void Odom_Reset(void);
void Odom_StartTeach(void);
void Odom_CancelTeach(void);
uint16_t Odom_Save(void); /* 双槽Flash，先写新槽，最后提交；失败保留旧图 */
void Odom_Update(int16_t vL, int16_t vR, uint8_t line_centered);
void Odom_ObserveLine(uint8_t bitmap); /* 学习丢线检查 */
uint8_t Odom_TeachValid(void);
OdomTeachError Odom_GetTeachError(void); /* 保留首次故障，停车后仍可查询 */
const char *Odom_GetTeachErrorName(void);

/* trusted_line是共享LineGuard的验证结果，绝不能直接传raw bitmap！
 * verifying=正在低速验线：仍计算救援预算，但不因接近目标提前宣布失败。
 * force_return=避障结束仍未确认线，允许立即走地图返回，不再等一次丢线超时。
 * allowed=0时避障/学习优先，取消救援。FAILED输出0，调用者刹停。 */
uint8_t Odom_RecoveryControl(uint8_t trusted_line, uint8_t allowed, uint8_t verifying,
                            uint8_t force_return, float *base, float *steer);
OdomRecoveryState Odom_GetRecoveryState(void);
OdomMode Odom_GetMode(void);
uint8_t Odom_MapReady(void);
float Odom_GetHeadingErrDeg(void);
float Odom_GetHeadingDeg(void);
float Odom_GetDistCm(void);
float Odom_GetX(void);
float Odom_GetY(void);
float Odom_GetMapDistanceCm(void);
float Odom_GetProgressCm(void);
float Odom_GetStepCm(void);
float Odom_GetStoredLengthCm(void); /* 只读旧Flash图长度；学习新图时也可用，0=未知 */
uint16_t Odom_GetMapCount(void);
uint16_t Odom_GetMapStepCm(void);

#endif
