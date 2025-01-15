// 走行パラメータ
#define STREAT_JUDGE_ANGLE 40

#define CORNER_SPEED 65         // 未使用
#define CORNER_PWM 100          // コーナー脱出時の最高PWM
#define CORNER_FREEPWM_ANGLE 70 // コーナー　モータフリー（パーシャル）判定角度

#define CORNER_START_POWER 70 // コーナー　モータフリー（パーシャル）判定角度

#define SLOPE_UP_SPEED 60   // 坂の上り速度 60= 4.0m/s
#define SLOPE_DOWN_SPEED 60 // 坂の下り速度 60= 4.0m/s

// レーンチェンジ
#define LANE_ANGLE_R 66 // 右レーンアングル
#define LANE_ANGLE_L 66 // 左レーンアングル

// クランク
#define CRANK_ANGLE_R 110 // 右クランクアングル
#define CRANK_ANGLE_L 110 // 左クランクアングル

// ボリューム角度変換
// #define VR_DEG_CHANGE 33 / 100 // 竹内
#define VR_DEG_CHANGE 1 / 5 // 竹内

// ボリュームのセンター値
#define VR_CENTER 515

// 坂
#define SLOPE_CENTER 290     // 坂センサのセンター値
#define SLOPE_UP_START 90    // 上り開始判定
#define SLOPE_UP_FIN 415     // 上り終わり判定
#define SLOPE_DOWN_START 415 // 下りはじめ判定
#define SLOPE_DOWN_FIN 96    // 下り終わり判定

// モータIO
#define FL_B p8_0
#define FL_A p8_1
#define FL_PWM trdgrd0 /* trdgrd0 p2_2	*/

#define FR_PWM trdgrd1 /*trdgrd1 p2_5  */
#define FR_B p5_0
#define FR_A p6_7

#define ST_A p5_1
#define ST_B p5_4
#define ST_PWM trdgrc1 /*  trdgrc1 p2_4	*/
#define RL_B p8_3
#define RL_A p8_2
#define RL_PWM trcgrb_buff /*  trcgrb_buff p5_2 */

#define RR_A p6_5
#define RR_B p6_6
#define RR_PWM trcgrc_buff /*  @trcgrd_buff p5_3 */
