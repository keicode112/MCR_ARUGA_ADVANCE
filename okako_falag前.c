/****************************************************************************/
/* Copyright    株式会社日立ドキュメントソリューションズ                    */
/* 				2022.8 マイコンカー長野県講習会　液晶
 * スイッチEEPROM版		*/
/****************************************************************************/
// 走行パターン
//  11:通常
//  21・31・41:ブレーキ処理
//
// 101:クランク(左右)
//  低速:131
// 151:レーンチェンジ（左右）
// 191:坂道(のぼるくん)
// 211:坂道
// 231:走行停止
// 241:ログデータ出力
//  3/15 R450立ち上がりで止まった動作
//  ⇒アングル判定速度アップ　4⇒8
// LCD変更（須崎くんモデルへ） インクルードファイルの変更 switch_lib_okako_cross.c
//														  switch_lib.h（内容変更）

#include <stdio.h>

#include "data_flash_lib.h" /* データフラッシュライブラリ   */
#include "i2c_eeprom2015_lib.h"
#include "lcd_lib.h"
#include "personal_setting.h"
#include "printf_lib.h"
#include "sfr_r838a.h"   /* R8C/38A SFRの定義ファイル    */
#include "switch_lib.h"  /* スイッチ追加                 */
#include "18curves_array.h"

/*======================================*/
/* I/O設定                        */
/*======================================*/
#define SENS_LL !p9_0
#define ANA_SENS_L ad2
#define SENS_C !p9_1

#define ANA_SENS_R ad3
#define SENS_RR !p9_2
#define GATE !p9_2
#define SENS_ALL (SENS_LL << 2 | SENS_C << 1 | SENS_RR)
#define RUN_SW !p6_0 /* RUN_SW */
#define SLOPE_ANGLE ad1>>2//11/25より
#define BAR_ANGLE (1023 - ad0)

#define L_LED p9_4   /* 左　高輝度ＬＥＤ  p6_4*/
#define R_LED p3_6   /* 右　高輝度ＬＥＤ  p6_5　（使用できない）*/
#define CPU_LED p4_5 /* CPUボードＬＥＤ		*/

#define THRESHOLD_H 500  // アナログセンサ(0～1023)白判定しきい値
#define THRESHOLD_L 300  // アナログセンサ(0～1023)黒判定しきい値

#define ON 1
#define OFF 0

/* その他入出力用 */
#define MTCT_SW p1_3 /* モーターコントロールＳＷ(p1_3)	*/
/*======================================*/
/* シンボル定義                         */
/*======================================*/
/* 定数設定 */
#define TRC_MOTOR_CYCLE 20000 /* 左前,右前モータPWMの周期     */
                              /* 50[ns] * 20000 = 1.00[ms]    */
#define TRD_MOTOR_CYCLE 20000 /* 左後,右後,ｻｰﾎﾞﾓｰﾀPWMの周期   */
                              /* 50[ns] * 20000 = 1.00[ms]    */
//#define FREE 1                /* モータモード　フリー  (使用しない)       */
//#define BRAKE 0               /* モータモード　ブレーキ  (使用しない)        */

/* DataFlash関連 */
#define DF_ADDR_START 0x3000 /* 書き込み開始アドレス         */
#define DF_ADDR_END 0x33ff   /* 書き込み終了アドレス         */

#define DF_PARA_SIZE 32 /* DataFlashパラメータ数        */

#define DF_CHECK 0x00     /* DataFlashチェック            */
#define DF_STOP 0x01      /* 走行停止距離	               	*/
#define DF_Start_dly 0x02 /* スタート待ち時間            	*/
#define DF_kp 0x03        /* トレース比例制御係数         */
#define DF_kd 0x04        /* トレース微分制御係数         */
#define DF_SP_S 0x05      /* 直線走行目標速度値           */
//#define DF_SP_C 0x06      /* カーブ走行目標速度値         */
#define DF_SP_CL 0x07     /* クランク進入目標速度値       */
#define DF_SP_RC 0x08     /* レーンチェンジ進入目標速度値 */

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void init(void);
unsigned char sensor_inp(void);
unsigned char center_inp(void);
unsigned char sensor5_inp(void);
unsigned char startbar_get(void);
unsigned char dipsw_get(void);
unsigned char dipsw_get2(void);
unsigned char pushsw_get(void);
unsigned char cn6_get(void);
// void led_out_org(unsigned char led);  // 未使用
void led_out(unsigned char led);
void motor_r(int accele_l, int accele_r);
void motor2_r(int accele_l, int accele_r);
void motor_f(int accele_l, int accele_r);
void motor2_f(int accele_l, int accele_r);
// void motor_mode_r(int mode_l, int mode_r);  // 未使用
// void motor_mode_f(int mode_l, int mode_r);  // 未使用
void servoPwmOut(int pwm);
int check_crossline(void);
int check_rightline(void);
int check_leftline(void);


int diff_fi(int pwm);  // front_in内輪差を求める
int diff_ri(int pwm);  // diff_ri
int diff_ro(int pwm);  // diff_ri

int getServoAngle(void);
int getAnalogSensor(void);
void servoControl(void);
void servoControl2(void);
int diff(int pwm);
void readDataFlashParameter(void);
void writeDataFlashParameter(void);
int lcdProcess(void);

void wait_ms(unsigned long timer_set);  // 1ms時間待ち関数(割り込み未使用)
unsigned int abs(int i);

int courceOut(void);  // コースハズレ特定関数
void mtTest(void);    // モータテスト
int angleStreatCheck(int i,int jide_angle) ;//ブレーキ時のノイズ対策


int isSensllON=OFF;//左のON判定　　SENS_LL
int isSensrrON=OFF;//右のON判定　　SENS_RR

int isSensllCount=0;
int isSensrrcount=0;


/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
// const char *C_DATE = __DATE__; /* コンパイルした日付           */
// const char *C_TIME = __TIME__; /* コンパイルした時間           */

int pattern = 0; /* マイコンカー動作パターン     */

unsigned long cnt1;          /* タイマ用                     */
unsigned long cnt2;          /* タイマ用                     */

unsigned long check_sen_cnt; /* タイマ用                     */
unsigned long check_enc_cnt; /* タイマ用                     */
unsigned long check_ana_cnt; /* タイマ用                     */
unsigned long cnt_lcd;       /* LCD処理で使用                */

/* 走行モード・時間処理等 */
int isBrakeOn;  // ブレーキ判定フラグ
int crankMode = 0;  // クランク判定   1:クランクモード 0:通常
char crankDirection = 'N';  // クランクの方向 R:右 L:左
int laneMode = 0;          // レーン判定
char laneDirection = 'N';  // レーンの方向 R:右 L:左
int slopeTotalCount=0;//坂通過数（２度通過防止）

long slopeFinTime = 0;//登坂後の安定待ち
int laneClearTime=0;//レーン後のブレーキ防止
int crankClearTime=0;//クランク後のブレーキ防止
int mtPower=0;//コーナ立ち上がり徐々に
int temp;

int lane_count=0;
int crank_count=0;


/* エンコーダ関連 */
int iTimer10;              /* 10msカウント用               */
long lEncoderTotal;        /* 積算値保存用                 */
int iEncoder;              /* 10ms毎の最新値               */
unsigned int uEncoderBuff; /* 計算用　割り込み内で使用     */
long lEncoderBuff;//エンコーダの値取得（距離制御用）

/* 速度移動平均算出用変数*/
signed int sp;						/* 速度　移動平均値  */
signed int spSum=0;				/* 速度　移動平均値演算用変数 */
signed int spBuf[4];				/* 速度　移動平均値演算用変数 */
signed int spCount=0;				/* 速度　移動平均値演算用変数 */


/* 坂道検出移動平均算出用変数*/
signed int sakaSum=0;				/* 坂道検出センサ　移動平均値演算用変数 */
signed int sakaBuf[8];				/* 坂道検出センサ　移動平均値演算用変数 */
signed int sakaCount=0;			/* 坂道検出センサ　移動平均値演算用変数 */
signed int sakaTemp;

/*  サーボ関連 */
signed int iSensorBefore; /* 前回のセンサ値保存           */
signed int iServoPwm;     /* サーボＰＷＭ値               */
signed int iAngle0;       /* 中心時のA/D値保存            */
signed int iAngle2;       /* ステアリング角速度     */
signed int iAngleBuff;    /* 計算用　割り込み内で使用     */

/* サーボ関連2 */
int iSetAngle;
int iAngleBefore2;
int iServoPwm2;
int cource = 0;  // コースハズレ値

/* センサ関連 */
int iSensorPattern; /* センサ状態保持用 (使用しない)            */

/* データ保存関連 */
volatile int saveIndex;            /* 保存インデックス             */
volatile int saveSendIndex;        /* 送信インデックス             */
volatile int saveFlag;             /* 保存フラグ                   */
volatile signed char saveData[16]; /* 一時保存エリア               */
volatile char data_lost;           /* 保存失敗データ数				*/

/* DataFlash関係 */
signed char data_buff[16];

/* LCD関連 */
int lcd_pattern = 1;

/* TRCレジスタのバッファ */
unsigned int trcgrb_buff; /* TRCGRBのバッファ             */
unsigned int trcgrc_buff; /* TRCGRCのバッファ             */

/* モータドライブ基板TypeS Ver.3上のLED、ディップスイッチ制御 */
// 使用しない
unsigned char types_led;   /* LED値設定                    */
unsigned char types_dipsw; /* ディップスイッチ値保存       */

// 使用しない
/* 内輪差値計算用　各マイコンカーに合わせて再計算して下さい */
const int revolution_difference[] = {/* 角度から内輪、外輪回転差計算 */
                                     100, 98, 97, 95, 94, 92, 91, 89, 88, 87,
                                     85,  84, 82, 81, 80, 78, 77, 76, 74, 73,
                                     72,  70, 69, 68, 66, 65, 64, 62, 61, 60,
                                     58,  57, 56, 54, 53, 52, 50, 49, 48, 46,
                                     45,  43, 42, 40, 39, 38};

/*共通(グローバル)変数*/
// 内輪差関係配列
int revolution_difference_fi[60] = {
    100, 98, 97, 95, 94, 92, 90, 89, 87, 86, 84, 82, 81, 79, 77, 76, 74, 72, 71,
    69,  68, 66, 64, 63, 61, 59, 58, 56, 55, 53, 51, 50, 48, 47, 45, 44, 42, 40,
    39,  37, 36, 34, 33, 31, 30, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28};
int revolution_difference_ri[60] = {
    100, 99, 97, 96, 94, 93, 92, 90, 89, 88, 86, 85, 84, 83, 81, 80, 79, 78, 77,
    76,  75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 65, 64, 63, 62, 62, 61, 60, 59,
    59,  58, 57, 56, 56, 55, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54, 54,
};  // 54
int revolution_difference_ro[60] = {
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 99, 99, 99,
    99,  98,  98,  98,  97,  97,  97,  96,  96,  95,  95,  94, 94, 93,
    93,  92,  91,  91,  90,  90,  89,  88,  88,  87,  86,  85, 85, 84,
    83,  82,  82,  81,  81,  81,  81,  81,  81,  81,  81,  81, 81, 81,
};  // 81






/************************************************************************/



/* メインプログラム                                                     */



/************************************************************************/
void main(void) {
    signed int i, ret;
    //    char fileName[8 + 1 + 3 + 1]; /* 名前＋'.'＋拡張子＋'\0'      */
    //	char s[50];
    unsigned char b;

    /* マイコン機能の初期化 */
    init();                         /* 初期化                       */
    initI2CEeprom();                /* EEP-ROM初期設定      		*/
    init_uart0_printf(SPEED_38400); /* UART0とprintf関連の初期化    */

    asm(" fset I "); /* 全体の割り込み許可           */
    initLcd();
    initSwitch(); /* スイッチ初期化               */
    // initBeepS();  /* ブザー関連処理               */

    readDataFlashParameter(); /* DataFlashパラメータ読み込み  */

    /* マイコンカーの状態初期化 */
    printf("start\n");
    motor_f(0, 0);
    motor_r(0, 0);
    servoPwmOut(0);

    // リセット動作確認
    for (i = 0; i < 5; i++) {

        CPU_LED = ON;
        L_LED=ON;
        //R_LED=ON;
        wait_ms(100);

        CPU_LED = OFF;
        L_LED=OFF;
        //R_LED=OFF;
        wait_ms(100);
    }

    /* スタート時、スイッチが押されていればログデータ転送モード */
    //    if (pushsw_get()) {
    if (RUN_SW == ON) {
        printf("log_printout\n");
        pattern = 241;
        cnt1 = 0;
    }





#if 0

    while (1) {
        mtTest();
    }


    while (1) {
                            // iAngle0 = getServoAngle(); /* 0度の位置記憶 */
         iAngle0 = VR_CENTER;  // センター値固定
         iSetAngle = -100; /* +で左 -で右に曲がります      */
         servoPwmOut(iServoPwm2);
                    //wait_ms(2000);
    }
    iAngle0=getServoAngle();

    while (RUN_SW == OFF) {
        //i = getServoAngle();//ステアリング角度取得
        i =VR_CENTER;//センター値固定
        lcdPosition(0, 1);
        lcdPrintf("angle = %4d%   ",i);
    }


    while(1){//内輪差チェック
            i =VR_CENTER;//センター値固定
            i = getServoAngle();//ステアリング角度取得

            if (i >0 ) {
             //速度制御
                  motor_f(diff(80), 80);
                  motor_r(diff(80), 80);
             }
             else if(i < 0){
             //速度制御
                  motor_f(80,diff(80));
                  motor_r(80,diff(80));
             }
             else{
                  motor_f(80,80);
                  motor_r(80,80);
             }
    }
    /* トレース微分制御調整 */
        while(1){
               servoPwmOut(iServoPwm);
               motor_f(0,0);
               motor_r(0,0);
        }

#endif

    while (1) {//無限ループ
        //printf("pattern=%d\n",pattern);
        I2CEepromProcess(); /* I2C EEP-ROM保存処理          */


		if(SENS_RR == ON){
			isSensrrCount++;
			if(isSensrrCount>15){
				isSensrrON=ON;
			}
		}
		else{
			isSensrrCount=0;			
			isSensrrON=0;
		}

		if(SENS_LL == ON){
			isSensllCount++;
			if(isSensllCount>15){
				isSensllON=ON;
			}
		}
		else{
			isSensllCount=0;			
			isSensllON=0;
		}


        if (pattern >= 11 && pattern <= 50) {


        if (pattern >= 11 && pattern <= 50) {
            /* クロスラインチェック         */
            if (check_crossline()) {
                cnt1 = 0;
                crankMode = 1;
                pattern = 101;
            }

            /* 左ハーフラインチェック       
            if (check_leftline() &&  abs(getServoAngle())<8) {
                cnt1 = 0;
                laneMode = 1;
                pattern = 151;
                laneDirection = 'L';
            }
			*/


            /* 右ハーフラインチェック       */
            if (check_rightline()==1 &&  abs(getServoAngle())<15) {
                cnt1 = 0;
                laneMode = 1;
                pattern = 151;
                laneDirection = 'R';
            }

		    /* 登坂検出      */
    		if( (SLOPE_ANGLE >  SLOPE_UP_START-5) && slopeTotalCount==0 ){
					pattern=191;	//坂走行処理へ	 のぼるくん
		}

    		/* 下り坂検出 */
			if((SLOPE_ANGLE < SLOPE_DOWN_START+5) && slopeTotalCount==1 &&slopeFinTime==0) {
					pattern=211;	//坂走行処理へ	 のぼるくん
			}

			



        }

        switch (pattern) {
            case 0:
                pattern = 2;
                cnt1 = 0;
                break;

            case 1:

                break;

            case 2:
                /* プッシュスイッチ押下待ち */

                servoPwmOut(0);

                // LCD表示、パラメータ設定処理
                lcdProcess();
                //    if (pushsw_get()) {

                if (RUN_SW == ON) {
                    clearI2CEeprom(&p2, 7);
                    //  setBeepPatternS(0xcc00);
                    //   パラメータ保存
                    writeDataFlashParameter();
                    printf("writeDataFlashParameter");
                    cnt1 = 0;
                    //pattern = 3;//ゲートセンサ無し

                    pattern = 3;//ゲートセンサ有り（手押し）
                    break;
                }

                // led_out(i); /* LED点滅処理                  */
                break;

            case 3:
                /* スタートバー開待ち */
                servoPwmOut(iServoPwm / 2);
                //                if (!startbar_get()) {
                if (GATE == OFF) {
                    // iAngle0 = getServoAngle(); /* 0度の位置記憶 */
                    iAngle0 = VR_CENTER;  // センター値固定

                    // led_out(0x0);
                    CPU_LED = OFF;
                    cnt1 = 0;
                    saveIndex = 0;
                    //saveFlag = 1; //pattern :5 で行う　/* データ保存開始               */
                    check_sen_cnt = 0;
                    check_enc_cnt = 0;
					check_ana_cnt = 0;
                    cnt1=0;
                    pattern = 5;
                    break;
                } else {
                    if (cnt1 < 300) {
                        CPU_LED = ON;
                        R_LED = ON;
                        L_LED = OFF;
                    } else {
                        CPU_LED = OFF;
                        R_LED = OFF;
                        L_LED = ON;
                        if (cnt1 > 600) {
                            cnt1 = 0;
                        }
                    }
                }

                // if (startbar_get()) {

                //                led_out(1 << (cnt1 / 50) % 4);
                break;



            case 4:
                /* スタートSW待ち */
                servoPwmOut(iServoPwm / 2);
                //                if (!startbar_get()) {
                if (RUN_SW == ON) {
                    // iAngle0 = getServoAngle(); /* 0度の位置記憶 */
                    iAngle0 = VR_CENTER;  // センター値固定

                    // led_out(0x0);
                    CPU_LED = OFF;
                    cnt1 = 0;
                    saveIndex = 0;
                    //saveFlag = 1; //pattern :5 で行う　/* データ保存開始               */
                    check_sen_cnt = 0;
                    check_enc_cnt = 0;
					check_ana_cnt = 0;
                    pattern = 5;
                    wait_ms(100);
                    while(RUN_SW == ON){
                        if (cnt1 < 100) {
                            CPU_LED = ON;
                            R_LED = ON;
                            L_LED = OFF;
                        } else {
                            CPU_LED = OFF;
                            R_LED = OFF;
                            L_LED = ON;
                            if (cnt1 > 200) {
                                cnt1 = 0;
                            }

                        }
                    }
                }
                    if (cnt1 < 300) {
                            CPU_LED = ON;
                            R_LED = ON;
                            L_LED = OFF;
                    } else {
                            CPU_LED = OFF;
                            R_LED = OFF;
                            L_LED = ON;
                            if (cnt1 > 600) {
                                cnt1 = 0;
                            }
                    }

                break;


            case 5:
                    if (cnt2 < 100) {
                            CPU_LED = ON;
                            R_LED = ON;
                            L_LED = OFF;
                    } else {
                            CPU_LED = OFF;
                            R_LED = OFF;
                            L_LED = ON;
                            if (cnt2 > 200) {
                                cnt2 = 0;
                            }
                    }


                if(cnt1>data_buff[DF_Start_dly]*1000){
                    pattern=11;
                    cnt1 = 0;
                    saveIndex = 0;
                    saveFlag = 1; /* データ保存開始               */
                    check_sen_cnt = 0;
                    check_enc_cnt = 0;
                }
                servoPwmOut(iServoPwm);  // ライントレース制御

                break;

/************************************************************************/



/* 通常走行処理 */



/************************************************************************/
            case 11:
                /* 通常トレース */
                servoPwmOut(iServoPwm);  // ライントレース制御
                i = getServoAngle();     // ステアリング角度取得

                if (abs(i)< 5) {  // ブレーキ許可フラグON
                    isBrakeOn = 1;
                }
                if ((abs(i) >8  && abs(iAngle2) > 6) && isBrakeOn == 1 && laneClearTime==0 && crankClearTime==0 ) {
                    pattern = 12;  // ブレーキ処理へ
                    isBrakeOn = 0;
                    lEncoderBuff = lEncoderTotal;
                    break;
                }

                // 通常走行時
                if (i > 110) {  // 車体九の字　モータ停止
                    motor_f(0, 0);
                    motor_r(0, 0);
                } else if (i > 25) {
                    // 速度制御
                    if (iEncoder > data_buff[DF_SP_S] + 5) {
                        motor_f(-5, -5);  // 前 (左,右)
                        motor_r(-5, -5);  // 後 (左,右)
                    } else if (iEncoder > data_buff[DF_SP_S]) {
                        motor_f(1, 1);
                        motor_r(1, 1);
                    } else {
                        motor_f(diff_fi(85), 85);
                        motor_r(diff_ri(85), diff_ro(85));
                    }
                }

                else if (i < -110) {  // 110
                    motor_f(0, 0);
                    motor_r(0, 0);
                } else if (i < -25) {
                    // 速度制御
                    if (iEncoder > data_buff[DF_SP_S] + 5) {
                        motor_f(-5, -5);  // 前 (左,右)
                        motor_r(-5, -5);  // 後 (左,右)
                    } else if (iEncoder > data_buff[DF_SP_S]) {
                        motor_f(1, 1);
                        motor_r(1, 1);
                    } else {
                        motor_f(85, diff_fi(85));
                        motor_r(diff_ro(85), diff_ri(85));
                    }
                }

                else {
                    // 速度制御
                    if (iEncoder > data_buff[DF_SP_S] + 5) {
                        motor_f(-5, -5);  // 前 (左,右)
                        motor_r(-5, -5);  // 後 (左,右)
                    } else if (iEncoder > data_buff[DF_SP_S]) {
                        motor_f(1, 1);
                        motor_r(1, 1);
                    }
                    // 速度制御
                    else {
                        motor_f(100, 100);
                        motor_r(100, 100);
                    }
                }

                break;

/************************************************************************/



/* ブレーキ処理          */



/************************************************************************/

            case 12:  //	ストレート後のブレーキ処理　①　フロントブレーキ強め　100mm程度
                // 初期ブレーキのみ左右が異なるがその後のブレーキバランスは、左右同様
                servoPwmOut(iServoPwm);  // ライントレース制御
                i = getServoAngle();     // ステアリング角度取得

                if (i >= 0) {              // 左コーナー
                    if (70 <= iEncoder) {  //------強ブレーキ 4.7[m/s]
                        pattern = 21;
                        lEncoderBuff = lEncoderTotal;  //    kyori = 300;        // 200mm
                        cnt1=0;
                        motor_f(-85, -85);  // 前（左,右)
                        motor_r(-65, -65);  // 後（左,右)
                    } else if (65 <= iEncoder) {  //-中ブレーキ 4.3[m/s]
                        pattern = 31;
                        lEncoderBuff =lEncoderTotal;  //    kyori = 225;        // 150mm
                        motor_f(-60, -60);  // 前（左,右)
                        motor_r(-50, -50);  // 後（左,右)
                    } else {                //-------------弱ブレーキ
                        pattern = 41;       // pattern = 41;
                        lEncoderBuff =lEncoderTotal;  //    kyori = 151;        // 100mm
                        motor_f(-25, -25);  // 前（左,右)
                        motor_r(-15, -15);  // 後（左,右)
                    }
                }

                else {                     // 右コーナー
                    if (70 <= iEncoder) {  //------強ブレーキ 4.7[m/s]
                        pattern = 21;
                        lEncoderBuff =lEncoderTotal;  //    kyori = 300;        // 200mm
                        cnt1=0;
                        motor_f(-85, -85);  // 前（左,右)
                        motor_r(-65, -65);  // 後（左,右)
                    } else if (65 <= iEncoder) {  //-中ブレーキ 4.3[m/s]
                        pattern = 31;             // pattern = 31;
                        lEncoderBuff = lEncoderTotal;  //    kyori = 225;        // 150mm
                        motor_f(-60, -60);  // 前（左,右)
                        motor_r(-50, -50);  // 後（左,右)
                    } else {                //-------------弱ブレーキ
                        pattern = 41;       // pattern = 41;
                        lEncoderBuff = lEncoderTotal;  //    kyori = 151;        // 100mm
                        motor_f(-25, -25);  // 前（左,右)
                        motor_r(-15, -15);  // 後（左,右)
                    }
                }
                break;

            // 強ブレーキ
            case 21:  //	ストレート後の強ブレーキ処理
                servoPwmOut(iServoPwm);  // ライントレース制御
                i = getServoAngle();     // ステアリング角度取得
                if (lEncoderTotal - lEncoderBuff >= 350) {  // 200m
                //if(cnt1>30){
                    lEncoderBuff = lEncoderTotal;
                    pattern = 23;
                    break;
                }
                
                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // 通常処理へ
                }
                break;

            case 23:  //	ストレート後の強ブレーキ処理
                servoPwmOut(iServoPwm);  // ライントレース制御
                i = getServoAngle();     // ステアリング角度取得

                motor_f(-65, -65);  // 前（左,右)
                motor_r(-35, -35);  // 後（左,右)

                if (lEncoderTotal - lEncoderBuff >= 225) {  // 150m
                    lEncoderBuff = lEncoderTotal;
                    pattern = 24;
                    break;
                }

                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // 通常処理へ
                }

                break;


            case 24:  //	ストレート後の強ブレーキ処理
                servoPwmOut(iServoPwm);  // ライントレース制御
                i = getServoAngle();     // ステアリング角度取得

                motor_f(1, 1);  // 前（左,右)
                motor_r(1, 1);  // 後（左,右)

                if (lEncoderTotal - lEncoderBuff >= 250) {  // 375mm
                    mtPower=0;
                    pattern = 25;
                    break;
                }

                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // 通常処理へ
                }

                break;

            case 25:  //	ストレート後の強ブレーキ処理
                servoPwmOut(iServoPwm);  // ライントレース制御
                i = getServoAngle();     // ステアリング角度取得

                if(mtPower>CORNER_PWM){
                    mtPower=CORNER_PWM;
                }                

                if (i >= 0) {  // 左コーナー
                    if(abs(i)>CORNER_FREEPWM_ANGLE){//コーナー　モータフリー（パーシャル）判定角度
                        motor_f(1, 1);
                        motor_r(1, 1);                        
                    }
                    else{
                        motor_f(diff_fi(mtPower), mtPower);
                        motor_r(diff_ri(mtPower), diff_ro(mtPower));
                    }
                } else {  // 右コーナー
                    if(abs(i)>CORNER_FREEPWM_ANGLE){//コーナー　モータフリー（パーシャル）判定角度
                        motor_f(1, 1);
                        motor_r(1, 1);                        
                    }
                    else{
                        motor_f(mtPower, diff_fi(mtPower));
                        motor_r(diff_ro(mtPower), diff_ri(mtPower));
                    }
                }


                if(angleStreatCheck(i,STREAT_JUDGE_ANGLE)==1){//true
                    pattern = 11;  // 通常処理へ
                }

                break;

            // 中ブレーキ
            case 31:  //	ストレート後の中ブレーキ処理
                servoPwmOut(iServoPwm);  // ライントレース制御
                i = getServoAngle();     // ステアリング角度取得

                if (lEncoderTotal - lEncoderBuff >= 338) {  // 200m
                //if(cnt1>30){
                    lEncoderBuff = lEncoderTotal;
                    pattern = 33;
                    break;
                }
                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // 通常処理へ
                }

                break;

            case 33:  //	ストレート後の中ブレーキ処理
                servoPwmOut(iServoPwm);  // ライントレース制御
                i = getServoAngle();     // ステアリング角度取得

                motor_f(0, 0);      // 前（左,右)
                motor_r(-10, -10);  // 後（左,右)

                if (lEncoderTotal - lEncoderBuff >= 263) {  // 150m
                    pattern = 34;
                    break;
                }
                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // 通常処理へ
                }

                break;


            case 34:  //	ストレート後の中ブレーキ処理
                servoPwmOut(iServoPwm);  // ライントレース制御
                i = getServoAngle();     // ステアリング角度取得

                motor_f(1, 1);  // 前（左,右)
                motor_r(1, 1);  // 後（左,右)

                if (lEncoderTotal - lEncoderBuff >= 150) {  // 200mm(300) →　100m(150)　（徐々に加速）
                    mtPower=0;
                    pattern = 35;
                    break;
                }

                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // 通常処理へ
                }

                break;

            case 35:  //	ストレート後の中ブレーキ処理
                servoPwmOut(iServoPwm);  // ライントレース制御
                i = getServoAngle();     // ステアリング角度取得


                if(mtPower>CORNER_PWM){
                    mtPower=CORNER_PWM;
                }                

                if (i >= 0) {  // 左コーナー
                    if(abs(i)>CORNER_FREEPWM_ANGLE){
                        motor_f(1, 1);
                        motor_r(1, 1);                        
                    }
                    else{
                        motor_f(diff_fi(mtPower), mtPower);
                        motor_r(diff_ri(mtPower), diff_ro(mtPower));
                    }
                } else {  // 右コーナー
                    if(abs(i)>CORNER_FREEPWM_ANGLE){
                        motor_f(1, 1);
                        motor_r(1, 1);                        
                    }
                    else{
                        motor_f(mtPower, diff_fi(mtPower));
                        motor_r(diff_ro(mtPower), diff_ri(mtPower));
                    }
                }

                if(angleStreatCheck(i,STREAT_JUDGE_ANGLE)==1){//true
                    pattern = 11;  // 通常処理へ
                }
                break;

            // 弱ブレーキ
            case 41:  //	ストレート後の弱ブレーキ処理
                servoPwmOut(iServoPwm);  // ライントレース制御
                i = getServoAngle();     // ステアリング角度取得

                if (lEncoderTotal - lEncoderBuff >= 151) {  // 100m
                    lEncoderBuff = lEncoderTotal;
                    pattern = 43;
                    break;
                }
                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // 通常処理へ
                }
                break;

            case 43:  //	ストレート後の弱ブレーキ処理
                servoPwmOut(iServoPwm);  // ライントレース制御
                i = getServoAngle();     // ステアリング角度取得

                motor_f(0, 0);      // 前（左,右)
                motor_r(-10, -10);  // 後（左,右)

                if (lEncoderTotal - lEncoderBuff >= 150) {  // 200mm(300) →　100m(150)　（徐々に加速）
                    mtPower=0;
                    pattern = 45;
                    break;
                }
                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // 通常処理へ
                }

                break;

            case 45:  //	ストレート後の弱ブレーキ処理
                servoPwmOut(iServoPwm);  // ライントレース制御
                i = getServoAngle();     // ステアリング角度取得


                if(mtPower>CORNER_PWM){
                    mtPower=CORNER_PWM;
                }                

                if (i >= 0) {  // 左コーナー
                    if(abs(i)>CORNER_FREEPWM_ANGLE){
                        motor_f(1, 1);
                        motor_r(1, 1);                        
                    }
                    else{
                        motor_f(diff_fi(mtPower), mtPower);
                        motor_r(diff_ri(mtPower), diff_ro(mtPower));
                    }
                } else {  // 右コーナー
                    if(abs(i)>CORNER_FREEPWM_ANGLE){
                        motor_f(1, 1);
                        motor_r(1, 1);                        
                    }
                    else{
                        motor_f(mtPower, diff_fi(mtPower));
                        motor_r(diff_ro(mtPower), diff_ri(mtPower));
                    }
                }

                if(angleStreatCheck(i,STREAT_JUDGE_ANGLE)==1){//true
                    pattern = 11;  // 通常処理へ
                }

                break;

/************************************************************************/



// クランク走行処理



/************************************************************************/

            case 101:
                /* クロスライン通過処理 */
                servoPwmOut(iServoPwm);
                R_LED = ON;
                L_LED = ON;
                // led_out(0x3);
                motor_f(-90, -90);
                motor_r(-90, -90);
                if (cnt1 >= 30) {  // 誤読み防止(120mm程度)
                    cnt1 = 0;
                    pattern = 102;
                    laneMode = 0;  // レーンモードフラグクリア
					break;
                }
				break;

            case 102:
                lEncoderBuff = lEncoderTotal;
                pattern = 104;
                break;

            case 104:  // クロスライン後の処理(1段目の減速処理)
                servoPwmOut(iServoPwm);
                if (iEncoder >= data_buff[DF_SP_CL] + 3) { /* エンコーダによりスピード制御 */
                    motor_f(-90, -90);
                    motor_r(-90, -90);
                } else if (iEncoder >= data_buff[DF_SP_CL]) {
                    motor_f(10, 10);
                    motor_r(10, 10);
                } else {
                    motor_f(80, 80);
                    motor_r(80, 80);
                }

                if (lEncoderTotal - lEncoderBuff >= 300) {
					  // 200m
                    pattern = 106;
                    break;
                }
				
                break;

            case 106:  // クランク処理 (2段目の減速処理)　ハーフライン検出
                servoPwmOut(iServoPwm);

                if (iEncoder >= data_buff[DF_SP_CL] + 2) { /* エンコーダによりスピード制御 */
                    motor_f(-80, -80);
                    motor_r(-50, -50);
                } else if (iEncoder >= data_buff[DF_SP_CL]) {
                    motor_f(10, 10);
                    motor_r(10, 10);
                } else {
                    motor_f(70, 70);
                    motor_r(70, 70);
                }

                if (isSensllON==ON ) {  // クランク方向　左
                    crankDirection = 'L';  // クランク方向記憶変数＝左クランク
                    iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);
                    pattern = 108;
                    break;
                } else if (isSensrrON==ON) {  // クランク方向　右
                    crankDirection = 'R';  // クランク方向記憶変数＝左クランク
                    iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);
                    pattern = 108;
                    break;
                }

                break;

            case 108:  // クランク処理	 　ハーフライン検出後
                if (crankDirection == 'L') {  // クランク方向　左
                    iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);

                    motor_f(-80, 1);  // 前 （左,右）
                    motor_r(-50, -50);  // 後モータ（左,右）
                }

                else if (crankDirection == 'R') {  // クランク方向　右
                    iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);

                    motor_f(1, -80);  // 前 （左,右）
                    motor_r(-50, -50);  // 後モータ（左,右）
                }

                if (SENS_C == OFF && SENS_LL == OFF && SENS_RR == OFF) {
                    pattern = 110;  // ﾌﾛﾝﾄのﾃﾞｼﾞﾀﾙｾﾝｻ全て黒の時次の処理へ
                    lEncoderBuff = lEncoderTotal;
                    cnt1 = 0;
                    CPU_LED = ON;
                    // break;
                }

                break;

            case 110:  // クランク処理	 　ハーフライン検出後

                if (crankDirection == 'L') {  // クランク方向　左
                    iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);

                    if (isSensrrON==ON) {
                        pattern = 112;  //(通常クランクの処理へ)　ﾌﾛﾝﾄ右ｾﾝｻ(out)反応時
                        cnt1 = 0;  // 116:20ms待ち

                        motor_f(1, 1);   // 前 （左,右）
                        motor_r(-50, 10);  // 後モータ（左,右）
                        break;
                    }

                    if (isSensllON==ON && ANA_SENS_L < THRESHOLD_L &&  (lEncoderTotal - lEncoderBuff) >= 75) {
                        pattern = 131;  //(低速進入時処理)　ﾌﾛﾝﾄ左ｾﾝｻ(in)反応時
                        cnt1 = 0;
                        break;
                        // （左,右） motor_f(1,1);
                        // //前 （左,右）
                        // motor_r(-40,1);
                    }



                } else if (crankDirection == 'R') {  // クランク方向　右
                    iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);

                    if (isSensllON==ON) {
                        pattern =  112;  //(通常クランクの処理へ) ﾌﾛﾝﾄ左ｾﾝｻ(out)反応時
                        motor_f(1, 1);   // 前 （左,右）
                        motor_r(-50, 10);  // 後モータ（左,右）
                        cnt1 = 0;  // 116:20ms待ち
                        break;
                    }

                    if (isSensrrON==ON && ANA_SENS_R < THRESHOLD_L &&
                        (lEncoderTotal - lEncoderBuff) >= 75) {
                        pattern = 131;  //(低速進入時処理)　ﾌﾛﾝﾄ右ｾﾝｻ(in)反応時
                        cnt1 = 0;
                        break;
                    }
                }
                break;

            case 112:
                if (crankDirection == 'L') {  // クランク方向　左

                    iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);

                    //					motor_f(1,1);
                    ////前
                    // （左,右）
                    // motor_r(-40,1);
                    // //後モータ（左,右） 			motor_f(-50,1);
                    // //前 （左,右）
                    // motor_r(-60,30);
                    // //後モータ（左,右）

                    motor_f(1, 1);  // 前 （左,右）
                    motor_r(-50, 10);  // 後モータ（左,右）

                    // 通常クランク処理
                    // ﾌﾛﾝﾄのﾃﾞｼﾞﾀﾙｾﾝｻ全て黒（右センサＯＦＦ）の時次の処理へ　　（センサー拡張のため　左センサー反応時を除く）
                    if (SENS_C == OFF && ANA_SENS_R < THRESHOLD_L &&
                        SENS_RR == OFF && ANA_SENS_L < THRESHOLD_L &&
                        cnt1 > 20) {
                        pattern = 114;
                    }
                    // コースアウト対応クランク処理(←対応しない)
                } else if (crankDirection == 'R') {  // クランク方向　右

                    iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);

                    //					motor_f(1,a1);
                    ////前
                    // （左,右）
                    // motor_r(1,-40);
                    // //後モータ（左,右）
                    motor_f(1, 1);  // 前 （左,右）
                    motor_r(10, -50);  // 後モータ（左,右）

                    // 通常クランク処理
                    // ﾌﾛﾝﾄのﾃﾞｼﾞﾀﾙｾﾝｻ全て黒（左センサＯＦＦ）の時次の処理へ　（センサー拡張のため　左センサー反応時を除く）
                    if (SENS_C == OFF && ANA_SENS_R < THRESHOLD_L &&
                        SENS_LL == OFF && ANA_SENS_L < THRESHOLD_L &&
                        cnt1 > 20) {
                        pattern = 114;
                    }
                    // コースアウト対応クランク処理(←対応しない)
                }
                break;

            case 114:  // コースアウト時のクランク処理
                if (crankDirection == 'L') {  // クランク方向　左
                    iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);
                    //					motor_f(-70,1);
                    ////前 （左,右）
                    motor_f(1, 20);    // 前 （左,右）
                    motor_r(1, 60);  // 後モータ（左,右）motor_r(-40, 1);
                    // ﾌﾛﾝﾄのﾃﾞｼﾞﾀﾙｾﾝｻ全て黒（右センサＯＦＦ）の時次の処理へ　　（センサー拡張のため　左センサー反応時を除く）
                    if (SENS_C == OFF && ANA_SENS_R < THRESHOLD_L &&
                        SENS_RR == OFF && ANA_SENS_L < THRESHOLD_L) {
                        pattern = 116;
                        break;
                    }
                }

                else if (crankDirection == 'R') {  // クランク方向　右
                    iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);
                    //					motor_f(1,-70);
                    ////前 （左,右）
                    motor_f(20, 1);    // 前 （左,右）
                    motor_r(60, 1);  // 後モータ（左,右）
                    // ﾌﾛﾝﾄのﾃﾞｼﾞﾀﾙｾﾝｻ全て黒（左センサＯＦＦ）の時次の処理へ　　（センサー拡張のため　右センサー反応時を除く）
                    if (SENS_C == OFF && ANA_SENS_R < THRESHOLD_L &&
                        SENS_LL == OFF && ANA_SENS_L < THRESHOLD_L) {
                        pattern = 116;
                        break;
                    }
                }
                break;

            case 116:                         // クランク処理
                if (crankDirection == 'L') {  // クランク方向　左
                    iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);
                    motor_f(100, 70);  // 前 （左,右)(100,70) (30, 0)
                    motor_r(50, 60);  // 後モータ(左,右)

                    if (isSensllON==ON && ANA_SENS_L < THRESHOLD_L) {
                        pattern =
                            118;  // ﾌﾛﾝﾄ左ｾﾝｻ（ﾃﾞｼﾞﾀﾙまたはｱﾅﾛｸﾞ）反応時次の処理へ
                        cnt1 = 0;
                        break;
                    }
                    /* ステアリング角 0:中央 +:左 -:右 1.6=1度 */
                    //					ST_A = 0;
                    //					ST_B = 1;
                    //					ST_PWM=10;

                } else if (crankDirection == 'R') {  // クランク方向　右
                    /* ステアリング角 0:中央 +:左 -:右 1.6=1度 */
                    //					ST_A = 1;
                    //					ST_B = 0;
                    //					ST_PWM=10;
                    // ST_PWM=0;

                    iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);
                    motor_f(70, 100);  // 前 （左,右）(70,100) (0, 30)
                    motor_r(60, 50);  // 後モータ（左,右)

                    if (isSensrrON==ON && ANA_SENS_R < THRESHOLD_L) {
                        pattern =
                            118;  // ﾌﾛﾝﾄ左ｾﾝｻ（ﾃﾞｼﾞﾀﾙまたはｱﾅﾛｸﾞ）反応時次の処理へ
                        cnt1 = 0;
                    }
                }
                break;
            case 118:
                if (crankDirection == 'L') {  // クランク方向　左
                    //					ST_A = 0;
                    //					ST_B = 1;
                    //					ST_PWM=0;
                    iSetAngle = CRANK_ANGLE_L/2+20; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);
                    motor_f(100, 70);  // 前 （左,右）(100,70)
                    motor_r(60, 50);    // 後モータ（左,右）　
                } else if (crankDirection == 'R') {  // クランク方向　右
                    // カウンターステア
                    //					ST_A = 1;
                    //					ST_B = 0;
                    //					ST_PWM=0;
                    // ST_PWM=0;

                    iSetAngle = -(CRANK_ANGLE_R/2+20); /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);
                    motor_f(70, 100);  // 前 （左,右）(70,100)
                    motor_r(50, 60);    // 後モータ（左,右）
                }
                if (cnt1 > 10 && SENS_C == ON) {  // 10ms後
                    pattern = 120;
                    crankMode = 0;  // クランクモードクリア
                    cource = 0;  // コース外れ値0クリア
                }
                break;

            case 120:
                /* 少し時間が経つまで待つ */
                i = getServoAngle();  // ステアリング角度取得
                servoPwmOut(iServoPwm);
                motor_r(100, 100);
                motor_f(100, 100);
                if (abs(i) < 5) {
                    cnt1 = 0;
                    // led_out(0x0);
                    pattern = 11;
                    crankDirection = 0;  // クランクモード（クランク方向）クリア
                    laneMode = 0;  // レーンモードクリア
                    laneDirection = 0;  // レーンモード（レーン方向）クリア
                    crankClearTime=50;
                    break;
                }
               break;

                // 低速進入時のクランクの処理
            case 131:
                if (crankDirection == 'L') {  // クランク方向　左
                    iSetAngle = CRANK_ANGLE_L; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);
                    motor_f(-10, 60);  // 前 （左,右)(100,70)
                    motor_r(-10, 50);  // 後モータ(左,右)

                } else if (crankDirection == 'R') {  // クランク方向　右
                    iSetAngle = -CRANK_ANGLE_R; /* +で左 -で右に曲がります      */
                    servoPwmOut(iServoPwm2);
                    motor_f(60, -10);  // 前 （左,右）(70,100)
                    motor_r(50, -10);  // 後モータ（左,右)
                }
                if (cnt1 > 10 && SENS_C == ON) {
                    crankMode = 0;  // クランクモードクリア(ライントレースモード復活のため)
                    pattern = 132;
                    cource = 0;  // コース外れ値0クリア
                }
                break;

            case 132:
                /* 少し時間が経つまで待つ */
                i = getServoAngle();  // ステアリング角度取得
                servoPwmOut(iServoPwm);
                motor_r(100, 100);
                motor_f(100, 100);
                if (abs(i) < 5) {
                    cnt1 = 0;
                    // led_out(0x0);
                    pattern = 11;
                    crankDirection = 0;  // クランクモード（クランク方向）クリア
                    laneMode = 0;  // レーンモードクリア
                    laneDirection = 0;  // レーンモード（レーン方向）クリア
                    crankClearTime=50;

                    break;
                }
                break;

/************************************************************************/



/* レーンチェンジの処理*/




/************************************************************************/
            case 151:  // ハーフライン後の処理１（速度制御）
                servoPwmOut(iServoPwm);  // ライントレース制御 motor_r(80, 80);
                motor_f(90, 90);
                motor_r(90, 90);
                crankMode = 1;//ステアリング制御補正なし

                lEncoderBuff = lEncoderTotal;


                if (check_crossline()) { /* クロスラインチェック         */
                    cnt1 = 0;
                    crankMode = 1;
                    pattern = 101;
                    break;
                }

                if(cnt1>10){//50mm進んじゃう
                    pattern = 152;
                }
                break;

            case 152:  // クロスライン後の処理(白線トレース時)
                i = getServoAngle();  // ステアリング角度取得
                servoPwmOut(iServoPwm);

                if (iEncoder >= data_buff[DF_SP_RC] +  3) { /* エンコーダによりスピード制御 */
                    motor_f(-30, -30);
                    motor_r(-30, -30);
                } else if (iEncoder >= data_buff[DF_SP_RC]) {
                    motor_f(10, 10);
                    motor_r(10, 10);
                } else {
                    motor_f(90, 90);
                    motor_r(90, 90);
                }

                if (SENS_RR == OFF && SENS_LL == OFF && SENS_C == OFF && ANA_SENS_L < THRESHOLD_L && ANA_SENS_R < THRESHOLD_L) {
                    pattern = 154;  // 全てのセンサ　黒検出時次の処理へ
                }

                // レーン誤検知用の通常復帰
				
                if (lEncoderTotal - lEncoderBuff >= 3000) {  // 2000m
                    pattern = 11;                            // 通常に戻す
                    break;
                }
				
				
				            /* クロスラインチェック         */
	            if (check_crossline()) {
	                cnt1 = 0;
	                crankMode = 1;
	                pattern = 101;
				    break;
	            }

				
                break;

            case 154:  // 白線トレース終了後処理	最外センサ　白反応待ち
                i = getServoAngle();  // ステアリング角度取得

                if (laneDirection == 'L') {  // レーン方向　左

                    iSetAngle = LANE_ANGLE_L; /* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(-70, 20);         // 前 （左,右-70）
                    motor_r(-30, 1);          // 後（左,右-30)


                    if((LANE_ANGLE_L)-5 < abs(i))  {
                            pattern=155;	//ステアリング角度目標値付近
                            break;
                     }

                    if (isSensllON==ON) {
                            cnt1 = 0;
                            pattern = 156;
                    }



                }

                else if (laneDirection == 'R') {  // レーン方向　右
                    iSetAngle = -LANE_ANGLE_R; 
					
					/* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(20, -70);         // 前 （左,右-70）
                    motor_r(1, -30);          // 後（左,右-30)


                    if((LANE_ANGLE_R)-5 < abs(i))  {
                            pattern=155;	//ステアリング角度目標値付近
                            break;
                     }

                    if (isSensrrON==ON) {
                            cnt1 = 0;
                            pattern = 156;
                    }
                }

                break;



            case 155:  // 白線トレース終了後処理	最外センサ　白反応待ち
                i = getServoAngle();  // ステアリング角度取得

                if (laneDirection == 'L') {  // レーン方向　左

                    iSetAngle = LANE_ANGLE_L; /* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(70, 70);         // 前 （左,右-70）
                    motor_r(1, 1);          // 後（左,右-30)

                    if (isSensllON==ON) {
                          cnt1 = 0;
                          pattern = 156;
                    }

                }

                else if (laneDirection == 'R') {  // レーン方向　右
                    iSetAngle = -LANE_ANGLE_R; /* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(70, 70);         // 前 （左,右-70）
                    motor_r(1, 1);          // 後（左,右-30)

                    if (isSensrrON==ON) {
                          cnt1 = 0;
                          pattern = 156;
                    }
                }


                break;

            case 156:  // 白線トレース終了後処理	最外センサ　白反応待ち
                if (laneDirection == 'L') {  // レーン方向　左
                    iSetAngle = 0; /* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 角度制御
                    motor_f(85, 0);           // 前 （左,右）
                    motor_r(85, 0);           // 後（左,右)

                }

                else if (laneDirection == 'R') {  // レーン方向　右
                    iSetAngle = 0; /* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 角度制御 
                    motor_f(0, 85);           // 前 （左,右）
                    motor_r(0, 85);           // 後（左,右)
                }

                if (cnt1 >= 5) {
                    pattern = 160;  // 5ms後次の処理へ
                    cnt1 = 0;
                }


                break;


            case 160:  // 10m秒待ち後の処理　最内センサ　白反応時待ち
                if (laneDirection == 'L') {         // レーン方向　左

                    iSetAngle = 0; /* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(85, 0);           // 前 （左,右）
                    motor_r(85, 0);           // 後（左,右)

                    if (isSensrrON==ON) {
                        cnt1 = 0;
                        pattern = 162; /*左デジタルセンサ反応時次の処理へ */
                    }


                } else if (laneDirection == 'R') {  // レーン方向　右
                    iSetAngle = 0; /* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(0, 85);           // 前 （左,右）
                    motor_r(0, 85);           // 後（左,右)

                    if (isSensllON==ON) {
                        cnt1 = 0;
                        pattern = 162; /*左デジタルセンサ反応時次の処理へ */
                    }
                }
                break;

            case 162:  // 最内センサ反応後　10ｍ秒待ち
                if (laneDirection == 'L') {         // レーン方向　左
                    iSetAngle = 0; /* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(85, 0);           // 前 （左,右）
                    motor_r(85, 0);           // 後（左,右)

                } else if (laneDirection == 'R') {  // レーン方向　右
                    iSetAngle = 0; /* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(0, 85);           // 前 （左,右）
                    motor_r(0, 85);           // 後（左,右)
                }

                if (cnt1 >= 10) {
                    cnt1 = 0;
                    pattern = 164;  // 10ms後次の処理へ
                }
                break;

            case 164:  // 10ｍ秒待ち後の処理　最内センサ　黒反応時待ち
                if (laneDirection == 'L') {  // レーン方向　左
                    iSetAngle = 0; /* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(85, 0);           // 前 （左,右）
                    motor_r(85, 0);           // 後（左,右)

                    if (SENS_RR == OFF) {
                        pattern = 166; /*左デジタルセンサOFF反応時次の処理へ */
                        cnt1 = 0;
                    }

                }

                else if (laneDirection == 'R') {  // レーン方向　右
                    iSetAngle = 0; /* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(0, 85);           // 前 （左,右）
                    motor_r(0, 85);           // 後（左,右)

                    if (SENS_LL == OFF) {
                        pattern = 166; /*左デジタルセンサOFF反応時次の処理へ */
                        cnt1 = 0;
                    }
                }
                break;

            case 166:  // 最内センサ　黒反応後の処理（大カウンター）　最内センサ　白反応時待ち
                if (laneDirection == 'L') {  // レーン方向　左
                    iSetAngle = -LANE_ANGLE_L;      /* +で左 -で右に曲がります *///カウンターなので逆に振る　
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(90, 80);  // 前 （左,右）
                    motor_r(90, 0);   // 後（左,右)
                    if (isSensrrON==ON && cnt1 >= 10) {
                        pattern = 168;
                    }
                    if (SENS_C == ON && cnt1 >= 10) {
                        pattern = 168;

                    }

                }
                else if (laneDirection == 'R') {   // レーン方向　右　カウンター処理
                    iSetAngle = LANE_ANGLE_R;      /* +で左 -で右に曲がります *///カウンターなので逆に振る　
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(80, 90);  // 前 （左,右）
                    motor_r(0, 90);   // 後（左,右)
                    if (isSensllON==ON && cnt1 >= 10) {
                        pattern = 168;
                    }
                    if (SENS_C == ON && cnt1 >= 10) {
                        pattern = 168;

                    }
                }
               break;
			   
            case 168:  // センターセンサ　白反応時待ち
                if (laneDirection == 'L') {  // レーン方向　左
                    iSetAngle = -LANE_ANGLE_L; /* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(90, 80);  // 前 （左,右）
                    motor_r(90, 0);   // 後（左,右)

                }

                else if (laneDirection == 'R') {  // レーン方向　右　カウンター処理
                    iSetAngle = LANE_ANGLE_R; /* +で左 -で右に曲がります */
                    servoPwmOut(iServoPwm2);  // 2角度制御 3:割込制御無
                    motor_f(80, 90);  // 前 （左,右）
                    motor_r(0, 90);   // 後（左,右)
                }

                if (SENS_C == ON) {
                    pattern = 170; /*中央デジタルセンサ反応時次の処理へ*/
                    cource = 0;  // コース外れ値0クリア
                    cnt1 = 0;
                }
                break;

            case 170:
                /* 少し時間が経つまで待つ */
                i = getServoAngle();  // ステアリング角度取得
                servoPwmOut(iServoPwm);
                motor_r(90, 90);
                motor_f(90, 90);
                if (abs(i) < 10 &&  cnt1>100) {
                    cnt1 = 0;
                    // led_out(0x0);
                    pattern = 11;
                    crankMode = 0;  // クランクモードクリア
                    crankDirection = 0;  // クランクモード（クランク方向）クリア
                    laneMode = 0;  // レーンモードクリア
                    laneDirection = 0;  // レーンモード（レーン方向）クリア
                    laneClearTime=200;
                    break;
                }
                break;




/************************************************************************/



/* 	坂モード処理	case 191                                        */



/************************************************************************/

			//====================上り==========================//

			case 191://再チェック
                servoPwmOut(iServoPwm);
				motor_f(100,100); //前（左,右）
				motor_r(100,100); //後（左,右）

				if( cnt1>=15 ){//15ms
					if(SLOPE_ANGLE > SLOPE_UP_START-5 ){//上るくん
                        pattern=192;
						temp = data_buff[DF_kp];//比例ゲイン
						data_buff[DF_kp] = 3;//比例ゲインダウン
                        lEncoderBuff = lEncoderTotal;//1500ct程度
                        cnt1=0;

					}
					else {//誤検出時
						pattern=11;
					}
                }
               break;



      		case 192:	//車体を少し安定させる
                servoPwmOut(iServoPwm);
				motor_f(100,100); //前（左,右）
				motor_r(1,1); //後（左,右）

				if( cnt1>=40 ){//40ms
                        pattern=194;
                }
	    		break;


			case 194:	//上り坂の4分の3くらいまではSLOPE_SPEEDで走行
                servoPwmOut(iServoPwm);
                if (iEncoder >= SLOPE_UP_SPEED+3) {
					motor_f(-10,-10);	//前 (左,右)
					motor_r(-10,-10);	//後 (左,右)
                }
                else if (iEncoder >= SLOPE_UP_SPEED) {
                    motor_f(1, 1);
                    motor_r(1, 1);
                } else {
                    motor_f(80, 80);
                    motor_r(80, 80);
                }

                 if (cnt2 < 300) {
                        CPU_LED = ON;
                    } else {
                        CPU_LED = OFF;
                        if (cnt2 > 600) {
                            cnt2 = 0;
                        }
                 }

				if(lEncoderTotal-lEncoderBuff >= 1500){//3/4(1000mm程度)
					pattern=196;
                    lEncoderBuff = lEncoderTotal;//1500ct程度
                    cnt1=0;
				}
			break;

			case 196:
                servoPwmOut(iServoPwm);
                motor_f(1,1); 			//前（左,右）
				motor_r(-30,-30); 			//後（左,右）
				if(lEncoderTotal-lEncoderBuff >= 300){//200mm程度
					pattern=198;
                    lEncoderBuff = lEncoderTotal;//1500ct程度
                    cnt1=0;
				}
                 if (cnt2 < 200) {
                        CPU_LED = ON;
                    } else {
                        CPU_LED = OFF;
                        if (cnt2 > 400) {
                            cnt2 = 0;
                        }
                 }
			break;

			case 198:
                CPU_LED = ON;
                servoPwmOut(iServoPwm);
                if (iEncoder >= SLOPE_UP_SPEED-10) {//3.0m/s程度
					motor_f(1,1);	//前 (左,右)
					motor_r(-30,-30);	//後 (左,右)
                }
                else {
					motor_f(1,1);	//前 (左,右)
					motor_r(30,30);	//後 (左,右)
                }
				if(SLOPE_ANGLE <= SLOPE_UP_FIN+20){//坂終わり判定登り
					pattern=200;

				}
			break;



			case 200:
                servoPwmOut(iServoPwm);
				motor_f(1,1); 			//前（左,右）
				motor_r(-30,-30); 			//後（左,右）
				if(lEncoderTotal-lEncoderBuff >= 300){//200mm程度
					pattern=202;
                    cnt1=0;
				}
                break;


			case 202:
                servoPwmOut(iServoPwm);
    			slopeTotalCount=1;
            	data_buff[DF_kp] = temp;
                slopeFinTime=0;//坂下りご検出防止タイマー
				pattern=11;

                /*while(1){//確認用
					motor_f(0,0);	//前 (左,右)
					motor_r(0,0);	//後 (左,右)
                }
                */

			break;

//====================上り終了======================//


//====================下り==========================//

			case 211://検出再確認(不要？とりあえず)
                servoPwmOut(iServoPwm);
				motor_f(70,70); //前（左,右）
				motor_r(70,70); //後（左,右）

				if( cnt1>=15 ){//15ms
					if(SLOPE_ANGLE < SLOPE_DOWN_START+5 ){//下るくん
                        pattern=212;
						temp = data_buff[DF_kp];//比例ゲイン
						data_buff[DF_kp] = 3;//比例ゲインダウン
                        lEncoderBuff = lEncoderTotal;
                        cnt1=0;
					}
					else {//誤検出時
						pattern=11;
					}
                }
               break;


			case 212://センサーバーを落とすためのブレーキ
                servoPwmOut(iServoPwm);
				motor_f(0,0); 			//前（左,右）
				motor_r(-60,-60); 		//後（左,右）
				if(lEncoderTotal-lEncoderBuff >= 300){//200mm程度
                    lEncoderBuff = lEncoderTotal;
					pattern=214;
				}
			break;

			case 214://安定走行を目指し前輪で走行
                servoPwmOut(iServoPwm);
            	motor_f(70,70); 			//前（左,右）
				motor_r(30,30); 			//後（左,右）
				if(lEncoderTotal-lEncoderBuff >= 450){//300mm程度
                    lEncoderBuff = lEncoderTotal;
					pattern=216;
				}
			break;

			case 216:
                servoPwmOut(iServoPwm);
                if (iEncoder >= SLOPE_DOWN_SPEED+3) {
					motor_f(-10,-10);	//前 (左,右)
					motor_r(-10,-10);	//後 (左,右)
                }
                else if (iEncoder >= SLOPE_DOWN_SPEED) {
                    motor_f(1, 1);
                    motor_r(1, 1);
                } else {
                    motor_f(80, 80);
                    motor_r(80, 80);
                }

				if(SLOPE_ANGLE > SLOPE_DOWN_FIN -5 ){//下り終わり検出
                    lEncoderBuff = lEncoderTotal;
					pattern=218;
				}
			break;


			case 218://車体安定のためのブレーキ（センサーバーを安定させる）
                servoPwmOut(iServoPwm);
				motor_f(-30,-30); 			//前（左,右）
				motor_r(-30,-30); 			//後（左,右）
				if(lEncoderTotal-lEncoderBuff >= 151){//50mm
                    lEncoderBuff = lEncoderTotal;
					pattern=220;
				}
			break;

			case 220://車体安定のためのフリー
                servoPwmOut(iServoPwm);
				motor_f(1,1); 			//前（左,右）
				motor_r(1,1); 			//後（左,右）
				if(lEncoderTotal-lEncoderBuff >= 151){//50mm
					pattern=222;
				}
			break;

			case 222:
                servoPwmOut(iServoPwm);
    			slopeTotalCount=2;
            	data_buff[DF_kp] = temp;
				pattern=11;
                /*
                while(1){
					motor_f(0,0);	//前 (左,右)
					motor_r(0,0);	//後 (左,右)
                }
                */
			break;


			//====================下り終了======================//

            case 231:
                /* 停止処理 */
                servoPwmOut(iServoPwm);
                motor_f(0, 0);
                motor_r(0, 0);
                crankMode = 1;
                pattern = 232;
                cnt1 = 0;
                break;

            case 232:
                servoPwmOut(iServoPwm);
                if (iEncoder <= 1) {
                    servoPwmOut(0);
                    // led_out(0x55);
                    pattern = 233;
                    // setBeepPatternS(0xcc00);
                    cnt1 = 0;
                    break;
                }
                break;

            case 233:
                servoPwmOut(iServoPwm);
                if (cnt1 > 500) {
                    //                    msdFlag = 0;
                    pattern = 234;
                    cnt1 = 0;
                    break;
                }
                break;

            case 234:
                servoPwmOut(iServoPwm);
                /* 最後のデータが書き込まれるまで待つ */
                //                if (microSDProcessEnd() == 0) {
                //                    pattern = 235;
                //                }
                //pattern = 235;

                break;

            case 235:
                /* 何もしない */
                break;

            case 241:
                /* 停止 */
                servoPwmOut(0);
                motor_f(0, 0);
                motor_r(0, 0);
                // setBeepPatternS(0xc000);
                saveFlag = 0;
                saveSendIndex = 0;
                pattern = 243;
                cnt1 = 0;
                break;

            case 242:
                /* プッシュスイッチが離されたかチェック */
                if (RUN_SW == OFF) {
                    pattern = 243;
                    cnt1 = 0;
                }
                break;

            case 243:
                /* 0.5s待ち */
                if (cnt1 >= 500) {
                    pattern = 245;
                    cnt1 = 0;
                }
                break;
            case 245:
                /* タイトル転送、転送準備 */
                printf("\n");
                printf("Run Data Out\n");
                pattern = 246;
                break;

            case 246:
                /* データ転送 */
                /* 終わりのチェック */
#if 0
            if (((char)readI2CEeprom(saveSendIndex) == 203) || (saveSendIndex >= 0x8000 /*0x1ffff*/)) {
                pattern = 247;
                printf("fin\n");
                // setBeepPatternS(0xff00);
                cnt1 = 0;
                break;
            }

                /* 空データのチェック */
                if (readI2CEeprom(saveSendIndex) == 0) {
                    data_lost++;
                    saveSendIndex += 16;
                    break;
                }
#endif

                /* データの転送 */
                printf("%4d     ", (saveSendIndex) / 16);
                printf(
                    "d_0=%03d d_1=%03d d_2=%03d d_3=%03d d_4=%03d d_5=%03d "
                    "d_6=%03d d_7=%03d d_8=%03d d_9=%03d d_10=%03d d_11=%03d "
                    "d_12=%03d d_13=%03d d_14=%03d d_15=%03d fin \n\r",
                    /* 00 SENS_ALL 	*/
                    (char)readI2CEeprom(saveSendIndex + 0),
                    /* 01 iEncoder 		*/
                    (char)readI2CEeprom(saveSendIndex + 1),
                    /* 02 	*/
                    (signed char)readI2CEeprom(saveSendIndex + 2),
                    /* 03 getServoAngle 	*/
                    (signed char)readI2CEeprom(saveSendIndex + 3),
                    /* 04  	iAngle2*/
                    (signed char)readI2CEeprom(saveSendIndex + 4),
                    /* 05 Rセンサ アナログ 	*/
                    (char)readI2CEeprom(saveSendIndex + 5),
                    /* 06 Lセンサ アナログ 	*/
                    (char)readI2CEeprom(saveSendIndex + 6),
                    /* 07 パターン 	*/
                    (char)readI2CEeprom(saveSendIndex + 7),
                    /* 08 saka */
                    (char)readI2CEeprom(saveSendIndex + 8),
                    /* 09 ステアリング	*/
                    (signed char)readI2CEeprom(saveSendIndex + 9),
                    /* 10 accele_fr 		*/
                    (signed char)readI2CEeprom(saveSendIndex + 10),
                    /* 11 accele_fl 	*/
                    (signed char)readI2CEeprom(saveSendIndex + 11),
                    /* 12 accele_rr 	*/
                    (signed char)readI2CEeprom(saveSendIndex + 12),
                    /* 13 accele_rl 	*/
                    (signed char)readI2CEeprom(saveSendIndex + 13),
                    /* 14 	 	*/
                    (char)readI2CEeprom(saveSendIndex + 14),
                    /* 15  	*/
                    (char)readI2CEeprom(saveSendIndex + 15));

                saveSendIndex += 16;
                break;

            case 247:
                /* 転送終了 */

                break;

            default:
                break;
        }
    }
}
void init(void) {
    int i;

    /* クロックをXINクロック(20MHz)に変更 */
    prc0 = 1; /* プロテクト解除               */
    cm13 = 1; /* P4_6,P4_7をXIN-XOUT端子にする*/
    cm05 = 0; /* XINクロック発振              */
    for (i = 0; i < 50; i++)
        ;     /* 安定するまで少し待つ(約10ms) */
    ocd2 = 0; /* システムクロックをXINにする  */
    prc0 = 0; /* プロテクトON                 */
    prc2 = 1; /* PD0のプロテクト解除          */

    /* ポートの入出力設定 */

    /*  PWM(予備)       左前M_PMW       右前M_PWM       ブザー
        センサ左端      センサ左中      センサ右中      センサ右端  */
    // p0  = 0x00;
    pd0 = 0x7f; /* LCDの制御に使用  */

    /*  none			none			RxD0            TxD0
        DIPSW3          DIPSW2          DIPSW1          DIPSW0          */
    pur0 = 0x04; /* P1_3～P1_0(内蔵ＳＷ)のプルアップON     */
    p1 = 0x00;
    pd1 = 0xd0;

    /*  none			none 右前M_PWM
       ステアM_PWM none			左後M_PWM			none
       none      */
    p2 = 0x00;
    pd2 = 0xff;

    /*  none            none            none            none
        none            none            none            エンコーダA相   */
    p3 = 0x00;
    pd3 = 0xf0;

    /*  XOUT            XIN             ボード上のLED   none
        none            VREF            none            none            */
    p4 = 0x20; /* P4_5のLED:初期は点灯         */
    pd4 = 0xb8;

    /*  none		none		none
       ステアM_方向
        none		none		ステアM_方向 右前M_方向		*/
    p5 = 0x00;
    pd5 = 0xff;

    /*  none		右後M_方向		右後M_方向		none
        none		none			none RUN_SW            */
    // p6  = 0x00;
    pd6 = 0xe0;

    /*  none				none				none
       none アナログ左(IN)		アナログ右(IN)		坂センサー(IN)
       角度VR(IN)  */
    // p7  = 0x00;
    pd7 = 0x00;

    /*  none			none			none
       none 左後M_方向 		左後M_方向      左前M_方向      左前M_方向 */
    p8 = 0xff;
    pd8 = 0xff;
    //    pd8 = 0x0f;

    /*  -               -               L_LED				R_LED
        none		Dセンサ右(IN)		Dセンサ中(IN)
       Dセンサ左(IN)		*/
    // p9  = 0x00;
    pd9 = 0x10; /* 0-3:デジタルセンサ各種   4:確認用LED  */
    pu23 = 1;   // P9_4,P9_5をプルアップする

    // trbrcsr = 0x00;

    /* タイマRBの設定 */
    /* 割り込み周期 = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                    = 1 / (20*10^6) * 200        * 100
                    = 0.001[s] = 1[ms]
    */
    //    trbmr = 0x10;     /* 動作モード、分周比設定       */
    //    trbpre = 200 - 1; /* プリスケーラレジスタ         */
    //    trbpr = 100 - 1;  /* プライマリレジスタ           */
    //    trbic = 0x06;     /* 割り込み優先レベル設定       */
    //    trbcr = 0x01;     /* カウント開始                 */
    // 割り込み取りやめ（TimerRCへ移行）

    /* A/Dコンバータの設定 */
    admod = 0x33;   /* 繰り返し掃引モードに設定     */
    adinsel = 0xb0; /* 入力端子P7の4端子を選択      */
    adcon1 = 0x30;  /* A/D動作可能                  */
    asm(" nop ");   /* φADの1サイクルウエイト入れる*/
    adcon0 = 0x01;  /* A/D変換スタート              */

    /* タイマRG タイマモード(両エッジでカウント)の設定 */
    timsr = 0x40; /* TRGCLKA端子 P3_0に割り当てる */
    // trgcr = 0x15; /* TRGCLKA端子の両エッジでカウント 0001 0101*/
    trgcr = 0x05; /* TRGCLKA端子の立ち上がりエッジでカウント 0000 0101*/

    trgmr = 0x80; /* TRGのカウント開始            */

    /* タイマRD PWMモード設定(左前モータ、右前モータ、ステアモータ) */
    /* PWM周期 = 1 / 20[MHz]   * カウントソース * (TRDGRA0+1)
               = 1 / (20*10^6) * 1 =50ns    (*32 = 1600ns)
                           = 1.616 us          * 101
               = 0.1616[ms]     =約6.18k[Hz]
    */
    trdpsr0 = 0x08;              /* TRDIOB0,C0,D0端子設定        */
    trdpsr1 = 0x05;              /* TRDIOA1,B1,C1,D1端子設定     */
    trdmr = 0xf0;                /* バッファレジスタ設定         */
    trdfcr = 0x01;               /* リセット同期PWMモードに設定  */
    trdcr0 = 0x24;               /* ソースカウントの選択:f1      */
    //trdgra0 = trdgrc0 = 101 - 1; /* 周期設定 */ 
	trdgra0 =trdgrc0=601-1;
    trdgrb0 = trdgrd0 = 0;       /* P2_2端子のON幅(左前モータ)   */
    trdgra1 = trdgrc1 = 0;       /* P2_4端子のON幅(ステアモータ)   */
    trdgrb1 = trdgrd1 = 0;       /* P2_5端子のON幅(右前モータ) */
    trdoer1 = 0xcd;              /* 出力端子の選択               */
    trdstr = 0x0d;               /* TRD0カウント開始             */

    /* タイマRC リセット同期PWMモード設定(左後ﾓｰﾀ、右後ﾓｰﾀ) */
    /* PWM周期 = 1 / 20[MHz]   * カウントソース * (TRDGRA0+1)*/

    trcmr = 0x0b; /* 3:"1"  2:D  1:C  0:B  PWMモード選択ビット設定  1011*/
    trccr1 =
        0x8e; /* 7:"1"  6～4:000 ←ｿｰｽｶｳﾝﾄ:f1, 初期出力の設定(0:ＮＯアクティブ
        1:アクティブ) 3:D 2:C 1:B 0:"0" 0110*/
    trccr2 = 0x00; /* 7.6:"00"  5～3:"0 00"  出力レベルの設定(0:Lアクティブ
                      1:Hアクティブ) 2:D 1:C 0:B*/
    /*　アクティブレベル　TRCGRA のコンペアマッチ後の信号  */
    trcoer = 0x06; /* 7～4:"0000" 出力端子の選択 (0:許可　1:不許可) 3:D 2:C 1:B
                      0:A   Cは、予備なので使用しない（I/Oとして利用）*/
    trcpsr0 = 0x70; /* p.323 TRCIOA,B端子の設定 p5_2(RL_PWM:TRCIOB) 7:"0"
                       6～4:111 3:"0" 2～0:"000"*/
    trcpsr1 = 0x06; /* p.324 TRCIOC,D端子の設定 p5_3(RR_PWM:TRCIOC) 7:"0"
                       6～4:"000" 3:"0" 2～0:110*/
    /* p5_4(        :TRCIOD)は予備とする  */

    trcgra = 20000 - 1; /* 周期設定 1ms */
    trcgrb = trcgra;    /* P5_2端子のON幅(RLモーター) */
    trcgrc = trcgra;    /* P5_3端子のON幅(RRモーター) */

    // trcgrd = 0; /* ON幅(予備) 未使用I/Oとして利用 */

    /*バッファ動作を行うため、割り込みを設定 */
    trcic = 0x07;  /* 割り込み優先レベル設定 */
    trcier = 0x01; /* IMIAを許可 */
    trcoer = 0x01; /* 出力端子の選択 */

    trcmr |= 0x80; /* TRCカウント開始 */
}

/************************************************************************/
/* タイマRB 割り込み処理                                                */
/************************************************************************/
// 削除（タイマRCへ移行）

/************************************************************************/
/* タイマRC 割り込み処理                                                */
/************************************************************************/
#pragma interrupt intTRC(vect = 7)
void intTRC(void) {
    static int line_no; /* 行番号                       */
    signed long i;

    trcsr &= 0xfe;

    cnt1++;
    cnt2++;//LED制御用
    slopeFinTime++;//坂誤検出防止タイマー

    mtPower++;//コーナリング時PWMを徐々にアップ用

    if(laneClearTime>0){
        laneClearTime--;
    }

    if(crankClearTime>0){
        crankClearTime--;
    }

    /* タイマRC　デューティ比の設定 */
    trcgrb = trcgrb_buff;
    trcgrc = trcgrc_buff;

    /* サーボモータ制御 */
    servoControl();
    servoControl2();

    if (pattern >= 11 && pattern <= 230) {
        /* 距離による停止処理 */
        // モータSW(ON)：0なら走行
        if (lEncoderTotal >= 1515L * data_buff[DF_STOP] && MTCT_SW == 0) {
            pattern = 231;
        }

        /* 脱輪時の停止処理（デジタルセンサ） */
        //            b = sensor_inp();
        //            if (b == 0x0f || b == 0x0d || b == 0x0b) {
#if 1
        if (SENS_LL == OFF&& SENS_C == OFF && SENS_RR == OFF) {
            check_sen_cnt++;
            if (check_sen_cnt >= 400) {
                pattern = 231;
            }
        } else {
            check_sen_cnt = 0;
        }
#endif

        /* 脱輪時の停止処理（ロータリエンコーダ） */
        // モータSW(ON)：0なら走行
        if (iEncoder <= 1 && MTCT_SW == 0) {
            check_enc_cnt++;
            if (check_enc_cnt >= 300) {
                pattern = 231;
            }
        } else {
            check_enc_cnt = 0;
        }
    }
#if 0
 if (ANA_SENS_L > 300 && ANA_SENS_L > 300) {
            check_ana_cnt++;
            if (check_ana_cnt >= 100) {
                pattern = 231;
            }
        } else {
            check_ana_cnt = 0;
        }
#endif
		
    /* 拡張スイッチ用関数(1msごとに実行)    */
    // switchProcess();

    /* LCD表示処理用関数(1msごとに実行)     */
    // lcdShowProcess();

    /* ブザー処理 */
    // beepProcessS();

    /* microSD間欠書き込み処理(1msごとに実行)   */
    // microSDProcess();

    /* 10回中1回実行する処理 */
    iTimer10++;
    switch (iTimer10) {
        case 1:
            /* エンコーダ制御 */
            i = trg;
            iEncoder = i - uEncoderBuff;
            lEncoderTotal += iEncoder;
            uEncoderBuff = i;
            break;

        case 2:
            i = BAR_ANGLE;             // 現在バーアングル取得
            iAngle2 = i - iAngleBuff;  // ステアリング速度を求める
            iAngleBuff = i;
            break;

        case 3:

            break;

        case 4:
            break;

        case 5:
            break;

        case 6:
            break;

        case 7:
            break;

        case 8:
            /* データ保存関連 */
            if (saveFlag) {
                saveData[0] = SENS_LL << 2 | SENS_C << 1 | SENS_RR;  // SENS_ALL; /* デジタルセンサ */
                saveData[1] = iEncoder; /* エンコーダ */
                saveData[2] = crank_count;       //
                saveData[3] = getServoAngle(); /* サーボ角度 */
                saveData[4] = iAngle2;         /* サーボ角速度 */
                saveData[5] = ANA_SENS_R >> 2; /* アナログセンサ左 */
                saveData[6] = ANA_SENS_L >> 2; /* アナログセンサ右 */
                saveData[7] = pattern;         /* パターン       */
                saveData[8] = SLOPE_ANGLE;     /* 坂道センサAD           */
                // saveData[9] :PWMステアリング;
                // saveData[10] :PWM前右;
                // saveData[11] :PWM前左;
                // saveData[12] :PWM後右;
                // saveData[13] :PWM後左;
                saveData[14] = isBrakeOn;
                // saveData[15] = ;

                setPageWriteI2CEeprom(saveIndex, 16, saveData);
                saveIndex += 16;
                if (saveIndex >= 0x8000 /*0x1ffff*/) saveFlag = 0;
            }
            break;

        case 9:

            break;

        case 10:
            /* iTimer10変数の処理 */
            iTimer10 = 0;
            break;
    }
}

/************************************************************************/
/* タイマ本体                                                           */
/* 引数　 タイマ値 1=1ms                                                */
/************************************************************************/
void wait_ms(unsigned long timer_set) {
    int i;

    timer_set *= 10;

    do {
        for (i = 0; i < 100; i++)
            ;
    } while (timer_set--);
}

/************************************************************************/
/* 絶対値変換 */
/* 引数　 変換値 */
/************************************************************************/
unsigned int abs(int i) {
    if (i < 0) {
        return i * -1;
    } else {
        return i;
    }
}

// 使用しない
/************************************************************************/
/* アナログセンサ基板TypeS Ver.2のデジタルセンサ値読み込み              */
/* 引数　 なし                                                          */
/* 戻り値 左端、左中、右中、右端のデジタルセンサ 0:黒 1:白              */
/************************************************************************/
unsigned char sensor_inp(void) {
    unsigned char sensor;

    sensor = ~p0 & 0x0f;

    return sensor;
}

// 使用しない
/************************************************************************/
/* アナログセンサ基板TypeS Ver.2の中心デジタルセンサ読み込み            */
/* 引数　 なし                                                          */
/* 戻り値 中心デジタルセンサ 0:黒 1:白                                  */
/************************************************************************/
unsigned char center_inp(void) {
    unsigned char sensor;

    sensor = ~p1_7 & 0x01;

    return sensor;
}

// 使用しない
/************************************************************************/
/* アナログセンサ基板TypeS Ver.2のすべてのデジタルセンサ読み込み        */
/* 引数　 なし                                                          */
/* 戻り値 ５つのデジタルセンサ 0:黒 1:白                                */
/************************************************************************/
unsigned char sensor5_inp(void) {
    volatile unsigned char sensor;

    static volatile unsigned char L2;
    static volatile unsigned char L1;
    static volatile unsigned char C;
    static volatile unsigned char R1;
    static volatile unsigned char R2;

    sensor = sensor_inp();

    L2 = (sensor << 1) & 0x10;
    L1 = (sensor << 1) & 0x08;
    C = (center_inp() << 2) & 0x04;
    R1 = (sensor)&0x02;
    R2 = (sensor)&0x01;

    sensor = L2 | L1 | C | R1 | R2;

    return sensor;
}

/************************************************************************/
/* アナログセンサ基板TypeS Ver.2のスタートバー検出センサ読み込み        */
/* 引数　 なし                                                          */
/* 戻り値 0:スタートバーなし 1:スタートバーあり                         */
/************************************************************************/
// 使用しない
unsigned char startbar_get(void) {
    unsigned char sensor;

    sensor = ~p1_6 & 0x01;

    return sensor;
}

/************************************************************************/
/* マイコンボード上のディップスイッチ値読み込み                         */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0～15                                              */
/************************************************************************/
unsigned char dipsw_get(void) {
    unsigned char sw;

    sw = p1 & 0x0f; /* P1_3～P1_0読み込み           */

    return sw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のディップスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0～255                                             */
/************************************************************************/
unsigned char dipsw_get2(void) {
    /* 実際の入力はタイマRB割り込み処理で実施 */
    return types_dipsw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のプッシュスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0:OFF 1:ON                                         */
/************************************************************************/
// 使用しない
unsigned char pushsw_get(void) {
    unsigned char sw;

    sw = ~p9_5 & 0x01;

    return sw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3のCN6の状態読み込み                     */
/* 引数　 なし                                                          */
/* 戻り値 0～15                                                         */
/************************************************************************/
unsigned char cn6_get(void) {
    unsigned char data;

    data = p7 >> 4;

    return data;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3のLED制御                               */
/* 引数　 8個のLED制御 0:OFF 1:ON                                       */
/* 戻り値 なし                                                          */
/************************************************************************/
void led_out(unsigned char led) {
    /* 実際の出力はタイマRB割り込み処理で実施 */
    types_led = led;
}

/************************************************************************/
/* 後輪の速度制御                                                       */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_r(int accele_l, int accele_r) {
    int sw_data;

    // PWM　DOWNを行わない
    //    sw_data = dipsw_get() + 5; /* ディップスイッチ読み込み     */
    //    accele_l = accele_l * sw_data / 20;
    //    accele_r = accele_r * sw_data / 20;

    // モータSW：ON（OFFなら停止）
    if (MTCT_SW == 0) {
        motor2_r(accele_l, accele_r);
    }
}

/************************************************************************/
/* 後輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_r(int accele_l, int accele_r) {
    /* 後左モータ */
    if (accele_l > 0) {
        RL_A = 0;
        RL_B = 1;
    } else if (accele_l < 0) {
        RL_A = 1;
        RL_B = 0;
    } else {
        RL_A = 0;
        RL_B = 0;
        accele_l = 1;
    }
    saveData[13] = accele_l;  // saveData[5]
    RL_PWM = abs(accele_l * 200);
    if (RL_PWM >= 19999) {
        RL_PWM = trcgra + 1;
    }
    /* 後右モータ */
    if (accele_r > 0) {
        RR_A = 0;
        RR_B = 1;
    } else if (accele_r < 0) {
        RR_A = 1;
        RR_B = 0;
    } else {
        RR_A = 0;
        RR_B = 0;
        accele_r = 1;
    }
    saveData[12] = accele_r;  // saveData[5]
    RR_PWM = abs(accele_r * 200);
    if (RR_PWM >= 19999) {
        RR_PWM = trcgra + 1;
    }
}

/************************************************************************/
/* 前輪の速度制御                                                       */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_f(int accele_l, int accele_r) {
    int sw_data;

    // PWM　DOWNを行わない
    //     sw_data = dipsw_get() + 5; /* ディップスイッチ読み込み     */
    //     accele_l = accele_l * sw_data / 20;
    //    accele_r = accele_r * sw_data / 20;

    // モータSW：ON（OFFなら停止）
    if (MTCT_SW == 0) {
        motor2_f(accele_l, accele_r);
    }
}

/************************************************************************/
/* 前輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_f(int accele_l, int accele_r) {
    /* 左前モータ */
    if (accele_l > 0) {
        FL_A = 0;
        FL_B = 1;
    } else if (accele_l < 0) {
        FL_A = 1;
        FL_B = 0;
    } else {
        FL_A = 0;
        FL_B = 0;
        accele_l = 1;
    }
    saveData[11] = accele_l;  // saveData[2]
    FL_PWM = abs(accele_l*6);
    if (FL_PWM >= 599) {
        FL_PWM = trdgra0 + 1;
    }

    /* 右前モータ */
    if (accele_r > 0) {
        FR_A = 0;
        FR_B = 1;
    } else if (accele_r < 0) {
        FR_A = 1;
        FR_B = 0;
    } else {
        FR_A = 0;
        FR_B = 0;
        accele_r = 1;
    }
    saveData[10] = accele_r;  // saveData[3]
    FR_PWM = abs(accele_r*6);
    if (FR_PWM >= 599) {
        FR_PWM = trdgra0 + 1;
    }
}
// 使用しない
/************************************************************************/
/* 後モータ停止動作（ブレーキ、フリー）                                 */
/* 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE               */
/* 戻り値 なし                                                          */
/************************************************************************/
/*void motor_mode_r(int mode_l, int mode_r) {
    if (mode_l) {
        p9_0 = 1;
    } else {
        p9_0 = 0;
    }
    if (mode_r) {
        p9_1 = 1;
    } else {
        p9_1 = 0;
    }
}*/
// 使用しない
/************************************************************************/
/* 前モータ停止動作（ブレーキ、フリー）                                 */
/* 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE               */
/* 戻り値 なし                                                          */
/************************************************************************/
/*void motor_mode_f(int mode_l, int mode_r) {
    if (mode_l) {
        p9_2 = 1;
    } else {
        p9_2 = 0;
    }
    if (mode_r) {
        p9_3 = 1;
    } else {
        p9_3 = 0;
    }
}*/

/************************************************************************/
/* サーボモータ制御                                                     */
/* 引数　 サーボモータPWM：-100～100                                    */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void servoPwmOut(int pwm) {
    /* ボリューム値により左リミット制御 */
//    if (getServoAngle() >= 100 && pattern >= 11) {
//        if (pwm < -10) pwm = 0;
//    }
    /* ボリューム値により右リミット制御 */
//    if (getServoAngle() <= -100 && pattern >= 11) {
//        if (pwm > 10) pwm = 0;
//    }
//
//  if(pattern==24 ||pattern==25||pattern==34 ||pattern==35 ||pattern==44||pattern==45 ){
//        if(pwm>50){
//            pwm=50;
//        }
//        else if(pwm<-50){
//            pwm=-50;
//        }
//    }

    saveData[9] = pwm;  // saveData[13]

    if (pwm > 0) {
        ST_A = 0;
        ST_B = 1;
    } else if (pwm < 0) {
        ST_A = 1;
        ST_B = 0;
    } else {
        ST_A = 0;
        ST_B = 0;
    }
    if (abs(pwm) >= 100) {
        ST_PWM = trdgra0 + 1;  // 100%
    } else {
        ST_PWM = abs(pwm*6);
    }
}

/************************************************************************/
/* クロスライン検出処理                                                 */
/* 引数　 なし                                                          */
/* 戻り値 0:クロスラインなし 1:あり                                     */
/************************************************************************/
int check_crossline(void) {
    unsigned char b;
    int ret = 0;

    //    b = sensor_inp();
    //    if (b == 0x0f || b == 0x0e || b == 0x0d || b == 0x0b || b == 0x07) {
    //        ret = 1;
    //    }
    if (isSensllON==ON && SENS_C == ON && isSensrrON==ON) {


        ret = 1;
    }

    return ret;
}

/***********************9**************************************************/
/* 右ハーフライン検出処理                                               */
/* 引数　 なし                                                          */
/* 戻り値 0:右ハーフラインなし 1:あり                                   */
/************************************************************************/
int check_rightline(void) {
    unsigned char b;
    int ret = 0;

    //    if ((center_inp() == 1) && (sensor_inp() == 0x03)) {
    //        ret = 1;
    //    }

    if (SENS_LL == OFF && SENS_C == ON && isSensrrON==ON) {
		crank_count++;
		lane_count++;
		if(lane_count>20){
             ret = 1;			
		}
    }
	
	
	else{
		
		lane_count=0;	
	}

    return ret;
}


/************************************************************************/
/* 左ハーフライン検出処理                                               */
/* 引数　 なし                                                          */
/* 戻り値 0:左ハーフラインなし 1:あり                                   */
/************************************************************************/
int check_leftline(void) {
    unsigned char b;
    int ret = 0;

    //    if ((center_inp() == 1) && (sensor_inp() == 0x0c)) {
    //        ret = 1;
    //    }

    if (SENS_C == ON && isSensllON==ON && SENS_RR == OFF) {
        ret = 1;

    }

    return ret;
}

/************************************************************************/
/* サーボ角度取得                                                       */
/* 引数　 なし                                                          */
/* 戻り値 サーボ角度                                                */
/************************************************************************/
int getServoAngle(void) {
    /* 検出移動平均算出用変数*/
    static signed int angleSum=0;			/* 移動平均値演算用変数 */
    static signed int angleBuf[8];			/* 移動平均値演算用変数 */
    static signed int angleCount=0;			/* 移動平均値演算用変数 */
    static signed int retAngle;			/* 移動平均値演算用変数 */

	/* 8点の速度の移動平均計算 angleBuf[8]*/
	angleSum =(angleSum + iAngle0) -angleBuf[angleCount];//合計に最も古いデータの値減算　最新データの値加算
	angleBuf[angleCount]=iAngle0; //最新のデータを代入 
	angleCount++;//インデックスのインクリメント
	angleCount = angleCount & 0x07;//0→1　・・・　7→0→1・・
	retAngle = angleSum >> 3 ;// /8

    return (BAR_ANGLE - retAngle);
}

// 使用しない
/************************************************************************/
/* アナログセンサ値取得 (オリジナルプログラム)                          */
/* 引数　 なし                                                          */
/* 戻り値 センサ値                                                      */
/************************************************************************/
int getAnalogSensor_org(void) {
    int ret;

    //   ret = ad1 - ad0; /* アナログセンサ情報取得       */
    ret = ANA_SENS_R - ANA_SENS_L; /* アナログセンサ情報取得       */

    if (!crankMode) {
        /* クランクモードでなければ補正処理 */
        switch (iSensorPattern) {
            case 0:
                if (sensor_inp() == 0x04) {
                    ret = -650;
                    break;
                }
                if (sensor_inp() == 0x02) {
                    ret = 650;
                    break;
                }
                if (sensor_inp() == 0x0c) {
                    ret = -700;
                    iSensorPattern = 1;
                    break;
                }
                if (sensor_inp() == 0x03) {
                    ret = 700;
                    iSensorPattern = 2;
                    break;
                }
                break;

            case 1:
                /* センサ右寄り */
                ret = -700;
                if (sensor_inp() == 0x04) {
                    iSensorPattern = 0;
                }
                break;

            case 2:
                /* センサ左寄り */
                ret = 700;
                if (sensor_inp() == 0x02) {
                    iSensorPattern = 0;
                }
                break;
        }
    }

    return ret;
}
/************************************************************************/
/* アナログセンサ値取得 (岡谷工業  デジタルセンサ3つ用使用)                */
/* 引数　 なし                                                          */
/* 戻り値 センサ値                                                      */
/************************************************************************/
int getAnalogSensor(void) {
    int ret;

    //   ret = ad1 - ad0; /* アナログセンサ情報取得       */
    ret = (ANA_SENS_L>>2) - (ANA_SENS_R>>2); /* アナログセンサ情報取得    左大：＋ 　右大：-　  */
    if(ret<0){
       ret=-curves18[abs(ret)];
    }
    else{
       ret=curves18[abs(ret)];
    }

    if (((isSensllON==ON && SENS_C == ON) || (isSensrrON==ON && SENS_C == ON)) &&pattern != 3) {
        ret = 0;
    }

    if (!crankMode) {
        /* クランクモードでなければ補正処理 */
        // courceOut():0のときは、アナログセンサによるトレース
        switch (courceOut()) {
            case 1:
                ret = -550;
                break;

            case 2:
            case 3:
                ret = -650;
                break;

            case -1:
                ret = 550;
                break;
            case -2:
            case -3:
                ret = 650;
                break;
        }
    }

    return ret;
}

/************************************************************************/
/* コース外れ値取得関数 */
/* 引数　 無し */
/* 戻り値 無し */
/*　注意：　変数cource＝グローバル変数
/************************************************************************/
int courceOut(void) {
    if (SENS_C == ON) { /* Cセンサ白線 */
        switch (cource) {
            case 1:
            case 2:
            case -1:
            case -2:
                cource = 0; /* コース上 */
                break;
        }
    } else if (ANA_SENS_L > THRESHOLD_H) { /* Lセンサ白線 */
        switch (cource) {
            case 0:
            case -2:
                cource = -1; /* コース外れR1 */
                break;
        }
    } else if (ANA_SENS_R > THRESHOLD_H) { /* Rセンサ白線 */
        switch (cource) {
            case 0:
            case 2:
                cource = 1; /* コース外れL1 */
                break;
        }
    } else if (isSensllON==ON) { /* LLセンサ白線 */
        switch (cource) {
            case -1:
            case -3:
                cource = -2; /* コース外れR2 */
                break;
        }
    } else if (isSensllON==ON) { /* RRセンサ白線 */
        switch (cource) {
            case 1:
            case 3:
                cource = 2; /* コース外れL2 */
                break;
        }
    }

    else if (SENS_RR == OFF && SENS_C == OFF && SENS_LL == OFF &&
             ANA_SENS_R < THRESHOLD_L &&
             ANA_SENS_L < THRESHOLD_L) { /* 全てのセンサ黒 */
        switch (cource) {
            case 2:
                cource = 3; /* コース外れL3 */
                break;
            case -2:
                cource = -3; /* コース外れR3 */
                break;
        }
    }
    return cource;
}

/************************************************************************/
/* サーボモータ制御                                                     */
/* 引数　 なし                                                          */
/* 戻り値 グローバル変数 iServoPwm に代入                               */
/************************************************************************/
void servoControl(void) {
    int i, iRet, iP, iD;
    int kp, kd;

    i = getAnalogSensor(); /* センサ値取得                 */
    kp = data_buff[DF_kp];
    kd = data_buff[DF_kd];

    /* サーボモータ用PWM値計算 */
    iP = kp * i;                   /* 比例                         */
    iD = kd * (iSensorBefore - i); /* 微分(目安はPの5～10倍)       */
    iRet = iP - iD;

//    iRet /= 64;//  <<1 :/2  <<2 :/4  <<3 :/8  <<4 :/16   <<5 :/32  <<6 :/64
    iRet /= 16;//  <<1 :/2  <<2 :/4  <<3 :/8  <<4 :/16   <<5 :/32  <<6 :/64


    /* PWMの上限の設定 */
//    if (iRet > 70) iRet = 70;   /* マイコンカーが安定したら     */
//    if (iRet < -70) iRet = -70; /* 上限を70くらいにしてください */

    if (iRet > 100) iRet = 100;   /* マイコンカーが安定したら     */
    if (iRet < -100) iRet = -100; /* 上限を70くらいにしてください */

    iServoPwm = -iRet;
    iSensorBefore = i; /* 次回はこの値が1ms前の値となる*/
}

/************************************************************************/
/* サーボモータ2制御                                                     */
/* 引数　 なし                                                          */
/* 戻り値 グローバル変数 iServoPwm に代入                               */
/************************************************************************/
void servoControl2(void) {

    signed int i, j, iRet, iP, iD;
    signed int kp, kd;

    i = iSetAngle;
    j = getServoAngle();

    /* サーボモータ用PWM値計算 */
    iP = 20 * (j - i);              /* 比例                         */
    iD = 100 * (iAngleBefore2 - j); /* 微分(目安はPの5～10倍)       */
    iRet = iP - iD;
    iRet /= 2;

    /* PWMの上限の設定 */
    if (iRet > 100) iRet = 100;   /* マイコンカーが安定したら     */
    if (iRet < -100) iRet = -100; /* 上限を70くらいにしてください */
    iServoPwm2 = iRet;

    iAngleBefore2 = j; /* 次回はこの値が1ms前の値となる*/
}

/************************************************************************/
/* 外輪のPWMから、内輪のPWMを割り出す　ハンドル角度は現在の値を使用     */
/* 引数　 外輪PWM                                                       */
/* 戻り値 内輪PWM                                                       */
/************************************************************************/
int diff(int pwm) {
    int i, ret;

    i = getServoAngle()* VR_DEG_CHANGE; /* 1度あたりの増分で割る        */
    if (i < 0) i = -i;
    if (i > 45) i = 45;
    ret = revolution_difference[i] * pwm / 100;

    return ret;
}

/* ステアリング角 0:中央 +:左 -:右 1.6=1度 */
/************************************************************************/
/* 外輪のPWMから、内輪のPWMを割り出す　ハンドル角度は現在の値を使用     */
/* 引数　 外輪PWM                                                       */
/* 戻り値 内輪PWM                                                       */
/************************************************************************/
int diff_fi(int pwm) {
    int i, ret;

    i = getServoAngle() * VR_DEG_CHANGE; /* 1度あたりの増分で割る        */
    if (i < 0) i = -i;
    if (i > 45) i = 45;
    ret = revolution_difference_fi[i] * pwm / 100;

    return ret;
}

/************************************************************************/
/* 外輪のPWMから、内輪のPWMを割り出す　ハンドル角度は現在の値を使用     */
/* 引数　 外輪PWM                                                       */
/* 戻り値 内輪PWM                                                       */
/************************************************************************/
int diff_ri(int pwm) {
    int i, ret;

    i = getServoAngle() * VR_DEG_CHANGE; /* 1度あたりの増分で割る        */
    if (i < 0) i = -i;
    if (i > 45) i = 45;
    ret = revolution_difference_ri[i] * pwm / 100;

    return ret;
}

/************************************************************************/
/* 外輪のPWMから、内輪のPWMを割り出す　ハンドル角度は現在の値を使用     */
/* 引数　 外輪PWM                                                       */
/* 戻り値 内輪PWM                                                       */
/************************************************************************/
int diff_ro(int pwm) {
    int i, ret;

    i = getServoAngle() *VR_DEG_CHANGE; /* 1度あたりの増分で割る        */
    if (i < 0) i = -i;
    if (i > 45) i = 45;
    ret = revolution_difference_ro[i] * pwm / 100;

    return ret;
}

/************************************************************************/
/* DataFlashのパラメータ読み込み                                        */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void readDataFlashParameter(void) {
    int i;
    unsigned int st = DF_ADDR_END + 1 - DF_PARA_SIZE;
    signed char c;

    while (1) {
        // 読み込む番地を探す
        readDataFlash(st, &c, 1);
        if (c == 0x11) {
            readDataFlash(st, data_buff, DF_PARA_SIZE);
            break;
        }

        st -= DF_PARA_SIZE;

        if (st < DF_ADDR_START) {
            // 該当無し　初めて使用
            for (i = 0; i < DF_PARA_SIZE; i++) data_buff[i] = 0;
            data_buff[DF_CHECK] = 0x11;  // 0x20
            data_buff[DF_STOP] = 10;
            data_buff[DF_Start_dly] = 10;
            data_buff[DF_kp] = 20;
            data_buff[DF_kd] = 0;
            data_buff[DF_SP_S] = 50;
//            data_buff[DF_SP_C] = 44;
            data_buff[DF_SP_CL] = 16;
            data_buff[DF_SP_RC] = 20;

            blockEraseDataFlash(DF_ADDR_START);
            writeDataFlash(DF_ADDR_START, data_buff, DF_PARA_SIZE);
            break;
        }
    }
}

/************************************************************************/
/* DataFlashへパラメータ書き込み                                        */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void writeDataFlashParameter(void) {
    unsigned int st = DF_ADDR_START;
    signed char c;

    while (1) {
        // 書き込む番地を探す
        readDataFlash(st, &c, 1);
        if (c == -1) {
            writeDataFlash(st, data_buff, DF_PARA_SIZE);
            break;
        }

        st += DF_PARA_SIZE;

        if (st > DF_ADDR_END) {
            // すべて使用したら、イレーズして先頭に書き込み
            blockEraseDataFlash(DF_ADDR_START);
            writeDataFlash(DF_ADDR_START, data_buff, DF_PARA_SIZE);
            break;
        }
    }
}

/************************************************************************/
/* LCDとスイッチを使ったパラメータセット処理                            */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
int lcdProcess(void) {
    int i;
    char sw = 0;  // LCDスイッチ情報用

    // printf("lcd_pattern=%d",lcd_pattern );
    // printf(" pattern=%d\n",pattern );

#if 0
    if (pattern != 0) {
        if (cnt_lcd >= 250) {
            cnt_lcd = 0;
            lcdPosition(0, 0);
            /* 0123456789abcbef 1行16文字 */
            lcdPrintf("pattern = %3d   ", pattern);
            /* 01234567..89abcde.f 1行16文字 */
            lcdPrintf("sensor=%02x bar=%d ", sensor_inp(), startbar_get());
        }
        return;
    }
#endif

    // スイッチ情報取得
    sw = getSwNow();
    //    printf("    sw=%d", sw);

    // メニュー＋１
    if (sw == MENU_UP) {
        lcd_pattern++;
        wait_ms(200);

        if (lcd_pattern == 11) lcd_pattern = 1;
    }

    // メニュー－１
    if (sw == MENU_DOWN) {
        lcd_pattern--;
        wait_ms(200);

        if (lcd_pattern == 0) lcd_pattern = 10;
    }

    /* LCD、スイッチ処理 */
    switch (lcd_pattern) {
        case 1:
            /* 走行停止距離調整 */
            servoPwmOut(0);
            i = data_buff[DF_STOP];
            if (sw == DATA_UP) {
                i++;
                if (i > 100) i = 100;
            }
            if (sw == DATA_DOWN) {
                i--;
                if (i < 0) i = 0;
            }
            data_buff[DF_STOP] = i;

            /* LCD処理 */
            lcdPosition(0, 0);
            lcdPrintf("01 Stop L[m]=%03d", i);
            lcdPosition(0, 1);
            lcdPrintf("Encoder = %03d   ", lEncoderTotal);
            break;

        case 2:
            /* スタート待ち時間調整 */
            servoPwmOut(0);

            i = data_buff[DF_Start_dly];
            if (sw == DATA_UP) {
                i++;
                if (i > 100) i = 100;
            }
            if (sw == DATA_DOWN) {
                i--;
                if (i < 0) i = 0;
            }
            data_buff[DF_Start_dly] = i;

            /* LCD処理 */
            lcdPosition(0, 0);
            lcdPrintf("02 St time = %03d", i);
            lcdPosition(0, 1);
            lcdPrintf("L=%1d C=%1d R=%1d %1d   ", SENS_LL, SENS_C, SENS_RR,
                      SENS_ALL);
            cnt1 = 0;
            break;

        case 3:
            /* トレース比例制御調整 */
            // servoPwmOut(iServoPwm);
            servoPwmOut(0);

            i = data_buff[DF_kp];
            if (sw == DATA_UP) {
                i++;
                if (i > 100) i = 100;
            }
            if (sw == DATA_DOWN) {
                i--;
                if (i < 0) i = 0;
            }
            data_buff[DF_kp] = i;

            /* LCD処理 */
            lcdPosition(0, 0);
            lcdPrintf("03 Trace kp =%03d", i);
            lcdPosition(0, 1);
            lcdPrintf("L=%4d  R=%4d   ", ANA_SENS_L, ANA_SENS_R);
            // lcdPrintf("cnt1=%4d" , cnt1);

            if (RUN_SW == 1) {
                //  パラメータ保存
                writeDataFlashParameter();
                lcdPosition(0, 0);
                lcdPrintf("para set           ");
                lcdPosition(0, 1);
                lcdPrintf("OK!                      ");
                while (1) {
                    servoPwmOut(iServoPwm);
                }
            }
            break;

        case 4:
            /* トレース微分制御調整 */
            // servoPwmOut(-iServoPwm);
            servoPwmOut(0);
            i = data_buff[DF_kd];
            if (sw == DATA_UP) {
                i++;
                if (i > 100) i = 100;
            }
            if (sw == DATA_DOWN) {
                i--;
                if (i < 0) i = 0;
            }
            data_buff[DF_kd] = i;

            /* LCD処理 */
            lcdPosition(0, 0);
            lcdPrintf("04 Trace kd =%03d", i);
            // lcdPosition(0, 1);
            // lcdPrintf("AS()=%4d  %4d  ",getAnalogSensor(),iServoPwm);
            lcdPosition(0, 1);
            lcdPrintf("courceOut=%2d       ", courceOut());
            break;

        case 5:
            /* 直線走行目標速度設定 */
            servoPwmOut(0);

            i = data_buff[DF_SP_S];
            if (sw == DATA_UP) {
                i++;
                if (i > 127) i = 127;
            }
            if (sw == DATA_DOWN) {
                i--;
                if (i < 0) i = 0;
            }
            data_buff[DF_SP_S] = i;

            /* LCD処理 */
            lcdPosition(0, 0);
            lcdPrintf("05 Speed_S = %3d", i);
            // lcdPosition(0, 1);
            // lcdPrintf("courceOut=%2d        ",courceOut());

            lcdPosition(0, 1);
            lcdPrintf("Bar Angle = %4d", BAR_ANGLE);

            break;

        case 6:
            /* カーブ走行目標速度設定 */
            servoPwmOut(0);

//            i = data_buff[DF_SP_C];
            if (sw == DATA_UP) {
                i++;
                if (i > 100) i = 100;
            }
            if (sw == DATA_DOWN) {
                i--;
                if (i < 0) i = 0;
            }
 //           data_buff[DF_SP_C] = i;

            /* LCD処理 */
            lcdPosition(0, 0);
 //           lcdPrintf("06 Speed_C = %3d", i);
            lcdPrintf("06                    ");

            lcdPosition(0, 1);
            lcdPrintf("Slope Angle=%4d", SLOPE_ANGLE);
            break;

        case 7:
            /* クランク進入目標速度設定 */
            servoPwmOut(0);

            i = data_buff[DF_SP_CL];
            if (sw == DATA_UP) {
                i++;
                if (i > 100) i = 100;
            }
            if (sw == DATA_DOWN) {
                i--;
                if (i < 0) i = 0;
            }
            data_buff[DF_SP_CL] = i;

            /* LCD処理 */
            lcdPosition(0, 0);
            lcdPrintf("07 Speed_CL =%3d", i);
            lcdPosition(0, 1);
            lcdPrintf("");
            break;

        case 8:
            /* レーンチェンジ進入目標速度設定 */
            servoPwmOut(0);

            i = data_buff[DF_SP_RC];
            if (sw == DATA_UP) {
                i++;
                if (i > 100) i = 100;
            }
            if (sw == DATA_DOWN) {
                i--;
                if (i < 0) i = 0;
            }
            data_buff[DF_SP_RC] = i;

            /* LCD処理 */
            lcdPosition(0, 0);
            lcdPrintf("08 Speed_RC =%3d", i);
            lcdPosition(0, 1);
            lcdPrintf("");
            break;

        case 9:
            /* 設定パラメーター保存 */
            servoPwmOut(0);

            lcdPosition(0, 0);
            lcdPrintf("09 Parameter Set");

            // 設定値保存
            if (RUN_SW == 0) {
                cnt1 = 0;
                do {
                    // スイッチ情報取得
                    sw = getSwNow();
                    wait_ms(200);
                    if (cnt1 > 2000) {
                        // パラメータ保存
                        writeDataFlashParameter();
                        lcdPosition(0, 1);
                        lcdPrintf("Set Complete    ");
                    } else {
                        lcdPosition(0, 1);
                        lcdPrintf("Setting Now     ");
                    }
                } while (RUN_SW == 0);
            } else {
                lcdPosition(0, 1);
                lcdPrintf("                ");
            }
            break;

        case 10:
            /* モーターテストドライバ基板確認 */

            /* LCD処理 */
            lcdPosition(0, 0);
            lcdPrintf("10 Motor_Test   ");
            lcdPosition(0, 1);
            lcdPrintf("SW_1/2 ON!      ");

            if (RUN_SW == 1 && sw == DATA_UP ||RUN_SW == 1 && DATA_DOWN) {
                do {
                    // スイッチ情報取得
                    sw = getSwNow();
                    wait_ms(200);
                    lcdPosition(0, 1);
                    lcdPrintf("SW_1/2 OFF!     ");
                } while (RUN_SW == 1 && sw == DATA_UP || RUN_SW == 1 && sw == DATA_DOWN);
                wait_ms(15);
                cnt1 = 0;
                while (sw == 0) {
                    // スイッチ情報取得
                    sw = getSwNow();
                    wait_ms(200);

                    servoPwmOut(0);

                    // 基板テスト
                    motor2_f(100, 100);  // 前 （左,右）
                    motor2_r(100, 100);  // 後（左,右）

                    servoPwmOut(100);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test CW   ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", 100);
                    wait_ms(2000);

                    motor2_f(0, 0);  // 前 （左,右）
                    motor2_r(0, 0);  // 後（左,右）
                    servoPwmOut(0);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test STOP ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", 0);
                    wait_ms(1000);

                    motor2_f(-100, -100);  // 前 （左,右）
                    motor2_r(-100, -100);  // 後（左,右）
                    servoPwmOut(-100);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test CCW   ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", -100);
                    wait_ms(2000);

                    motor2_f(0, 0);  // 前 （左,右）
                    motor2_r(0, 0);  // 後（左,右）
                    servoPwmOut(0);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test STOP ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", 0);
                    wait_ms(1000);
                    if (cnt1 > 3000) break;
                }
                motor_f(0, 0);  // 前 （左,右）
                motor_r(0, 0);  // 後（左,右）
                do {
                    // スイッチ情報取得
                    sw = getSwNow();
                    wait_ms(200);
                } while (sw == 0x01 || sw == 0x02);
            }
            break;

        case 11:
            /* モーターテストドライバ基板確認 */

            /* LCD処理 */
            lcdPosition(0, 0);
            lcdPrintf("10 Motor_Test   ");
            lcdPosition(0, 1);
            lcdPrintf("SW_1 ON!        ");

            if (sw == 0x01 || sw == 0x02) {
                do {
                    // スイッチ情報取得
                    sw = getSwNow();
                    wait_ms(200);
                    lcdPosition(0, 1);
                    lcdPrintf("SW_1 OFF!       ");
                } while (sw == 0x01 || sw == 0x02);
                wait_ms(15);
                i = 4;
                while (sw == 0 || i < 8) {
                    // スイッチ情報取得
                    sw = getSwNow();
                    wait_ms(200);

                    servoPwmOut(0);

                    // 基板テスト
                    if (i < 10)
                        i++;
                    else
                        i = 4;
                    motor2_f(i * 10, i * 10);  // 前 （左,右）
                    motor2_r(i * 10, i * 10);  // 後（左,右）
                    servoPwmOut(i * 10);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test CW   ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", i * 10);
                    wait_ms(2000);

                    motor2_f(0, 0);  // 前 （左,右）
                    motor2_r(0, 0);  // 後 （左,右）
                    servoPwmOut(0);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test STOP ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", 0);
                    wait_ms(1000);

                    motor2_f(-i * 10, -i * 10);  // 前 （左,右）
                    motor2_r(-i * 10, -i * 10);  // 後（左,右）
                    servoPwmOut(-i * 10);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test CCW   ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", -i * 10);
                    wait_ms(2000);

                    motor2_f(0, 0);  // 前 （左,右）
                    motor2_r(0, 0);  // 後 （左,右）
                    servoPwmOut(0);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test STOP ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", 0);
                    wait_ms(1000);
                }
                motor_f(0, 0);  // 前 （左,右）
                motor_r(0, 0);  // 後 （左,右）
                do {
                    // スイッチ情報取得
                    sw = getSwNow();
                    wait_ms(200);
                } while (sw == 0x01 || sw == 0x02);
            }
            break;
    }
}

void mtTest() {
    // 100% CW
    motor2_f(100, 100);
    motor2_r(100, 100);
    servoPwmOut(0);

    lcdPosition(0, 0);
    lcdPrintf("Motor Test CW   ");
    lcdPosition(0, 1);
    lcdPrintf("Power = %4d%%   ", 100);
    wait_ms(2000);


    // 100% STOP
    motor2_f(0, 0);
    motor2_r(0, 0);
    servoPwmOut(0);

    lcdPosition(0, 0);
    lcdPrintf("Motor Test STOP  ");
    lcdPosition(0, 1);
    lcdPrintf("Power = %4d%%   ", -100);
    wait_ms(2000);


    // 100% CCW
    motor2_f(-100, -100);
    motor2_r(-100, -100);
    servoPwmOut(0);

    lcdPosition(0, 0);
    lcdPrintf("Motor Test CCW   ");
    lcdPosition(0, 1);
    lcdPrintf("Power = %4d%%   ", -100);
    wait_ms(2000);

    // 100% STOP
    motor2_f(0, 0);
    motor2_r(0, 0);
    servoPwmOut(0);

    lcdPosition(0, 0);
    lcdPrintf("Motor Test STOP   ");
    lcdPosition(0, 1);
    lcdPrintf("Power = %4d%%   ", -100);
    wait_ms(2000);


}



int angleStreatCheck(int i,int jide_angle) {
	static int pattern11count;   /*11カウント用			 		 */
	if (abs(i) < jide_angle) {
		pattern11count++;
		if(pattern11count>75){
			return 1;
		}
	}
	else {
	    pattern11count=0;
	}
	return 0;

}






/************************************************************************/
/* end of file                                                          */
/************************************************************************/

