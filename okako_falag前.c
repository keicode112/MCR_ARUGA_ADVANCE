/****************************************************************************/
/* Copyright    ������Г����h�L�������g�\�����[�V�����Y                    */
/* 				2022.8 �}�C�R���J�[���쌧�u�K��@�t��
 * �X�C�b�`EEPROM��		*/
/****************************************************************************/
// ���s�p�^�[��
//  11:�ʏ�
//  21�E31�E41:�u���[�L����
//
// 101:�N�����N(���E)
//  �ᑬ:131
// 151:���[���`�F���W�i���E�j
// 191:�⓹(�̂ڂ邭��)
// 211:�⓹
// 231:���s��~
// 241:���O�f�[�^�o��
//  3/15 R450�����オ��Ŏ~�܂�������
//  �˃A���O�����葬�x�A�b�v�@4��8
// LCD�ύX�i�{�肭�񃂃f���ցj �C���N���[�h�t�@�C���̕ύX switch_lib_okako_cross.c
//														  switch_lib.h�i���e�ύX�j

#include <stdio.h>

#include "data_flash_lib.h" /* �f�[�^�t���b�V�����C�u����   */
#include "i2c_eeprom2015_lib.h"
#include "lcd_lib.h"
#include "personal_setting.h"
#include "printf_lib.h"
#include "sfr_r838a.h"   /* R8C/38A SFR�̒�`�t�@�C��    */
#include "switch_lib.h"  /* �X�C�b�`�ǉ�                 */
#include "18curves_array.h"

/*======================================*/
/* I/O�ݒ�                        */
/*======================================*/
#define SENS_LL !p9_0
#define ANA_SENS_L ad2
#define SENS_C !p9_1

#define ANA_SENS_R ad3
#define SENS_RR !p9_2
#define GATE !p9_2
#define SENS_ALL (SENS_LL << 2 | SENS_C << 1 | SENS_RR)
#define RUN_SW !p6_0 /* RUN_SW */
#define SLOPE_ANGLE ad1>>2//11/25���
#define BAR_ANGLE (1023 - ad0)

#define L_LED p9_4   /* ���@���P�x�k�d�c  p6_4*/
#define R_LED p3_6   /* �E�@���P�x�k�d�c  p6_5�@�i�g�p�ł��Ȃ��j*/
#define CPU_LED p4_5 /* CPU�{�[�h�k�d�c		*/

#define THRESHOLD_H 500  // �A�i���O�Z���T(0�`1023)�����肵�����l
#define THRESHOLD_L 300  // �A�i���O�Z���T(0�`1023)�����肵�����l

#define ON 1
#define OFF 0

/* ���̑����o�͗p */
#define MTCT_SW p1_3 /* ���[�^�[�R���g���[���r�v(p1_3)	*/
/*======================================*/
/* �V���{����`                         */
/*======================================*/
/* �萔�ݒ� */
#define TRC_MOTOR_CYCLE 20000 /* ���O,�E�O���[�^PWM�̎���     */
                              /* 50[ns] * 20000 = 1.00[ms]    */
#define TRD_MOTOR_CYCLE 20000 /* ����,�E��,����Ӱ�PWM�̎���   */
                              /* 50[ns] * 20000 = 1.00[ms]    */
//#define FREE 1                /* ���[�^���[�h�@�t���[  (�g�p���Ȃ�)       */
//#define BRAKE 0               /* ���[�^���[�h�@�u���[�L  (�g�p���Ȃ�)        */

/* DataFlash�֘A */
#define DF_ADDR_START 0x3000 /* �������݊J�n�A�h���X         */
#define DF_ADDR_END 0x33ff   /* �������ݏI���A�h���X         */

#define DF_PARA_SIZE 32 /* DataFlash�p�����[�^��        */

#define DF_CHECK 0x00     /* DataFlash�`�F�b�N            */
#define DF_STOP 0x01      /* ���s��~����	               	*/
#define DF_Start_dly 0x02 /* �X�^�[�g�҂�����            	*/
#define DF_kp 0x03        /* �g���[�X��ᐧ��W��         */
#define DF_kd 0x04        /* �g���[�X��������W��         */
#define DF_SP_S 0x05      /* �������s�ڕW���x�l           */
//#define DF_SP_C 0x06      /* �J�[�u���s�ڕW���x�l         */
#define DF_SP_CL 0x07     /* �N�����N�i���ڕW���x�l       */
#define DF_SP_RC 0x08     /* ���[���`�F���W�i���ڕW���x�l */

/*======================================*/
/* �v���g�^�C�v�錾                     */
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
// void led_out_org(unsigned char led);  // ���g�p
void led_out(unsigned char led);
void motor_r(int accele_l, int accele_r);
void motor2_r(int accele_l, int accele_r);
void motor_f(int accele_l, int accele_r);
void motor2_f(int accele_l, int accele_r);
// void motor_mode_r(int mode_l, int mode_r);  // ���g�p
// void motor_mode_f(int mode_l, int mode_r);  // ���g�p
void servoPwmOut(int pwm);
int check_crossline(void);
int check_rightline(void);
int check_leftline(void);


int diff_fi(int pwm);  // front_in���֍������߂�
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

void wait_ms(unsigned long timer_set);  // 1ms���ԑ҂��֐�(���荞�ݖ��g�p)
unsigned int abs(int i);

int courceOut(void);  // �R�[�X�n�Y������֐�
void mtTest(void);    // ���[�^�e�X�g
int angleStreatCheck(int i,int jide_angle) ;//�u���[�L���̃m�C�Y�΍�


int isSensllON=OFF;//����ON����@�@SENS_LL
int isSensrrON=OFF;//�E��ON����@�@SENS_RR

int isSensllCount=0;
int isSensrrcount=0;


/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
// const char *C_DATE = __DATE__; /* �R���p�C���������t           */
// const char *C_TIME = __TIME__; /* �R���p�C����������           */

int pattern = 0; /* �}�C�R���J�[����p�^�[��     */

unsigned long cnt1;          /* �^�C�}�p                     */
unsigned long cnt2;          /* �^�C�}�p                     */

unsigned long check_sen_cnt; /* �^�C�}�p                     */
unsigned long check_enc_cnt; /* �^�C�}�p                     */
unsigned long check_ana_cnt; /* �^�C�}�p                     */
unsigned long cnt_lcd;       /* LCD�����Ŏg�p                */

/* ���s���[�h�E���ԏ����� */
int isBrakeOn;  // �u���[�L����t���O
int crankMode = 0;  // �N�����N����   1:�N�����N���[�h 0:�ʏ�
char crankDirection = 'N';  // �N�����N�̕��� R:�E L:��
int laneMode = 0;          // ���[������
char laneDirection = 'N';  // ���[���̕��� R:�E L:��
int slopeTotalCount=0;//��ʉߐ��i�Q�x�ʉߖh�~�j

long slopeFinTime = 0;//�o���̈���҂�
int laneClearTime=0;//���[����̃u���[�L�h�~
int crankClearTime=0;//�N�����N��̃u���[�L�h�~
int mtPower=0;//�R�[�i�����オ�菙�X��
int temp;

int lane_count=0;
int crank_count=0;


/* �G���R�[�_�֘A */
int iTimer10;              /* 10ms�J�E���g�p               */
long lEncoderTotal;        /* �ώZ�l�ۑ��p                 */
int iEncoder;              /* 10ms���̍ŐV�l               */
unsigned int uEncoderBuff; /* �v�Z�p�@���荞�ݓ��Ŏg�p     */
long lEncoderBuff;//�G���R�[�_�̒l�擾�i��������p�j

/* ���x�ړ����ώZ�o�p�ϐ�*/
signed int sp;						/* ���x�@�ړ����ϒl  */
signed int spSum=0;				/* ���x�@�ړ����ϒl���Z�p�ϐ� */
signed int spBuf[4];				/* ���x�@�ړ����ϒl���Z�p�ϐ� */
signed int spCount=0;				/* ���x�@�ړ����ϒl���Z�p�ϐ� */


/* �⓹���o�ړ����ώZ�o�p�ϐ�*/
signed int sakaSum=0;				/* �⓹���o�Z���T�@�ړ����ϒl���Z�p�ϐ� */
signed int sakaBuf[8];				/* �⓹���o�Z���T�@�ړ����ϒl���Z�p�ϐ� */
signed int sakaCount=0;			/* �⓹���o�Z���T�@�ړ����ϒl���Z�p�ϐ� */
signed int sakaTemp;

/*  �T�[�{�֘A */
signed int iSensorBefore; /* �O��̃Z���T�l�ۑ�           */
signed int iServoPwm;     /* �T�[�{�o�v�l�l               */
signed int iAngle0;       /* ���S����A/D�l�ۑ�            */
signed int iAngle2;       /* �X�e�A�����O�p���x     */
signed int iAngleBuff;    /* �v�Z�p�@���荞�ݓ��Ŏg�p     */

/* �T�[�{�֘A2 */
int iSetAngle;
int iAngleBefore2;
int iServoPwm2;
int cource = 0;  // �R�[�X�n�Y���l

/* �Z���T�֘A */
int iSensorPattern; /* �Z���T��ԕێ��p (�g�p���Ȃ�)            */

/* �f�[�^�ۑ��֘A */
volatile int saveIndex;            /* �ۑ��C���f�b�N�X             */
volatile int saveSendIndex;        /* ���M�C���f�b�N�X             */
volatile int saveFlag;             /* �ۑ��t���O                   */
volatile signed char saveData[16]; /* �ꎞ�ۑ��G���A               */
volatile char data_lost;           /* �ۑ����s�f�[�^��				*/

/* DataFlash�֌W */
signed char data_buff[16];

/* LCD�֘A */
int lcd_pattern = 1;

/* TRC���W�X�^�̃o�b�t�@ */
unsigned int trcgrb_buff; /* TRCGRB�̃o�b�t�@             */
unsigned int trcgrc_buff; /* TRCGRC�̃o�b�t�@             */

/* ���[�^�h���C�u���TypeS Ver.3���LED�A�f�B�b�v�X�C�b�`���� */
// �g�p���Ȃ�
unsigned char types_led;   /* LED�l�ݒ�                    */
unsigned char types_dipsw; /* �f�B�b�v�X�C�b�`�l�ۑ�       */

// �g�p���Ȃ�
/* ���֍��l�v�Z�p�@�e�}�C�R���J�[�ɍ��킹�čČv�Z���ĉ����� */
const int revolution_difference[] = {/* �p�x������ցA�O�։�]���v�Z */
                                     100, 98, 97, 95, 94, 92, 91, 89, 88, 87,
                                     85,  84, 82, 81, 80, 78, 77, 76, 74, 73,
                                     72,  70, 69, 68, 66, 65, 64, 62, 61, 60,
                                     58,  57, 56, 54, 53, 52, 50, 49, 48, 46,
                                     45,  43, 42, 40, 39, 38};

/*����(�O���[�o��)�ϐ�*/
// ���֍��֌W�z��
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



/* ���C���v���O����                                                     */



/************************************************************************/
void main(void) {
    signed int i, ret;
    //    char fileName[8 + 1 + 3 + 1]; /* ���O�{'.'�{�g���q�{'\0'      */
    //	char s[50];
    unsigned char b;

    /* �}�C�R���@�\�̏����� */
    init();                         /* ������                       */
    initI2CEeprom();                /* EEP-ROM�����ݒ�      		*/
    init_uart0_printf(SPEED_38400); /* UART0��printf�֘A�̏�����    */

    asm(" fset I "); /* �S�̂̊��荞�݋���           */
    initLcd();
    initSwitch(); /* �X�C�b�`������               */
    // initBeepS();  /* �u�U�[�֘A����               */

    readDataFlashParameter(); /* DataFlash�p�����[�^�ǂݍ���  */

    /* �}�C�R���J�[�̏�ԏ����� */
    printf("start\n");
    motor_f(0, 0);
    motor_r(0, 0);
    servoPwmOut(0);

    // ���Z�b�g����m�F
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

    /* �X�^�[�g���A�X�C�b�`��������Ă���΃��O�f�[�^�]�����[�h */
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
                            // iAngle0 = getServoAngle(); /* 0�x�̈ʒu�L�� */
         iAngle0 = VR_CENTER;  // �Z���^�[�l�Œ�
         iSetAngle = -100; /* +�ō� -�ŉE�ɋȂ���܂�      */
         servoPwmOut(iServoPwm2);
                    //wait_ms(2000);
    }
    iAngle0=getServoAngle();

    while (RUN_SW == OFF) {
        //i = getServoAngle();//�X�e�A�����O�p�x�擾
        i =VR_CENTER;//�Z���^�[�l�Œ�
        lcdPosition(0, 1);
        lcdPrintf("angle = %4d%   ",i);
    }


    while(1){//���֍��`�F�b�N
            i =VR_CENTER;//�Z���^�[�l�Œ�
            i = getServoAngle();//�X�e�A�����O�p�x�擾

            if (i >0 ) {
             //���x����
                  motor_f(diff(80), 80);
                  motor_r(diff(80), 80);
             }
             else if(i < 0){
             //���x����
                  motor_f(80,diff(80));
                  motor_r(80,diff(80));
             }
             else{
                  motor_f(80,80);
                  motor_r(80,80);
             }
    }
    /* �g���[�X�������䒲�� */
        while(1){
               servoPwmOut(iServoPwm);
               motor_f(0,0);
               motor_r(0,0);
        }

#endif

    while (1) {//�������[�v
        //printf("pattern=%d\n",pattern);
        I2CEepromProcess(); /* I2C EEP-ROM�ۑ�����          */


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
            /* �N���X���C���`�F�b�N         */
            if (check_crossline()) {
                cnt1 = 0;
                crankMode = 1;
                pattern = 101;
            }

            /* ���n�[�t���C���`�F�b�N       
            if (check_leftline() &&  abs(getServoAngle())<8) {
                cnt1 = 0;
                laneMode = 1;
                pattern = 151;
                laneDirection = 'L';
            }
			*/


            /* �E�n�[�t���C���`�F�b�N       */
            if (check_rightline()==1 &&  abs(getServoAngle())<15) {
                cnt1 = 0;
                laneMode = 1;
                pattern = 151;
                laneDirection = 'R';
            }

		    /* �o�⌟�o      */
    		if( (SLOPE_ANGLE >  SLOPE_UP_START-5) && slopeTotalCount==0 ){
					pattern=191;	//�⑖�s������	 �̂ڂ邭��
		}

    		/* ����⌟�o */
			if((SLOPE_ANGLE < SLOPE_DOWN_START+5) && slopeTotalCount==1 &&slopeFinTime==0) {
					pattern=211;	//�⑖�s������	 �̂ڂ邭��
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
                /* �v�b�V���X�C�b�`�����҂� */

                servoPwmOut(0);

                // LCD�\���A�p�����[�^�ݒ菈��
                lcdProcess();
                //    if (pushsw_get()) {

                if (RUN_SW == ON) {
                    clearI2CEeprom(&p2, 7);
                    //  setBeepPatternS(0xcc00);
                    //   �p�����[�^�ۑ�
                    writeDataFlashParameter();
                    printf("writeDataFlashParameter");
                    cnt1 = 0;
                    //pattern = 3;//�Q�[�g�Z���T����

                    pattern = 3;//�Q�[�g�Z���T�L��i�艟���j
                    break;
                }

                // led_out(i); /* LED�_�ŏ���                  */
                break;

            case 3:
                /* �X�^�[�g�o�[�J�҂� */
                servoPwmOut(iServoPwm / 2);
                //                if (!startbar_get()) {
                if (GATE == OFF) {
                    // iAngle0 = getServoAngle(); /* 0�x�̈ʒu�L�� */
                    iAngle0 = VR_CENTER;  // �Z���^�[�l�Œ�

                    // led_out(0x0);
                    CPU_LED = OFF;
                    cnt1 = 0;
                    saveIndex = 0;
                    //saveFlag = 1; //pattern :5 �ōs���@/* �f�[�^�ۑ��J�n               */
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
                /* �X�^�[�gSW�҂� */
                servoPwmOut(iServoPwm / 2);
                //                if (!startbar_get()) {
                if (RUN_SW == ON) {
                    // iAngle0 = getServoAngle(); /* 0�x�̈ʒu�L�� */
                    iAngle0 = VR_CENTER;  // �Z���^�[�l�Œ�

                    // led_out(0x0);
                    CPU_LED = OFF;
                    cnt1 = 0;
                    saveIndex = 0;
                    //saveFlag = 1; //pattern :5 �ōs���@/* �f�[�^�ۑ��J�n               */
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
                    saveFlag = 1; /* �f�[�^�ۑ��J�n               */
                    check_sen_cnt = 0;
                    check_enc_cnt = 0;
                }
                servoPwmOut(iServoPwm);  // ���C���g���[�X����

                break;

/************************************************************************/



/* �ʏ푖�s���� */



/************************************************************************/
            case 11:
                /* �ʏ�g���[�X */
                servoPwmOut(iServoPwm);  // ���C���g���[�X����
                i = getServoAngle();     // �X�e�A�����O�p�x�擾

                if (abs(i)< 5) {  // �u���[�L���t���OON
                    isBrakeOn = 1;
                }
                if ((abs(i) >8  && abs(iAngle2) > 6) && isBrakeOn == 1 && laneClearTime==0 && crankClearTime==0 ) {
                    pattern = 12;  // �u���[�L������
                    isBrakeOn = 0;
                    lEncoderBuff = lEncoderTotal;
                    break;
                }

                // �ʏ푖�s��
                if (i > 110) {  // �ԑ̋�̎��@���[�^��~
                    motor_f(0, 0);
                    motor_r(0, 0);
                } else if (i > 25) {
                    // ���x����
                    if (iEncoder > data_buff[DF_SP_S] + 5) {
                        motor_f(-5, -5);  // �O (��,�E)
                        motor_r(-5, -5);  // �� (��,�E)
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
                    // ���x����
                    if (iEncoder > data_buff[DF_SP_S] + 5) {
                        motor_f(-5, -5);  // �O (��,�E)
                        motor_r(-5, -5);  // �� (��,�E)
                    } else if (iEncoder > data_buff[DF_SP_S]) {
                        motor_f(1, 1);
                        motor_r(1, 1);
                    } else {
                        motor_f(85, diff_fi(85));
                        motor_r(diff_ro(85), diff_ri(85));
                    }
                }

                else {
                    // ���x����
                    if (iEncoder > data_buff[DF_SP_S] + 5) {
                        motor_f(-5, -5);  // �O (��,�E)
                        motor_r(-5, -5);  // �� (��,�E)
                    } else if (iEncoder > data_buff[DF_SP_S]) {
                        motor_f(1, 1);
                        motor_r(1, 1);
                    }
                    // ���x����
                    else {
                        motor_f(100, 100);
                        motor_r(100, 100);
                    }
                }

                break;

/************************************************************************/



/* �u���[�L����          */



/************************************************************************/

            case 12:  //	�X�g���[�g��̃u���[�L�����@�@�@�t�����g�u���[�L���߁@100mm���x
                // �����u���[�L�̂ݍ��E���قȂ邪���̌�̃u���[�L�o�����X�́A���E���l
                servoPwmOut(iServoPwm);  // ���C���g���[�X����
                i = getServoAngle();     // �X�e�A�����O�p�x�擾

                if (i >= 0) {              // ���R�[�i�[
                    if (70 <= iEncoder) {  //------���u���[�L 4.7[m/s]
                        pattern = 21;
                        lEncoderBuff = lEncoderTotal;  //    kyori = 300;        // 200mm
                        cnt1=0;
                        motor_f(-85, -85);  // �O�i��,�E)
                        motor_r(-65, -65);  // ��i��,�E)
                    } else if (65 <= iEncoder) {  //-���u���[�L 4.3[m/s]
                        pattern = 31;
                        lEncoderBuff =lEncoderTotal;  //    kyori = 225;        // 150mm
                        motor_f(-60, -60);  // �O�i��,�E)
                        motor_r(-50, -50);  // ��i��,�E)
                    } else {                //-------------��u���[�L
                        pattern = 41;       // pattern = 41;
                        lEncoderBuff =lEncoderTotal;  //    kyori = 151;        // 100mm
                        motor_f(-25, -25);  // �O�i��,�E)
                        motor_r(-15, -15);  // ��i��,�E)
                    }
                }

                else {                     // �E�R�[�i�[
                    if (70 <= iEncoder) {  //------���u���[�L 4.7[m/s]
                        pattern = 21;
                        lEncoderBuff =lEncoderTotal;  //    kyori = 300;        // 200mm
                        cnt1=0;
                        motor_f(-85, -85);  // �O�i��,�E)
                        motor_r(-65, -65);  // ��i��,�E)
                    } else if (65 <= iEncoder) {  //-���u���[�L 4.3[m/s]
                        pattern = 31;             // pattern = 31;
                        lEncoderBuff = lEncoderTotal;  //    kyori = 225;        // 150mm
                        motor_f(-60, -60);  // �O�i��,�E)
                        motor_r(-50, -50);  // ��i��,�E)
                    } else {                //-------------��u���[�L
                        pattern = 41;       // pattern = 41;
                        lEncoderBuff = lEncoderTotal;  //    kyori = 151;        // 100mm
                        motor_f(-25, -25);  // �O�i��,�E)
                        motor_r(-15, -15);  // ��i��,�E)
                    }
                }
                break;

            // ���u���[�L
            case 21:  //	�X�g���[�g��̋��u���[�L����
                servoPwmOut(iServoPwm);  // ���C���g���[�X����
                i = getServoAngle();     // �X�e�A�����O�p�x�擾
                if (lEncoderTotal - lEncoderBuff >= 350) {  // 200m
                //if(cnt1>30){
                    lEncoderBuff = lEncoderTotal;
                    pattern = 23;
                    break;
                }
                
                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // �ʏ폈����
                }
                break;

            case 23:  //	�X�g���[�g��̋��u���[�L����
                servoPwmOut(iServoPwm);  // ���C���g���[�X����
                i = getServoAngle();     // �X�e�A�����O�p�x�擾

                motor_f(-65, -65);  // �O�i��,�E)
                motor_r(-35, -35);  // ��i��,�E)

                if (lEncoderTotal - lEncoderBuff >= 225) {  // 150m
                    lEncoderBuff = lEncoderTotal;
                    pattern = 24;
                    break;
                }

                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // �ʏ폈����
                }

                break;


            case 24:  //	�X�g���[�g��̋��u���[�L����
                servoPwmOut(iServoPwm);  // ���C���g���[�X����
                i = getServoAngle();     // �X�e�A�����O�p�x�擾

                motor_f(1, 1);  // �O�i��,�E)
                motor_r(1, 1);  // ��i��,�E)

                if (lEncoderTotal - lEncoderBuff >= 250) {  // 375mm
                    mtPower=0;
                    pattern = 25;
                    break;
                }

                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // �ʏ폈����
                }

                break;

            case 25:  //	�X�g���[�g��̋��u���[�L����
                servoPwmOut(iServoPwm);  // ���C���g���[�X����
                i = getServoAngle();     // �X�e�A�����O�p�x�擾

                if(mtPower>CORNER_PWM){
                    mtPower=CORNER_PWM;
                }                

                if (i >= 0) {  // ���R�[�i�[
                    if(abs(i)>CORNER_FREEPWM_ANGLE){//�R�[�i�[�@���[�^�t���[�i�p�[�V�����j����p�x
                        motor_f(1, 1);
                        motor_r(1, 1);                        
                    }
                    else{
                        motor_f(diff_fi(mtPower), mtPower);
                        motor_r(diff_ri(mtPower), diff_ro(mtPower));
                    }
                } else {  // �E�R�[�i�[
                    if(abs(i)>CORNER_FREEPWM_ANGLE){//�R�[�i�[�@���[�^�t���[�i�p�[�V�����j����p�x
                        motor_f(1, 1);
                        motor_r(1, 1);                        
                    }
                    else{
                        motor_f(mtPower, diff_fi(mtPower));
                        motor_r(diff_ro(mtPower), diff_ri(mtPower));
                    }
                }


                if(angleStreatCheck(i,STREAT_JUDGE_ANGLE)==1){//true
                    pattern = 11;  // �ʏ폈����
                }

                break;

            // ���u���[�L
            case 31:  //	�X�g���[�g��̒��u���[�L����
                servoPwmOut(iServoPwm);  // ���C���g���[�X����
                i = getServoAngle();     // �X�e�A�����O�p�x�擾

                if (lEncoderTotal - lEncoderBuff >= 338) {  // 200m
                //if(cnt1>30){
                    lEncoderBuff = lEncoderTotal;
                    pattern = 33;
                    break;
                }
                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // �ʏ폈����
                }

                break;

            case 33:  //	�X�g���[�g��̒��u���[�L����
                servoPwmOut(iServoPwm);  // ���C���g���[�X����
                i = getServoAngle();     // �X�e�A�����O�p�x�擾

                motor_f(0, 0);      // �O�i��,�E)
                motor_r(-10, -10);  // ��i��,�E)

                if (lEncoderTotal - lEncoderBuff >= 263) {  // 150m
                    pattern = 34;
                    break;
                }
                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // �ʏ폈����
                }

                break;


            case 34:  //	�X�g���[�g��̒��u���[�L����
                servoPwmOut(iServoPwm);  // ���C���g���[�X����
                i = getServoAngle();     // �X�e�A�����O�p�x�擾

                motor_f(1, 1);  // �O�i��,�E)
                motor_r(1, 1);  // ��i��,�E)

                if (lEncoderTotal - lEncoderBuff >= 150) {  // 200mm(300) ���@100m(150)�@�i���X�ɉ����j
                    mtPower=0;
                    pattern = 35;
                    break;
                }

                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // �ʏ폈����
                }

                break;

            case 35:  //	�X�g���[�g��̒��u���[�L����
                servoPwmOut(iServoPwm);  // ���C���g���[�X����
                i = getServoAngle();     // �X�e�A�����O�p�x�擾


                if(mtPower>CORNER_PWM){
                    mtPower=CORNER_PWM;
                }                

                if (i >= 0) {  // ���R�[�i�[
                    if(abs(i)>CORNER_FREEPWM_ANGLE){
                        motor_f(1, 1);
                        motor_r(1, 1);                        
                    }
                    else{
                        motor_f(diff_fi(mtPower), mtPower);
                        motor_r(diff_ri(mtPower), diff_ro(mtPower));
                    }
                } else {  // �E�R�[�i�[
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
                    pattern = 11;  // �ʏ폈����
                }
                break;

            // ��u���[�L
            case 41:  //	�X�g���[�g��̎�u���[�L����
                servoPwmOut(iServoPwm);  // ���C���g���[�X����
                i = getServoAngle();     // �X�e�A�����O�p�x�擾

                if (lEncoderTotal - lEncoderBuff >= 151) {  // 100m
                    lEncoderBuff = lEncoderTotal;
                    pattern = 43;
                    break;
                }
                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // �ʏ폈����
                }
                break;

            case 43:  //	�X�g���[�g��̎�u���[�L����
                servoPwmOut(iServoPwm);  // ���C���g���[�X����
                i = getServoAngle();     // �X�e�A�����O�p�x�擾

                motor_f(0, 0);      // �O�i��,�E)
                motor_r(-10, -10);  // ��i��,�E)

                if (lEncoderTotal - lEncoderBuff >= 150) {  // 200mm(300) ���@100m(150)�@�i���X�ɉ����j
                    mtPower=0;
                    pattern = 45;
                    break;
                }
                if(angleStreatCheck(i,5)==1){//true
                    pattern = 11;  // �ʏ폈����
                }

                break;

            case 45:  //	�X�g���[�g��̎�u���[�L����
                servoPwmOut(iServoPwm);  // ���C���g���[�X����
                i = getServoAngle();     // �X�e�A�����O�p�x�擾


                if(mtPower>CORNER_PWM){
                    mtPower=CORNER_PWM;
                }                

                if (i >= 0) {  // ���R�[�i�[
                    if(abs(i)>CORNER_FREEPWM_ANGLE){
                        motor_f(1, 1);
                        motor_r(1, 1);                        
                    }
                    else{
                        motor_f(diff_fi(mtPower), mtPower);
                        motor_r(diff_ri(mtPower), diff_ro(mtPower));
                    }
                } else {  // �E�R�[�i�[
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
                    pattern = 11;  // �ʏ폈����
                }

                break;

/************************************************************************/



// �N�����N���s����



/************************************************************************/

            case 101:
                /* �N���X���C���ʉߏ��� */
                servoPwmOut(iServoPwm);
                R_LED = ON;
                L_LED = ON;
                // led_out(0x3);
                motor_f(-90, -90);
                motor_r(-90, -90);
                if (cnt1 >= 30) {  // ��ǂݖh�~(120mm���x)
                    cnt1 = 0;
                    pattern = 102;
                    laneMode = 0;  // ���[�����[�h�t���O�N���A
					break;
                }
				break;

            case 102:
                lEncoderBuff = lEncoderTotal;
                pattern = 104;
                break;

            case 104:  // �N���X���C����̏���(1�i�ڂ̌�������)
                servoPwmOut(iServoPwm);
                if (iEncoder >= data_buff[DF_SP_CL] + 3) { /* �G���R�[�_�ɂ��X�s�[�h���� */
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

            case 106:  // �N�����N���� (2�i�ڂ̌�������)�@�n�[�t���C�����o
                servoPwmOut(iServoPwm);

                if (iEncoder >= data_buff[DF_SP_CL] + 2) { /* �G���R�[�_�ɂ��X�s�[�h���� */
                    motor_f(-80, -80);
                    motor_r(-50, -50);
                } else if (iEncoder >= data_buff[DF_SP_CL]) {
                    motor_f(10, 10);
                    motor_r(10, 10);
                } else {
                    motor_f(70, 70);
                    motor_r(70, 70);
                }

                if (isSensllON==ON ) {  // �N�����N�����@��
                    crankDirection = 'L';  // �N�����N�����L���ϐ������N�����N
                    iSetAngle = CRANK_ANGLE_L; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);
                    pattern = 108;
                    break;
                } else if (isSensrrON==ON) {  // �N�����N�����@�E
                    crankDirection = 'R';  // �N�����N�����L���ϐ������N�����N
                    iSetAngle = -CRANK_ANGLE_R; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);
                    pattern = 108;
                    break;
                }

                break;

            case 108:  // �N�����N����	 �@�n�[�t���C�����o��
                if (crankDirection == 'L') {  // �N�����N�����@��
                    iSetAngle = CRANK_ANGLE_L; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);

                    motor_f(-80, 1);  // �O �i��,�E�j
                    motor_r(-50, -50);  // �ヂ�[�^�i��,�E�j
                }

                else if (crankDirection == 'R') {  // �N�����N�����@�E
                    iSetAngle = -CRANK_ANGLE_R; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);

                    motor_f(1, -80);  // �O �i��,�E�j
                    motor_r(-50, -50);  // �ヂ�[�^�i��,�E�j
                }

                if (SENS_C == OFF && SENS_LL == OFF && SENS_RR == OFF) {
                    pattern = 110;  // ���Ă��޼��پݻ�S�č��̎����̏�����
                    lEncoderBuff = lEncoderTotal;
                    cnt1 = 0;
                    CPU_LED = ON;
                    // break;
                }

                break;

            case 110:  // �N�����N����	 �@�n�[�t���C�����o��

                if (crankDirection == 'L') {  // �N�����N�����@��
                    iSetAngle = CRANK_ANGLE_L; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);

                    if (isSensrrON==ON) {
                        pattern = 112;  //(�ʏ�N�����N�̏�����)�@���ĉE�ݻ(out)������
                        cnt1 = 0;  // 116:20ms�҂�

                        motor_f(1, 1);   // �O �i��,�E�j
                        motor_r(-50, 10);  // �ヂ�[�^�i��,�E�j
                        break;
                    }

                    if (isSensllON==ON && ANA_SENS_L < THRESHOLD_L &&  (lEncoderTotal - lEncoderBuff) >= 75) {
                        pattern = 131;  //(�ᑬ�i��������)�@���č��ݻ(in)������
                        cnt1 = 0;
                        break;
                        // �i��,�E�j motor_f(1,1);
                        // //�O �i��,�E�j
                        // motor_r(-40,1);
                    }



                } else if (crankDirection == 'R') {  // �N�����N�����@�E
                    iSetAngle = -CRANK_ANGLE_R; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);

                    if (isSensllON==ON) {
                        pattern =  112;  //(�ʏ�N�����N�̏�����) ���č��ݻ(out)������
                        motor_f(1, 1);   // �O �i��,�E�j
                        motor_r(-50, 10);  // �ヂ�[�^�i��,�E�j
                        cnt1 = 0;  // 116:20ms�҂�
                        break;
                    }

                    if (isSensrrON==ON && ANA_SENS_R < THRESHOLD_L &&
                        (lEncoderTotal - lEncoderBuff) >= 75) {
                        pattern = 131;  //(�ᑬ�i��������)�@���ĉE�ݻ(in)������
                        cnt1 = 0;
                        break;
                    }
                }
                break;

            case 112:
                if (crankDirection == 'L') {  // �N�����N�����@��

                    iSetAngle = CRANK_ANGLE_L; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);

                    //					motor_f(1,1);
                    ////�O
                    // �i��,�E�j
                    // motor_r(-40,1);
                    // //�ヂ�[�^�i��,�E�j 			motor_f(-50,1);
                    // //�O �i��,�E�j
                    // motor_r(-60,30);
                    // //�ヂ�[�^�i��,�E�j

                    motor_f(1, 1);  // �O �i��,�E�j
                    motor_r(-50, 10);  // �ヂ�[�^�i��,�E�j

                    // �ʏ�N�����N����
                    // ���Ă��޼��پݻ�S�č��i�E�Z���T�n�e�e�j�̎����̏����ց@�@�i�Z���T�[�g���̂��߁@���Z���T�[�������������j
                    if (SENS_C == OFF && ANA_SENS_R < THRESHOLD_L &&
                        SENS_RR == OFF && ANA_SENS_L < THRESHOLD_L &&
                        cnt1 > 20) {
                        pattern = 114;
                    }
                    // �R�[�X�A�E�g�Ή��N�����N����(���Ή����Ȃ�)
                } else if (crankDirection == 'R') {  // �N�����N�����@�E

                    iSetAngle = -CRANK_ANGLE_R; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);

                    //					motor_f(1,a1);
                    ////�O
                    // �i��,�E�j
                    // motor_r(1,-40);
                    // //�ヂ�[�^�i��,�E�j
                    motor_f(1, 1);  // �O �i��,�E�j
                    motor_r(10, -50);  // �ヂ�[�^�i��,�E�j

                    // �ʏ�N�����N����
                    // ���Ă��޼��پݻ�S�č��i���Z���T�n�e�e�j�̎����̏����ց@�i�Z���T�[�g���̂��߁@���Z���T�[�������������j
                    if (SENS_C == OFF && ANA_SENS_R < THRESHOLD_L &&
                        SENS_LL == OFF && ANA_SENS_L < THRESHOLD_L &&
                        cnt1 > 20) {
                        pattern = 114;
                    }
                    // �R�[�X�A�E�g�Ή��N�����N����(���Ή����Ȃ�)
                }
                break;

            case 114:  // �R�[�X�A�E�g���̃N�����N����
                if (crankDirection == 'L') {  // �N�����N�����@��
                    iSetAngle = CRANK_ANGLE_L; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);
                    //					motor_f(-70,1);
                    ////�O �i��,�E�j
                    motor_f(1, 20);    // �O �i��,�E�j
                    motor_r(1, 60);  // �ヂ�[�^�i��,�E�jmotor_r(-40, 1);
                    // ���Ă��޼��پݻ�S�č��i�E�Z���T�n�e�e�j�̎����̏����ց@�@�i�Z���T�[�g���̂��߁@���Z���T�[�������������j
                    if (SENS_C == OFF && ANA_SENS_R < THRESHOLD_L &&
                        SENS_RR == OFF && ANA_SENS_L < THRESHOLD_L) {
                        pattern = 116;
                        break;
                    }
                }

                else if (crankDirection == 'R') {  // �N�����N�����@�E
                    iSetAngle = -CRANK_ANGLE_R; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);
                    //					motor_f(1,-70);
                    ////�O �i��,�E�j
                    motor_f(20, 1);    // �O �i��,�E�j
                    motor_r(60, 1);  // �ヂ�[�^�i��,�E�j
                    // ���Ă��޼��پݻ�S�č��i���Z���T�n�e�e�j�̎����̏����ց@�@�i�Z���T�[�g���̂��߁@�E�Z���T�[�������������j
                    if (SENS_C == OFF && ANA_SENS_R < THRESHOLD_L &&
                        SENS_LL == OFF && ANA_SENS_L < THRESHOLD_L) {
                        pattern = 116;
                        break;
                    }
                }
                break;

            case 116:                         // �N�����N����
                if (crankDirection == 'L') {  // �N�����N�����@��
                    iSetAngle = CRANK_ANGLE_L; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);
                    motor_f(100, 70);  // �O �i��,�E)(100,70) (30, 0)
                    motor_r(50, 60);  // �ヂ�[�^(��,�E)

                    if (isSensllON==ON && ANA_SENS_L < THRESHOLD_L) {
                        pattern =
                            118;  // ���č��ݻ�i�޼��ق܂��ͱ�۸ށj���������̏�����
                        cnt1 = 0;
                        break;
                    }
                    /* �X�e�A�����O�p 0:���� +:�� -:�E 1.6=1�x */
                    //					ST_A = 0;
                    //					ST_B = 1;
                    //					ST_PWM=10;

                } else if (crankDirection == 'R') {  // �N�����N�����@�E
                    /* �X�e�A�����O�p 0:���� +:�� -:�E 1.6=1�x */
                    //					ST_A = 1;
                    //					ST_B = 0;
                    //					ST_PWM=10;
                    // ST_PWM=0;

                    iSetAngle = -CRANK_ANGLE_R; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);
                    motor_f(70, 100);  // �O �i��,�E�j(70,100) (0, 30)
                    motor_r(60, 50);  // �ヂ�[�^�i��,�E)

                    if (isSensrrON==ON && ANA_SENS_R < THRESHOLD_L) {
                        pattern =
                            118;  // ���č��ݻ�i�޼��ق܂��ͱ�۸ށj���������̏�����
                        cnt1 = 0;
                    }
                }
                break;
            case 118:
                if (crankDirection == 'L') {  // �N�����N�����@��
                    //					ST_A = 0;
                    //					ST_B = 1;
                    //					ST_PWM=0;
                    iSetAngle = CRANK_ANGLE_L/2+20; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);
                    motor_f(100, 70);  // �O �i��,�E�j(100,70)
                    motor_r(60, 50);    // �ヂ�[�^�i��,�E�j�@
                } else if (crankDirection == 'R') {  // �N�����N�����@�E
                    // �J�E���^�[�X�e�A
                    //					ST_A = 1;
                    //					ST_B = 0;
                    //					ST_PWM=0;
                    // ST_PWM=0;

                    iSetAngle = -(CRANK_ANGLE_R/2+20); /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);
                    motor_f(70, 100);  // �O �i��,�E�j(70,100)
                    motor_r(50, 60);    // �ヂ�[�^�i��,�E�j
                }
                if (cnt1 > 10 && SENS_C == ON) {  // 10ms��
                    pattern = 120;
                    crankMode = 0;  // �N�����N���[�h�N���A
                    cource = 0;  // �R�[�X�O��l0�N���A
                }
                break;

            case 120:
                /* �������Ԃ��o�܂ő҂� */
                i = getServoAngle();  // �X�e�A�����O�p�x�擾
                servoPwmOut(iServoPwm);
                motor_r(100, 100);
                motor_f(100, 100);
                if (abs(i) < 5) {
                    cnt1 = 0;
                    // led_out(0x0);
                    pattern = 11;
                    crankDirection = 0;  // �N�����N���[�h�i�N�����N�����j�N���A
                    laneMode = 0;  // ���[�����[�h�N���A
                    laneDirection = 0;  // ���[�����[�h�i���[�������j�N���A
                    crankClearTime=50;
                    break;
                }
               break;

                // �ᑬ�i�����̃N�����N�̏���
            case 131:
                if (crankDirection == 'L') {  // �N�����N�����@��
                    iSetAngle = CRANK_ANGLE_L; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);
                    motor_f(-10, 60);  // �O �i��,�E)(100,70)
                    motor_r(-10, 50);  // �ヂ�[�^(��,�E)

                } else if (crankDirection == 'R') {  // �N�����N�����@�E
                    iSetAngle = -CRANK_ANGLE_R; /* +�ō� -�ŉE�ɋȂ���܂�      */
                    servoPwmOut(iServoPwm2);
                    motor_f(60, -10);  // �O �i��,�E�j(70,100)
                    motor_r(50, -10);  // �ヂ�[�^�i��,�E)
                }
                if (cnt1 > 10 && SENS_C == ON) {
                    crankMode = 0;  // �N�����N���[�h�N���A(���C���g���[�X���[�h�����̂���)
                    pattern = 132;
                    cource = 0;  // �R�[�X�O��l0�N���A
                }
                break;

            case 132:
                /* �������Ԃ��o�܂ő҂� */
                i = getServoAngle();  // �X�e�A�����O�p�x�擾
                servoPwmOut(iServoPwm);
                motor_r(100, 100);
                motor_f(100, 100);
                if (abs(i) < 5) {
                    cnt1 = 0;
                    // led_out(0x0);
                    pattern = 11;
                    crankDirection = 0;  // �N�����N���[�h�i�N�����N�����j�N���A
                    laneMode = 0;  // ���[�����[�h�N���A
                    laneDirection = 0;  // ���[�����[�h�i���[�������j�N���A
                    crankClearTime=50;

                    break;
                }
                break;

/************************************************************************/



/* ���[���`�F���W�̏���*/




/************************************************************************/
            case 151:  // �n�[�t���C����̏����P�i���x����j
                servoPwmOut(iServoPwm);  // ���C���g���[�X���� motor_r(80, 80);
                motor_f(90, 90);
                motor_r(90, 90);
                crankMode = 1;//�X�e�A�����O����␳�Ȃ�

                lEncoderBuff = lEncoderTotal;


                if (check_crossline()) { /* �N���X���C���`�F�b�N         */
                    cnt1 = 0;
                    crankMode = 1;
                    pattern = 101;
                    break;
                }

                if(cnt1>10){//50mm�i�񂶂Ⴄ
                    pattern = 152;
                }
                break;

            case 152:  // �N���X���C����̏���(�����g���[�X��)
                i = getServoAngle();  // �X�e�A�����O�p�x�擾
                servoPwmOut(iServoPwm);

                if (iEncoder >= data_buff[DF_SP_RC] +  3) { /* �G���R�[�_�ɂ��X�s�[�h���� */
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
                    pattern = 154;  // �S�ẴZ���T�@�����o�����̏�����
                }

                // ���[���댟�m�p�̒ʏ한�A
				
                if (lEncoderTotal - lEncoderBuff >= 3000) {  // 2000m
                    pattern = 11;                            // �ʏ�ɖ߂�
                    break;
                }
				
				
				            /* �N���X���C���`�F�b�N         */
	            if (check_crossline()) {
	                cnt1 = 0;
	                crankMode = 1;
	                pattern = 101;
				    break;
	            }

				
                break;

            case 154:  // �����g���[�X�I���㏈��	�ŊO�Z���T�@�������҂�
                i = getServoAngle();  // �X�e�A�����O�p�x�擾

                if (laneDirection == 'L') {  // ���[�������@��

                    iSetAngle = LANE_ANGLE_L; /* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(-70, 20);         // �O �i��,�E-70�j
                    motor_r(-30, 1);          // ��i��,�E-30)


                    if((LANE_ANGLE_L)-5 < abs(i))  {
                            pattern=155;	//�X�e�A�����O�p�x�ڕW�l�t��
                            break;
                     }

                    if (isSensllON==ON) {
                            cnt1 = 0;
                            pattern = 156;
                    }



                }

                else if (laneDirection == 'R') {  // ���[�������@�E
                    iSetAngle = -LANE_ANGLE_R; 
					
					/* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(20, -70);         // �O �i��,�E-70�j
                    motor_r(1, -30);          // ��i��,�E-30)


                    if((LANE_ANGLE_R)-5 < abs(i))  {
                            pattern=155;	//�X�e�A�����O�p�x�ڕW�l�t��
                            break;
                     }

                    if (isSensrrON==ON) {
                            cnt1 = 0;
                            pattern = 156;
                    }
                }

                break;



            case 155:  // �����g���[�X�I���㏈��	�ŊO�Z���T�@�������҂�
                i = getServoAngle();  // �X�e�A�����O�p�x�擾

                if (laneDirection == 'L') {  // ���[�������@��

                    iSetAngle = LANE_ANGLE_L; /* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(70, 70);         // �O �i��,�E-70�j
                    motor_r(1, 1);          // ��i��,�E-30)

                    if (isSensllON==ON) {
                          cnt1 = 0;
                          pattern = 156;
                    }

                }

                else if (laneDirection == 'R') {  // ���[�������@�E
                    iSetAngle = -LANE_ANGLE_R; /* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(70, 70);         // �O �i��,�E-70�j
                    motor_r(1, 1);          // ��i��,�E-30)

                    if (isSensrrON==ON) {
                          cnt1 = 0;
                          pattern = 156;
                    }
                }


                break;

            case 156:  // �����g���[�X�I���㏈��	�ŊO�Z���T�@�������҂�
                if (laneDirection == 'L') {  // ���[�������@��
                    iSetAngle = 0; /* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // �p�x����
                    motor_f(85, 0);           // �O �i��,�E�j
                    motor_r(85, 0);           // ��i��,�E)

                }

                else if (laneDirection == 'R') {  // ���[�������@�E
                    iSetAngle = 0; /* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // �p�x���� 
                    motor_f(0, 85);           // �O �i��,�E�j
                    motor_r(0, 85);           // ��i��,�E)
                }

                if (cnt1 >= 5) {
                    pattern = 160;  // 5ms�㎟�̏�����
                    cnt1 = 0;
                }


                break;


            case 160:  // 10m�b�҂���̏����@�œ��Z���T�@���������҂�
                if (laneDirection == 'L') {         // ���[�������@��

                    iSetAngle = 0; /* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(85, 0);           // �O �i��,�E�j
                    motor_r(85, 0);           // ��i��,�E)

                    if (isSensrrON==ON) {
                        cnt1 = 0;
                        pattern = 162; /*���f�W�^���Z���T���������̏����� */
                    }


                } else if (laneDirection == 'R') {  // ���[�������@�E
                    iSetAngle = 0; /* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(0, 85);           // �O �i��,�E�j
                    motor_r(0, 85);           // ��i��,�E)

                    if (isSensllON==ON) {
                        cnt1 = 0;
                        pattern = 162; /*���f�W�^���Z���T���������̏����� */
                    }
                }
                break;

            case 162:  // �œ��Z���T������@10���b�҂�
                if (laneDirection == 'L') {         // ���[�������@��
                    iSetAngle = 0; /* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(85, 0);           // �O �i��,�E�j
                    motor_r(85, 0);           // ��i��,�E)

                } else if (laneDirection == 'R') {  // ���[�������@�E
                    iSetAngle = 0; /* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(0, 85);           // �O �i��,�E�j
                    motor_r(0, 85);           // ��i��,�E)
                }

                if (cnt1 >= 10) {
                    cnt1 = 0;
                    pattern = 164;  // 10ms�㎟�̏�����
                }
                break;

            case 164:  // 10���b�҂���̏����@�œ��Z���T�@���������҂�
                if (laneDirection == 'L') {  // ���[�������@��
                    iSetAngle = 0; /* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(85, 0);           // �O �i��,�E�j
                    motor_r(85, 0);           // ��i��,�E)

                    if (SENS_RR == OFF) {
                        pattern = 166; /*���f�W�^���Z���TOFF���������̏����� */
                        cnt1 = 0;
                    }

                }

                else if (laneDirection == 'R') {  // ���[�������@�E
                    iSetAngle = 0; /* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(0, 85);           // �O �i��,�E�j
                    motor_r(0, 85);           // ��i��,�E)

                    if (SENS_LL == OFF) {
                        pattern = 166; /*���f�W�^���Z���TOFF���������̏����� */
                        cnt1 = 0;
                    }
                }
                break;

            case 166:  // �œ��Z���T�@��������̏����i��J�E���^�[�j�@�œ��Z���T�@���������҂�
                if (laneDirection == 'L') {  // ���[�������@��
                    iSetAngle = -LANE_ANGLE_L;      /* +�ō� -�ŉE�ɋȂ���܂� *///�J�E���^�[�Ȃ̂ŋt�ɐU��@
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(90, 80);  // �O �i��,�E�j
                    motor_r(90, 0);   // ��i��,�E)
                    if (isSensrrON==ON && cnt1 >= 10) {
                        pattern = 168;
                    }
                    if (SENS_C == ON && cnt1 >= 10) {
                        pattern = 168;

                    }

                }
                else if (laneDirection == 'R') {   // ���[�������@�E�@�J�E���^�[����
                    iSetAngle = LANE_ANGLE_R;      /* +�ō� -�ŉE�ɋȂ���܂� *///�J�E���^�[�Ȃ̂ŋt�ɐU��@
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(80, 90);  // �O �i��,�E�j
                    motor_r(0, 90);   // ��i��,�E)
                    if (isSensllON==ON && cnt1 >= 10) {
                        pattern = 168;
                    }
                    if (SENS_C == ON && cnt1 >= 10) {
                        pattern = 168;

                    }
                }
               break;
			   
            case 168:  // �Z���^�[�Z���T�@���������҂�
                if (laneDirection == 'L') {  // ���[�������@��
                    iSetAngle = -LANE_ANGLE_L; /* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(90, 80);  // �O �i��,�E�j
                    motor_r(90, 0);   // ��i��,�E)

                }

                else if (laneDirection == 'R') {  // ���[�������@�E�@�J�E���^�[����
                    iSetAngle = LANE_ANGLE_R; /* +�ō� -�ŉE�ɋȂ���܂� */
                    servoPwmOut(iServoPwm2);  // 2�p�x���� 3:�������䖳
                    motor_f(80, 90);  // �O �i��,�E�j
                    motor_r(0, 90);   // ��i��,�E)
                }

                if (SENS_C == ON) {
                    pattern = 170; /*�����f�W�^���Z���T���������̏�����*/
                    cource = 0;  // �R�[�X�O��l0�N���A
                    cnt1 = 0;
                }
                break;

            case 170:
                /* �������Ԃ��o�܂ő҂� */
                i = getServoAngle();  // �X�e�A�����O�p�x�擾
                servoPwmOut(iServoPwm);
                motor_r(90, 90);
                motor_f(90, 90);
                if (abs(i) < 10 &&  cnt1>100) {
                    cnt1 = 0;
                    // led_out(0x0);
                    pattern = 11;
                    crankMode = 0;  // �N�����N���[�h�N���A
                    crankDirection = 0;  // �N�����N���[�h�i�N�����N�����j�N���A
                    laneMode = 0;  // ���[�����[�h�N���A
                    laneDirection = 0;  // ���[�����[�h�i���[�������j�N���A
                    laneClearTime=200;
                    break;
                }
                break;




/************************************************************************/



/* 	�⃂�[�h����	case 191                                        */



/************************************************************************/

			//====================���==========================//

			case 191://�ă`�F�b�N
                servoPwmOut(iServoPwm);
				motor_f(100,100); //�O�i��,�E�j
				motor_r(100,100); //��i��,�E�j

				if( cnt1>=15 ){//15ms
					if(SLOPE_ANGLE > SLOPE_UP_START-5 ){//��邭��
                        pattern=192;
						temp = data_buff[DF_kp];//���Q�C��
						data_buff[DF_kp] = 3;//���Q�C���_�E��
                        lEncoderBuff = lEncoderTotal;//1500ct���x
                        cnt1=0;

					}
					else {//�댟�o��
						pattern=11;
					}
                }
               break;



      		case 192:	//�ԑ̂��������肳����
                servoPwmOut(iServoPwm);
				motor_f(100,100); //�O�i��,�E�j
				motor_r(1,1); //��i��,�E�j

				if( cnt1>=40 ){//40ms
                        pattern=194;
                }
	    		break;


			case 194:	//�����4����3���炢�܂ł�SLOPE_SPEED�ő��s
                servoPwmOut(iServoPwm);
                if (iEncoder >= SLOPE_UP_SPEED+3) {
					motor_f(-10,-10);	//�O (��,�E)
					motor_r(-10,-10);	//�� (��,�E)
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

				if(lEncoderTotal-lEncoderBuff >= 1500){//3/4(1000mm���x)
					pattern=196;
                    lEncoderBuff = lEncoderTotal;//1500ct���x
                    cnt1=0;
				}
			break;

			case 196:
                servoPwmOut(iServoPwm);
                motor_f(1,1); 			//�O�i��,�E�j
				motor_r(-30,-30); 			//��i��,�E�j
				if(lEncoderTotal-lEncoderBuff >= 300){//200mm���x
					pattern=198;
                    lEncoderBuff = lEncoderTotal;//1500ct���x
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
                if (iEncoder >= SLOPE_UP_SPEED-10) {//3.0m/s���x
					motor_f(1,1);	//�O (��,�E)
					motor_r(-30,-30);	//�� (��,�E)
                }
                else {
					motor_f(1,1);	//�O (��,�E)
					motor_r(30,30);	//�� (��,�E)
                }
				if(SLOPE_ANGLE <= SLOPE_UP_FIN+20){//��I��蔻��o��
					pattern=200;

				}
			break;



			case 200:
                servoPwmOut(iServoPwm);
				motor_f(1,1); 			//�O�i��,�E�j
				motor_r(-30,-30); 			//��i��,�E�j
				if(lEncoderTotal-lEncoderBuff >= 300){//200mm���x
					pattern=202;
                    cnt1=0;
				}
                break;


			case 202:
                servoPwmOut(iServoPwm);
    			slopeTotalCount=1;
            	data_buff[DF_kp] = temp;
                slopeFinTime=0;//�≺�育���o�h�~�^�C�}�[
				pattern=11;

                /*while(1){//�m�F�p
					motor_f(0,0);	//�O (��,�E)
					motor_r(0,0);	//�� (��,�E)
                }
                */

			break;

//====================���I��======================//


//====================����==========================//

			case 211://���o�Ċm�F(�s�v�H�Ƃ肠����)
                servoPwmOut(iServoPwm);
				motor_f(70,70); //�O�i��,�E�j
				motor_r(70,70); //��i��,�E�j

				if( cnt1>=15 ){//15ms
					if(SLOPE_ANGLE < SLOPE_DOWN_START+5 ){//���邭��
                        pattern=212;
						temp = data_buff[DF_kp];//���Q�C��
						data_buff[DF_kp] = 3;//���Q�C���_�E��
                        lEncoderBuff = lEncoderTotal;
                        cnt1=0;
					}
					else {//�댟�o��
						pattern=11;
					}
                }
               break;


			case 212://�Z���T�[�o�[�𗎂Ƃ����߂̃u���[�L
                servoPwmOut(iServoPwm);
				motor_f(0,0); 			//�O�i��,�E�j
				motor_r(-60,-60); 		//��i��,�E�j
				if(lEncoderTotal-lEncoderBuff >= 300){//200mm���x
                    lEncoderBuff = lEncoderTotal;
					pattern=214;
				}
			break;

			case 214://���葖�s��ڎw���O�ւő��s
                servoPwmOut(iServoPwm);
            	motor_f(70,70); 			//�O�i��,�E�j
				motor_r(30,30); 			//��i��,�E�j
				if(lEncoderTotal-lEncoderBuff >= 450){//300mm���x
                    lEncoderBuff = lEncoderTotal;
					pattern=216;
				}
			break;

			case 216:
                servoPwmOut(iServoPwm);
                if (iEncoder >= SLOPE_DOWN_SPEED+3) {
					motor_f(-10,-10);	//�O (��,�E)
					motor_r(-10,-10);	//�� (��,�E)
                }
                else if (iEncoder >= SLOPE_DOWN_SPEED) {
                    motor_f(1, 1);
                    motor_r(1, 1);
                } else {
                    motor_f(80, 80);
                    motor_r(80, 80);
                }

				if(SLOPE_ANGLE > SLOPE_DOWN_FIN -5 ){//����I��茟�o
                    lEncoderBuff = lEncoderTotal;
					pattern=218;
				}
			break;


			case 218://�ԑ̈���̂��߂̃u���[�L�i�Z���T�[�o�[�����肳����j
                servoPwmOut(iServoPwm);
				motor_f(-30,-30); 			//�O�i��,�E�j
				motor_r(-30,-30); 			//��i��,�E�j
				if(lEncoderTotal-lEncoderBuff >= 151){//50mm
                    lEncoderBuff = lEncoderTotal;
					pattern=220;
				}
			break;

			case 220://�ԑ̈���̂��߂̃t���[
                servoPwmOut(iServoPwm);
				motor_f(1,1); 			//�O�i��,�E�j
				motor_r(1,1); 			//��i��,�E�j
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
					motor_f(0,0);	//�O (��,�E)
					motor_r(0,0);	//�� (��,�E)
                }
                */
			break;


			//====================����I��======================//

            case 231:
                /* ��~���� */
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
                /* �Ō�̃f�[�^���������܂��܂ő҂� */
                //                if (microSDProcessEnd() == 0) {
                //                    pattern = 235;
                //                }
                //pattern = 235;

                break;

            case 235:
                /* �������Ȃ� */
                break;

            case 241:
                /* ��~ */
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
                /* �v�b�V���X�C�b�`�������ꂽ���`�F�b�N */
                if (RUN_SW == OFF) {
                    pattern = 243;
                    cnt1 = 0;
                }
                break;

            case 243:
                /* 0.5s�҂� */
                if (cnt1 >= 500) {
                    pattern = 245;
                    cnt1 = 0;
                }
                break;
            case 245:
                /* �^�C�g���]���A�]������ */
                printf("\n");
                printf("Run Data Out\n");
                pattern = 246;
                break;

            case 246:
                /* �f�[�^�]�� */
                /* �I���̃`�F�b�N */
#if 0
            if (((char)readI2CEeprom(saveSendIndex) == 203) || (saveSendIndex >= 0x8000 /*0x1ffff*/)) {
                pattern = 247;
                printf("fin\n");
                // setBeepPatternS(0xff00);
                cnt1 = 0;
                break;
            }

                /* ��f�[�^�̃`�F�b�N */
                if (readI2CEeprom(saveSendIndex) == 0) {
                    data_lost++;
                    saveSendIndex += 16;
                    break;
                }
#endif

                /* �f�[�^�̓]�� */
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
                    /* 05 R�Z���T �A�i���O 	*/
                    (char)readI2CEeprom(saveSendIndex + 5),
                    /* 06 L�Z���T �A�i���O 	*/
                    (char)readI2CEeprom(saveSendIndex + 6),
                    /* 07 �p�^�[�� 	*/
                    (char)readI2CEeprom(saveSendIndex + 7),
                    /* 08 saka */
                    (char)readI2CEeprom(saveSendIndex + 8),
                    /* 09 �X�e�A�����O	*/
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
                /* �]���I�� */

                break;

            default:
                break;
        }
    }
}
void init(void) {
    int i;

    /* �N���b�N��XIN�N���b�N(20MHz)�ɕύX */
    prc0 = 1; /* �v���e�N�g����               */
    cm13 = 1; /* P4_6,P4_7��XIN-XOUT�[�q�ɂ���*/
    cm05 = 0; /* XIN�N���b�N���U              */
    for (i = 0; i < 50; i++)
        ;     /* ���肷��܂ŏ����҂�(��10ms) */
    ocd2 = 0; /* �V�X�e���N���b�N��XIN�ɂ���  */
    prc0 = 0; /* �v���e�N�gON                 */
    prc2 = 1; /* PD0�̃v���e�N�g����          */

    /* �|�[�g�̓��o�͐ݒ� */

    /*  PWM(�\��)       ���OM_PMW       �E�OM_PWM       �u�U�[
        �Z���T���[      �Z���T����      �Z���T�E��      �Z���T�E�[  */
    // p0  = 0x00;
    pd0 = 0x7f; /* LCD�̐���Ɏg�p  */

    /*  none			none			RxD0            TxD0
        DIPSW3          DIPSW2          DIPSW1          DIPSW0          */
    pur0 = 0x04; /* P1_3�`P1_0(�����r�v)�̃v���A�b�vON     */
    p1 = 0x00;
    pd1 = 0xd0;

    /*  none			none �E�OM_PWM
       �X�e�AM_PWM none			����M_PWM			none
       none      */
    p2 = 0x00;
    pd2 = 0xff;

    /*  none            none            none            none
        none            none            none            �G���R�[�_A��   */
    p3 = 0x00;
    pd3 = 0xf0;

    /*  XOUT            XIN             �{�[�h���LED   none
        none            VREF            none            none            */
    p4 = 0x20; /* P4_5��LED:�����͓_��         */
    pd4 = 0xb8;

    /*  none		none		none
       �X�e�AM_����
        none		none		�X�e�AM_���� �E�OM_����		*/
    p5 = 0x00;
    pd5 = 0xff;

    /*  none		�E��M_����		�E��M_����		none
        none		none			none RUN_SW            */
    // p6  = 0x00;
    pd6 = 0xe0;

    /*  none				none				none
       none �A�i���O��(IN)		�A�i���O�E(IN)		��Z���T�[(IN)
       �p�xVR(IN)  */
    // p7  = 0x00;
    pd7 = 0x00;

    /*  none			none			none
       none ����M_���� 		����M_����      ���OM_����      ���OM_���� */
    p8 = 0xff;
    pd8 = 0xff;
    //    pd8 = 0x0f;

    /*  -               -               L_LED				R_LED
        none		D�Z���T�E(IN)		D�Z���T��(IN)
       D�Z���T��(IN)		*/
    // p9  = 0x00;
    pd9 = 0x10; /* 0-3:�f�W�^���Z���T�e��   4:�m�F�pLED  */
    pu23 = 1;   // P9_4,P9_5���v���A�b�v����

    // trbrcsr = 0x00;

    /* �^�C�}RB�̐ݒ� */
    /* ���荞�ݎ��� = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                    = 1 / (20*10^6) * 200        * 100
                    = 0.001[s] = 1[ms]
    */
    //    trbmr = 0x10;     /* ���샂�[�h�A������ݒ�       */
    //    trbpre = 200 - 1; /* �v���X�P�[�����W�X�^         */
    //    trbpr = 100 - 1;  /* �v���C�}�����W�X�^           */
    //    trbic = 0x06;     /* ���荞�ݗD�惌�x���ݒ�       */
    //    trbcr = 0x01;     /* �J�E���g�J�n                 */
    // ���荞�ݎ���߁iTimerRC�ֈڍs�j

    /* A/D�R���o�[�^�̐ݒ� */
    admod = 0x33;   /* �J��Ԃ��|�����[�h�ɐݒ�     */
    adinsel = 0xb0; /* ���͒[�qP7��4�[�q��I��      */
    adcon1 = 0x30;  /* A/D����\                  */
    asm(" nop ");   /* ��AD��1�T�C�N���E�G�C�g�����*/
    adcon0 = 0x01;  /* A/D�ϊ��X�^�[�g              */

    /* �^�C�}RG �^�C�}���[�h(���G�b�W�ŃJ�E���g)�̐ݒ� */
    timsr = 0x40; /* TRGCLKA�[�q P3_0�Ɋ��蓖�Ă� */
    // trgcr = 0x15; /* TRGCLKA�[�q�̗��G�b�W�ŃJ�E���g 0001 0101*/
    trgcr = 0x05; /* TRGCLKA�[�q�̗����オ��G�b�W�ŃJ�E���g 0000 0101*/

    trgmr = 0x80; /* TRG�̃J�E���g�J�n            */

    /* �^�C�}RD PWM���[�h�ݒ�(���O���[�^�A�E�O���[�^�A�X�e�A���[�^) */
    /* PWM���� = 1 / 20[MHz]   * �J�E���g�\�[�X * (TRDGRA0+1)
               = 1 / (20*10^6) * 1 =50ns    (*32 = 1600ns)
                           = 1.616 us          * 101
               = 0.1616[ms]     =��6.18k[Hz]
    */
    trdpsr0 = 0x08;              /* TRDIOB0,C0,D0�[�q�ݒ�        */
    trdpsr1 = 0x05;              /* TRDIOA1,B1,C1,D1�[�q�ݒ�     */
    trdmr = 0xf0;                /* �o�b�t�@���W�X�^�ݒ�         */
    trdfcr = 0x01;               /* ���Z�b�g����PWM���[�h�ɐݒ�  */
    trdcr0 = 0x24;               /* �\�[�X�J�E���g�̑I��:f1      */
    //trdgra0 = trdgrc0 = 101 - 1; /* �����ݒ� */ 
	trdgra0 =trdgrc0=601-1;
    trdgrb0 = trdgrd0 = 0;       /* P2_2�[�q��ON��(���O���[�^)   */
    trdgra1 = trdgrc1 = 0;       /* P2_4�[�q��ON��(�X�e�A���[�^)   */
    trdgrb1 = trdgrd1 = 0;       /* P2_5�[�q��ON��(�E�O���[�^) */
    trdoer1 = 0xcd;              /* �o�͒[�q�̑I��               */
    trdstr = 0x0d;               /* TRD0�J�E���g�J�n             */

    /* �^�C�}RC ���Z�b�g����PWM���[�h�ݒ�(����Ӱ��A�E��Ӱ�) */
    /* PWM���� = 1 / 20[MHz]   * �J�E���g�\�[�X * (TRDGRA0+1)*/

    trcmr = 0x0b; /* 3:"1"  2:D  1:C  0:B  PWM���[�h�I���r�b�g�ݒ�  1011*/
    trccr1 =
        0x8e; /* 7:"1"  6�`4:000 ���������:f1, �����o�͂̐ݒ�(0:�m�n�A�N�e�B�u
        1:�A�N�e�B�u) 3:D 2:C 1:B 0:"0" 0110*/
    trccr2 = 0x00; /* 7.6:"00"  5�`3:"0 00"  �o�̓��x���̐ݒ�(0:L�A�N�e�B�u
                      1:H�A�N�e�B�u) 2:D 1:C 0:B*/
    /*�@�A�N�e�B�u���x���@TRCGRA �̃R���y�A�}�b�`��̐M��  */
    trcoer = 0x06; /* 7�`4:"0000" �o�͒[�q�̑I�� (0:���@1:�s����) 3:D 2:C 1:B
                      0:A   C�́A�\���Ȃ̂Ŏg�p���Ȃ��iI/O�Ƃ��ė��p�j*/
    trcpsr0 = 0x70; /* p.323 TRCIOA,B�[�q�̐ݒ� p5_2(RL_PWM:TRCIOB) 7:"0"
                       6�`4:111 3:"0" 2�`0:"000"*/
    trcpsr1 = 0x06; /* p.324 TRCIOC,D�[�q�̐ݒ� p5_3(RR_PWM:TRCIOC) 7:"0"
                       6�`4:"000" 3:"0" 2�`0:110*/
    /* p5_4(        :TRCIOD)�͗\���Ƃ���  */

    trcgra = 20000 - 1; /* �����ݒ� 1ms */
    trcgrb = trcgra;    /* P5_2�[�q��ON��(RL���[�^�[) */
    trcgrc = trcgra;    /* P5_3�[�q��ON��(RR���[�^�[) */

    // trcgrd = 0; /* ON��(�\��) ���g�pI/O�Ƃ��ė��p */

    /*�o�b�t�@������s�����߁A���荞�݂�ݒ� */
    trcic = 0x07;  /* ���荞�ݗD�惌�x���ݒ� */
    trcier = 0x01; /* IMIA������ */
    trcoer = 0x01; /* �o�͒[�q�̑I�� */

    trcmr |= 0x80; /* TRC�J�E���g�J�n */
}

/************************************************************************/
/* �^�C�}RB ���荞�ݏ���                                                */
/************************************************************************/
// �폜�i�^�C�}RC�ֈڍs�j

/************************************************************************/
/* �^�C�}RC ���荞�ݏ���                                                */
/************************************************************************/
#pragma interrupt intTRC(vect = 7)
void intTRC(void) {
    static int line_no; /* �s�ԍ�                       */
    signed long i;

    trcsr &= 0xfe;

    cnt1++;
    cnt2++;//LED����p
    slopeFinTime++;//��댟�o�h�~�^�C�}�[

    mtPower++;//�R�[�i�����O��PWM�����X�ɃA�b�v�p

    if(laneClearTime>0){
        laneClearTime--;
    }

    if(crankClearTime>0){
        crankClearTime--;
    }

    /* �^�C�}RC�@�f���[�e�B��̐ݒ� */
    trcgrb = trcgrb_buff;
    trcgrc = trcgrc_buff;

    /* �T�[�{���[�^���� */
    servoControl();
    servoControl2();

    if (pattern >= 11 && pattern <= 230) {
        /* �����ɂ���~���� */
        // ���[�^SW(ON)�F0�Ȃ瑖�s
        if (lEncoderTotal >= 1515L * data_buff[DF_STOP] && MTCT_SW == 0) {
            pattern = 231;
        }

        /* �E�֎��̒�~�����i�f�W�^���Z���T�j */
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

        /* �E�֎��̒�~�����i���[�^���G���R�[�_�j */
        // ���[�^SW(ON)�F0�Ȃ瑖�s
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
		
    /* �g���X�C�b�`�p�֐�(1ms���ƂɎ��s)    */
    // switchProcess();

    /* LCD�\�������p�֐�(1ms���ƂɎ��s)     */
    // lcdShowProcess();

    /* �u�U�[���� */
    // beepProcessS();

    /* microSD�Ԍ��������ݏ���(1ms���ƂɎ��s)   */
    // microSDProcess();

    /* 10��1����s���鏈�� */
    iTimer10++;
    switch (iTimer10) {
        case 1:
            /* �G���R�[�_���� */
            i = trg;
            iEncoder = i - uEncoderBuff;
            lEncoderTotal += iEncoder;
            uEncoderBuff = i;
            break;

        case 2:
            i = BAR_ANGLE;             // ���݃o�[�A���O���擾
            iAngle2 = i - iAngleBuff;  // �X�e�A�����O���x�����߂�
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
            /* �f�[�^�ۑ��֘A */
            if (saveFlag) {
                saveData[0] = SENS_LL << 2 | SENS_C << 1 | SENS_RR;  // SENS_ALL; /* �f�W�^���Z���T */
                saveData[1] = iEncoder; /* �G���R�[�_ */
                saveData[2] = crank_count;       //
                saveData[3] = getServoAngle(); /* �T�[�{�p�x */
                saveData[4] = iAngle2;         /* �T�[�{�p���x */
                saveData[5] = ANA_SENS_R >> 2; /* �A�i���O�Z���T�� */
                saveData[6] = ANA_SENS_L >> 2; /* �A�i���O�Z���T�E */
                saveData[7] = pattern;         /* �p�^�[��       */
                saveData[8] = SLOPE_ANGLE;     /* �⓹�Z���TAD           */
                // saveData[9] :PWM�X�e�A�����O;
                // saveData[10] :PWM�O�E;
                // saveData[11] :PWM�O��;
                // saveData[12] :PWM��E;
                // saveData[13] :PWM�㍶;
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
            /* iTimer10�ϐ��̏��� */
            iTimer10 = 0;
            break;
    }
}

/************************************************************************/
/* �^�C�}�{��                                                           */
/* �����@ �^�C�}�l 1=1ms                                                */
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
/* ��Βl�ϊ� */
/* �����@ �ϊ��l */
/************************************************************************/
unsigned int abs(int i) {
    if (i < 0) {
        return i * -1;
    } else {
        return i;
    }
}

// �g�p���Ȃ�
/************************************************************************/
/* �A�i���O�Z���T���TypeS Ver.2�̃f�W�^���Z���T�l�ǂݍ���              */
/* �����@ �Ȃ�                                                          */
/* �߂�l ���[�A�����A�E���A�E�[�̃f�W�^���Z���T 0:�� 1:��              */
/************************************************************************/
unsigned char sensor_inp(void) {
    unsigned char sensor;

    sensor = ~p0 & 0x0f;

    return sensor;
}

// �g�p���Ȃ�
/************************************************************************/
/* �A�i���O�Z���T���TypeS Ver.2�̒��S�f�W�^���Z���T�ǂݍ���            */
/* �����@ �Ȃ�                                                          */
/* �߂�l ���S�f�W�^���Z���T 0:�� 1:��                                  */
/************************************************************************/
unsigned char center_inp(void) {
    unsigned char sensor;

    sensor = ~p1_7 & 0x01;

    return sensor;
}

// �g�p���Ȃ�
/************************************************************************/
/* �A�i���O�Z���T���TypeS Ver.2�̂��ׂẴf�W�^���Z���T�ǂݍ���        */
/* �����@ �Ȃ�                                                          */
/* �߂�l �T�̃f�W�^���Z���T 0:�� 1:��                                */
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
/* �A�i���O�Z���T���TypeS Ver.2�̃X�^�[�g�o�[���o�Z���T�ǂݍ���        */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0:�X�^�[�g�o�[�Ȃ� 1:�X�^�[�g�o�[����                         */
/************************************************************************/
// �g�p���Ȃ�
unsigned char startbar_get(void) {
    unsigned char sensor;

    sensor = ~p1_6 & 0x01;

    return sensor;
}

/************************************************************************/
/* �}�C�R���{�[�h��̃f�B�b�v�X�C�b�`�l�ǂݍ���                         */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0�`15                                              */
/************************************************************************/
unsigned char dipsw_get(void) {
    unsigned char sw;

    sw = p1 & 0x0f; /* P1_3�`P1_0�ǂݍ���           */

    return sw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��̃f�B�b�v�X�C�b�`�l�ǂݍ���          */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0�`255                                             */
/************************************************************************/
unsigned char dipsw_get2(void) {
    /* ���ۂ̓��͂̓^�C�}RB���荞�ݏ����Ŏ��{ */
    return types_dipsw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��̃v�b�V���X�C�b�`�l�ǂݍ���          */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0:OFF 1:ON                                         */
/************************************************************************/
// �g�p���Ȃ�
unsigned char pushsw_get(void) {
    unsigned char sw;

    sw = ~p9_5 & 0x01;

    return sw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��CN6�̏�ԓǂݍ���                     */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0�`15                                                         */
/************************************************************************/
unsigned char cn6_get(void) {
    unsigned char data;

    data = p7 >> 4;

    return data;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��LED����                               */
/* �����@ 8��LED���� 0:OFF 1:ON                                       */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void led_out(unsigned char led) {
    /* ���ۂ̏o�͂̓^�C�}RB���荞�ݏ����Ŏ��{ */
    types_led = led;
}

/************************************************************************/
/* ��ւ̑��x����                                                       */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_r(int accele_l, int accele_r) {
    int sw_data;

    // PWM�@DOWN���s��Ȃ�
    //    sw_data = dipsw_get() + 5; /* �f�B�b�v�X�C�b�`�ǂݍ���     */
    //    accele_l = accele_l * sw_data / 20;
    //    accele_r = accele_r * sw_data / 20;

    // ���[�^SW�FON�iOFF�Ȃ��~�j
    if (MTCT_SW == 0) {
        motor2_r(accele_l, accele_r);
    }
}

/************************************************************************/
/* ��ւ̑��x����2 �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ�motor�֐�              */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor2_r(int accele_l, int accele_r) {
    /* �㍶���[�^ */
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
    /* ��E���[�^ */
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
/* �O�ւ̑��x����                                                       */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_f(int accele_l, int accele_r) {
    int sw_data;

    // PWM�@DOWN���s��Ȃ�
    //     sw_data = dipsw_get() + 5; /* �f�B�b�v�X�C�b�`�ǂݍ���     */
    //     accele_l = accele_l * sw_data / 20;
    //    accele_r = accele_r * sw_data / 20;

    // ���[�^SW�FON�iOFF�Ȃ��~�j
    if (MTCT_SW == 0) {
        motor2_f(accele_l, accele_r);
    }
}

/************************************************************************/
/* �O�ւ̑��x����2 �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ�motor�֐�              */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor2_f(int accele_l, int accele_r) {
    /* ���O���[�^ */
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

    /* �E�O���[�^ */
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
// �g�p���Ȃ�
/************************************************************************/
/* �ヂ�[�^��~����i�u���[�L�A�t���[�j                                 */
/* �����@ �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE               */
/* �߂�l �Ȃ�                                                          */
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
// �g�p���Ȃ�
/************************************************************************/
/* �O���[�^��~����i�u���[�L�A�t���[�j                                 */
/* �����@ �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE               */
/* �߂�l �Ȃ�                                                          */
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
/* �T�[�{���[�^����                                                     */
/* �����@ �T�[�{���[�^PWM�F-100�`100                                    */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void servoPwmOut(int pwm) {
    /* �{�����[���l�ɂ�荶���~�b�g���� */
//    if (getServoAngle() >= 100 && pattern >= 11) {
//        if (pwm < -10) pwm = 0;
//    }
    /* �{�����[���l�ɂ��E���~�b�g���� */
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
/* �N���X���C�����o����                                                 */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0:�N���X���C���Ȃ� 1:����                                     */
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
/* �E�n�[�t���C�����o����                                               */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0:�E�n�[�t���C���Ȃ� 1:����                                   */
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
/* ���n�[�t���C�����o����                                               */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0:���n�[�t���C���Ȃ� 1:����                                   */
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
/* �T�[�{�p�x�擾                                                       */
/* �����@ �Ȃ�                                                          */
/* �߂�l �T�[�{�p�x                                                */
/************************************************************************/
int getServoAngle(void) {
    /* ���o�ړ����ώZ�o�p�ϐ�*/
    static signed int angleSum=0;			/* �ړ����ϒl���Z�p�ϐ� */
    static signed int angleBuf[8];			/* �ړ����ϒl���Z�p�ϐ� */
    static signed int angleCount=0;			/* �ړ����ϒl���Z�p�ϐ� */
    static signed int retAngle;			/* �ړ����ϒl���Z�p�ϐ� */

	/* 8�_�̑��x�̈ړ����όv�Z angleBuf[8]*/
	angleSum =(angleSum + iAngle0) -angleBuf[angleCount];//���v�ɍł��Â��f�[�^�̒l���Z�@�ŐV�f�[�^�̒l���Z
	angleBuf[angleCount]=iAngle0; //�ŐV�̃f�[�^���� 
	angleCount++;//�C���f�b�N�X�̃C���N�������g
	angleCount = angleCount & 0x07;//0��1�@�E�E�E�@7��0��1�E�E
	retAngle = angleSum >> 3 ;// /8

    return (BAR_ANGLE - retAngle);
}

// �g�p���Ȃ�
/************************************************************************/
/* �A�i���O�Z���T�l�擾 (�I���W�i���v���O����)                          */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Z���T�l                                                      */
/************************************************************************/
int getAnalogSensor_org(void) {
    int ret;

    //   ret = ad1 - ad0; /* �A�i���O�Z���T���擾       */
    ret = ANA_SENS_R - ANA_SENS_L; /* �A�i���O�Z���T���擾       */

    if (!crankMode) {
        /* �N�����N���[�h�łȂ���Ε␳���� */
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
                /* �Z���T�E��� */
                ret = -700;
                if (sensor_inp() == 0x04) {
                    iSensorPattern = 0;
                }
                break;

            case 2:
                /* �Z���T����� */
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
/* �A�i���O�Z���T�l�擾 (���J�H��  �f�W�^���Z���T3�p�g�p)                */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Z���T�l                                                      */
/************************************************************************/
int getAnalogSensor(void) {
    int ret;

    //   ret = ad1 - ad0; /* �A�i���O�Z���T���擾       */
    ret = (ANA_SENS_L>>2) - (ANA_SENS_R>>2); /* �A�i���O�Z���T���擾    ����F�{ �@�E��F-�@  */
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
        /* �N�����N���[�h�łȂ���Ε␳���� */
        // courceOut():0�̂Ƃ��́A�A�i���O�Z���T�ɂ��g���[�X
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
/* �R�[�X�O��l�擾�֐� */
/* �����@ ���� */
/* �߂�l ���� */
/*�@���ӁF�@�ϐ�cource���O���[�o���ϐ�
/************************************************************************/
int courceOut(void) {
    if (SENS_C == ON) { /* C�Z���T���� */
        switch (cource) {
            case 1:
            case 2:
            case -1:
            case -2:
                cource = 0; /* �R�[�X�� */
                break;
        }
    } else if (ANA_SENS_L > THRESHOLD_H) { /* L�Z���T���� */
        switch (cource) {
            case 0:
            case -2:
                cource = -1; /* �R�[�X�O��R1 */
                break;
        }
    } else if (ANA_SENS_R > THRESHOLD_H) { /* R�Z���T���� */
        switch (cource) {
            case 0:
            case 2:
                cource = 1; /* �R�[�X�O��L1 */
                break;
        }
    } else if (isSensllON==ON) { /* LL�Z���T���� */
        switch (cource) {
            case -1:
            case -3:
                cource = -2; /* �R�[�X�O��R2 */
                break;
        }
    } else if (isSensllON==ON) { /* RR�Z���T���� */
        switch (cource) {
            case 1:
            case 3:
                cource = 2; /* �R�[�X�O��L2 */
                break;
        }
    }

    else if (SENS_RR == OFF && SENS_C == OFF && SENS_LL == OFF &&
             ANA_SENS_R < THRESHOLD_L &&
             ANA_SENS_L < THRESHOLD_L) { /* �S�ẴZ���T�� */
        switch (cource) {
            case 2:
                cource = 3; /* �R�[�X�O��L3 */
                break;
            case -2:
                cource = -3; /* �R�[�X�O��R3 */
                break;
        }
    }
    return cource;
}

/************************************************************************/
/* �T�[�{���[�^����                                                     */
/* �����@ �Ȃ�                                                          */
/* �߂�l �O���[�o���ϐ� iServoPwm �ɑ��                               */
/************************************************************************/
void servoControl(void) {
    int i, iRet, iP, iD;
    int kp, kd;

    i = getAnalogSensor(); /* �Z���T�l�擾                 */
    kp = data_buff[DF_kp];
    kd = data_buff[DF_kd];

    /* �T�[�{���[�^�pPWM�l�v�Z */
    iP = kp * i;                   /* ���                         */
    iD = kd * (iSensorBefore - i); /* ����(�ڈ���P��5�`10�{)       */
    iRet = iP - iD;

//    iRet /= 64;//  <<1 :/2  <<2 :/4  <<3 :/8  <<4 :/16   <<5 :/32  <<6 :/64
    iRet /= 16;//  <<1 :/2  <<2 :/4  <<3 :/8  <<4 :/16   <<5 :/32  <<6 :/64


    /* PWM�̏���̐ݒ� */
//    if (iRet > 70) iRet = 70;   /* �}�C�R���J�[�����肵����     */
//    if (iRet < -70) iRet = -70; /* �����70���炢�ɂ��Ă������� */

    if (iRet > 100) iRet = 100;   /* �}�C�R���J�[�����肵����     */
    if (iRet < -100) iRet = -100; /* �����70���炢�ɂ��Ă������� */

    iServoPwm = -iRet;
    iSensorBefore = i; /* ����͂��̒l��1ms�O�̒l�ƂȂ�*/
}

/************************************************************************/
/* �T�[�{���[�^2����                                                     */
/* �����@ �Ȃ�                                                          */
/* �߂�l �O���[�o���ϐ� iServoPwm �ɑ��                               */
/************************************************************************/
void servoControl2(void) {

    signed int i, j, iRet, iP, iD;
    signed int kp, kd;

    i = iSetAngle;
    j = getServoAngle();

    /* �T�[�{���[�^�pPWM�l�v�Z */
    iP = 20 * (j - i);              /* ���                         */
    iD = 100 * (iAngleBefore2 - j); /* ����(�ڈ���P��5�`10�{)       */
    iRet = iP - iD;
    iRet /= 2;

    /* PWM�̏���̐ݒ� */
    if (iRet > 100) iRet = 100;   /* �}�C�R���J�[�����肵����     */
    if (iRet < -100) iRet = -100; /* �����70���炢�ɂ��Ă������� */
    iServoPwm2 = iRet;

    iAngleBefore2 = j; /* ����͂��̒l��1ms�O�̒l�ƂȂ�*/
}

/************************************************************************/
/* �O�ւ�PWM����A���ւ�PWM������o���@�n���h���p�x�͌��݂̒l���g�p     */
/* �����@ �O��PWM                                                       */
/* �߂�l ����PWM                                                       */
/************************************************************************/
int diff(int pwm) {
    int i, ret;

    i = getServoAngle()* VR_DEG_CHANGE; /* 1�x������̑����Ŋ���        */
    if (i < 0) i = -i;
    if (i > 45) i = 45;
    ret = revolution_difference[i] * pwm / 100;

    return ret;
}

/* �X�e�A�����O�p 0:���� +:�� -:�E 1.6=1�x */
/************************************************************************/
/* �O�ւ�PWM����A���ւ�PWM������o���@�n���h���p�x�͌��݂̒l���g�p     */
/* �����@ �O��PWM                                                       */
/* �߂�l ����PWM                                                       */
/************************************************************************/
int diff_fi(int pwm) {
    int i, ret;

    i = getServoAngle() * VR_DEG_CHANGE; /* 1�x������̑����Ŋ���        */
    if (i < 0) i = -i;
    if (i > 45) i = 45;
    ret = revolution_difference_fi[i] * pwm / 100;

    return ret;
}

/************************************************************************/
/* �O�ւ�PWM����A���ւ�PWM������o���@�n���h���p�x�͌��݂̒l���g�p     */
/* �����@ �O��PWM                                                       */
/* �߂�l ����PWM                                                       */
/************************************************************************/
int diff_ri(int pwm) {
    int i, ret;

    i = getServoAngle() * VR_DEG_CHANGE; /* 1�x������̑����Ŋ���        */
    if (i < 0) i = -i;
    if (i > 45) i = 45;
    ret = revolution_difference_ri[i] * pwm / 100;

    return ret;
}

/************************************************************************/
/* �O�ւ�PWM����A���ւ�PWM������o���@�n���h���p�x�͌��݂̒l���g�p     */
/* �����@ �O��PWM                                                       */
/* �߂�l ����PWM                                                       */
/************************************************************************/
int diff_ro(int pwm) {
    int i, ret;

    i = getServoAngle() *VR_DEG_CHANGE; /* 1�x������̑����Ŋ���        */
    if (i < 0) i = -i;
    if (i > 45) i = 45;
    ret = revolution_difference_ro[i] * pwm / 100;

    return ret;
}

/************************************************************************/
/* DataFlash�̃p�����[�^�ǂݍ���                                        */
/* ����         �Ȃ�                                                    */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
void readDataFlashParameter(void) {
    int i;
    unsigned int st = DF_ADDR_END + 1 - DF_PARA_SIZE;
    signed char c;

    while (1) {
        // �ǂݍ��ޔԒn��T��
        readDataFlash(st, &c, 1);
        if (c == 0x11) {
            readDataFlash(st, data_buff, DF_PARA_SIZE);
            break;
        }

        st -= DF_PARA_SIZE;

        if (st < DF_ADDR_START) {
            // �Y�������@���߂Ďg�p
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
/* DataFlash�փp�����[�^��������                                        */
/* ����         �Ȃ�                                                    */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
void writeDataFlashParameter(void) {
    unsigned int st = DF_ADDR_START;
    signed char c;

    while (1) {
        // �������ޔԒn��T��
        readDataFlash(st, &c, 1);
        if (c == -1) {
            writeDataFlash(st, data_buff, DF_PARA_SIZE);
            break;
        }

        st += DF_PARA_SIZE;

        if (st > DF_ADDR_END) {
            // ���ׂĎg�p������A�C���[�Y���Đ擪�ɏ�������
            blockEraseDataFlash(DF_ADDR_START);
            writeDataFlash(DF_ADDR_START, data_buff, DF_PARA_SIZE);
            break;
        }
    }
}

/************************************************************************/
/* LCD�ƃX�C�b�`���g�����p�����[�^�Z�b�g����                            */
/* ����         �Ȃ�                                                    */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
int lcdProcess(void) {
    int i;
    char sw = 0;  // LCD�X�C�b�`���p

    // printf("lcd_pattern=%d",lcd_pattern );
    // printf(" pattern=%d\n",pattern );

#if 0
    if (pattern != 0) {
        if (cnt_lcd >= 250) {
            cnt_lcd = 0;
            lcdPosition(0, 0);
            /* 0123456789abcbef 1�s16���� */
            lcdPrintf("pattern = %3d   ", pattern);
            /* 01234567..89abcde.f 1�s16���� */
            lcdPrintf("sensor=%02x bar=%d ", sensor_inp(), startbar_get());
        }
        return;
    }
#endif

    // �X�C�b�`���擾
    sw = getSwNow();
    //    printf("    sw=%d", sw);

    // ���j���[�{�P
    if (sw == MENU_UP) {
        lcd_pattern++;
        wait_ms(200);

        if (lcd_pattern == 11) lcd_pattern = 1;
    }

    // ���j���[�|�P
    if (sw == MENU_DOWN) {
        lcd_pattern--;
        wait_ms(200);

        if (lcd_pattern == 0) lcd_pattern = 10;
    }

    /* LCD�A�X�C�b�`���� */
    switch (lcd_pattern) {
        case 1:
            /* ���s��~�������� */
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

            /* LCD���� */
            lcdPosition(0, 0);
            lcdPrintf("01 Stop L[m]=%03d", i);
            lcdPosition(0, 1);
            lcdPrintf("Encoder = %03d   ", lEncoderTotal);
            break;

        case 2:
            /* �X�^�[�g�҂����Ԓ��� */
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

            /* LCD���� */
            lcdPosition(0, 0);
            lcdPrintf("02 St time = %03d", i);
            lcdPosition(0, 1);
            lcdPrintf("L=%1d C=%1d R=%1d %1d   ", SENS_LL, SENS_C, SENS_RR,
                      SENS_ALL);
            cnt1 = 0;
            break;

        case 3:
            /* �g���[�X��ᐧ�䒲�� */
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

            /* LCD���� */
            lcdPosition(0, 0);
            lcdPrintf("03 Trace kp =%03d", i);
            lcdPosition(0, 1);
            lcdPrintf("L=%4d  R=%4d   ", ANA_SENS_L, ANA_SENS_R);
            // lcdPrintf("cnt1=%4d" , cnt1);

            if (RUN_SW == 1) {
                //  �p�����[�^�ۑ�
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
            /* �g���[�X�������䒲�� */
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

            /* LCD���� */
            lcdPosition(0, 0);
            lcdPrintf("04 Trace kd =%03d", i);
            // lcdPosition(0, 1);
            // lcdPrintf("AS()=%4d  %4d  ",getAnalogSensor(),iServoPwm);
            lcdPosition(0, 1);
            lcdPrintf("courceOut=%2d       ", courceOut());
            break;

        case 5:
            /* �������s�ڕW���x�ݒ� */
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

            /* LCD���� */
            lcdPosition(0, 0);
            lcdPrintf("05 Speed_S = %3d", i);
            // lcdPosition(0, 1);
            // lcdPrintf("courceOut=%2d        ",courceOut());

            lcdPosition(0, 1);
            lcdPrintf("Bar Angle = %4d", BAR_ANGLE);

            break;

        case 6:
            /* �J�[�u���s�ڕW���x�ݒ� */
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

            /* LCD���� */
            lcdPosition(0, 0);
 //           lcdPrintf("06 Speed_C = %3d", i);
            lcdPrintf("06                    ");

            lcdPosition(0, 1);
            lcdPrintf("Slope Angle=%4d", SLOPE_ANGLE);
            break;

        case 7:
            /* �N�����N�i���ڕW���x�ݒ� */
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

            /* LCD���� */
            lcdPosition(0, 0);
            lcdPrintf("07 Speed_CL =%3d", i);
            lcdPosition(0, 1);
            lcdPrintf("");
            break;

        case 8:
            /* ���[���`�F���W�i���ڕW���x�ݒ� */
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

            /* LCD���� */
            lcdPosition(0, 0);
            lcdPrintf("08 Speed_RC =%3d", i);
            lcdPosition(0, 1);
            lcdPrintf("");
            break;

        case 9:
            /* �ݒ�p�����[�^�[�ۑ� */
            servoPwmOut(0);

            lcdPosition(0, 0);
            lcdPrintf("09 Parameter Set");

            // �ݒ�l�ۑ�
            if (RUN_SW == 0) {
                cnt1 = 0;
                do {
                    // �X�C�b�`���擾
                    sw = getSwNow();
                    wait_ms(200);
                    if (cnt1 > 2000) {
                        // �p�����[�^�ۑ�
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
            /* ���[�^�[�e�X�g�h���C�o��m�F */

            /* LCD���� */
            lcdPosition(0, 0);
            lcdPrintf("10 Motor_Test   ");
            lcdPosition(0, 1);
            lcdPrintf("SW_1/2 ON!      ");

            if (RUN_SW == 1 && sw == DATA_UP ||RUN_SW == 1 && DATA_DOWN) {
                do {
                    // �X�C�b�`���擾
                    sw = getSwNow();
                    wait_ms(200);
                    lcdPosition(0, 1);
                    lcdPrintf("SW_1/2 OFF!     ");
                } while (RUN_SW == 1 && sw == DATA_UP || RUN_SW == 1 && sw == DATA_DOWN);
                wait_ms(15);
                cnt1 = 0;
                while (sw == 0) {
                    // �X�C�b�`���擾
                    sw = getSwNow();
                    wait_ms(200);

                    servoPwmOut(0);

                    // ��e�X�g
                    motor2_f(100, 100);  // �O �i��,�E�j
                    motor2_r(100, 100);  // ��i��,�E�j

                    servoPwmOut(100);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test CW   ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", 100);
                    wait_ms(2000);

                    motor2_f(0, 0);  // �O �i��,�E�j
                    motor2_r(0, 0);  // ��i��,�E�j
                    servoPwmOut(0);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test STOP ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", 0);
                    wait_ms(1000);

                    motor2_f(-100, -100);  // �O �i��,�E�j
                    motor2_r(-100, -100);  // ��i��,�E�j
                    servoPwmOut(-100);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test CCW   ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", -100);
                    wait_ms(2000);

                    motor2_f(0, 0);  // �O �i��,�E�j
                    motor2_r(0, 0);  // ��i��,�E�j
                    servoPwmOut(0);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test STOP ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", 0);
                    wait_ms(1000);
                    if (cnt1 > 3000) break;
                }
                motor_f(0, 0);  // �O �i��,�E�j
                motor_r(0, 0);  // ��i��,�E�j
                do {
                    // �X�C�b�`���擾
                    sw = getSwNow();
                    wait_ms(200);
                } while (sw == 0x01 || sw == 0x02);
            }
            break;

        case 11:
            /* ���[�^�[�e�X�g�h���C�o��m�F */

            /* LCD���� */
            lcdPosition(0, 0);
            lcdPrintf("10 Motor_Test   ");
            lcdPosition(0, 1);
            lcdPrintf("SW_1 ON!        ");

            if (sw == 0x01 || sw == 0x02) {
                do {
                    // �X�C�b�`���擾
                    sw = getSwNow();
                    wait_ms(200);
                    lcdPosition(0, 1);
                    lcdPrintf("SW_1 OFF!       ");
                } while (sw == 0x01 || sw == 0x02);
                wait_ms(15);
                i = 4;
                while (sw == 0 || i < 8) {
                    // �X�C�b�`���擾
                    sw = getSwNow();
                    wait_ms(200);

                    servoPwmOut(0);

                    // ��e�X�g
                    if (i < 10)
                        i++;
                    else
                        i = 4;
                    motor2_f(i * 10, i * 10);  // �O �i��,�E�j
                    motor2_r(i * 10, i * 10);  // ��i��,�E�j
                    servoPwmOut(i * 10);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test CW   ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", i * 10);
                    wait_ms(2000);

                    motor2_f(0, 0);  // �O �i��,�E�j
                    motor2_r(0, 0);  // �� �i��,�E�j
                    servoPwmOut(0);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test STOP ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", 0);
                    wait_ms(1000);

                    motor2_f(-i * 10, -i * 10);  // �O �i��,�E�j
                    motor2_r(-i * 10, -i * 10);  // ��i��,�E�j
                    servoPwmOut(-i * 10);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test CCW   ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", -i * 10);
                    wait_ms(2000);

                    motor2_f(0, 0);  // �O �i��,�E�j
                    motor2_r(0, 0);  // �� �i��,�E�j
                    servoPwmOut(0);

                    lcdPosition(0, 0);
                    lcdPrintf("Motor Test STOP ");
                    lcdPosition(0, 1);
                    lcdPrintf("Power = %4d%%   ", 0);
                    wait_ms(1000);
                }
                motor_f(0, 0);  // �O �i��,�E�j
                motor_r(0, 0);  // �� �i��,�E�j
                do {
                    // �X�C�b�`���擾
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
	static int pattern11count;   /*11�J�E���g�p			 		 */
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

