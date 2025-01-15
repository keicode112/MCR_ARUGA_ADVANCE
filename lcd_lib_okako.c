/****************************************************************************/
/* �Ώۃ}�C�R�� R8C/38A                                                     */
/* ̧�ٓ��e     �t�����䃉�C�u����                                          */
/* �o�[�W����   Ver.1.02                                                    */
/* Date         2012.04.01                                                  */
/* Copyright    �W���p���}�C�R���J�[�����[���s�ψ���                        */
/****************************************************************************/
/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include    <stdio.h>                   /* �W��Cײ���� ���o��           */
#include    <stdarg.h>                  /* �W��Cײ���� �ό��̎����� */
#include    "sfr_r838a.h"               /* R8C/38A SFR�̒�`�t�@�C��    */
#include    "lcd_lib.h"                 /* �t���֘A����                 */

/* P0 B0-D4 �f�[�^*/
/* P2 B6-Enable LCD ���I�����ꂽ���ǂ����̐M�� */
/* P2 B5-RS (LCD �ɑ΂���R�}���h�������f�[�^���̐M��) */

#define  LCDDATA   p0
#define  RS        p0_4   /* �R�}���h�E�f�[�^�M�� */
#define  RW        p0_5   /* �R�}���h�E�f�[�^�M�� */
#define  E         p0_6  /* Enable �M�� */

#define LCD_PORT            p0          /* �t���̐ڑ�����Ă���|�[�g   */
#define LCD_PORT_DIR        pd0         /* ��L�|�[�g�̓��o�͐ݒ�|�[�g */
#define LCD_PORT_PULLUP     pu00        /* p0_0�`p0_3�[�q���v���A�b�v���郌�W�X�^*/

#define LCD_NODATA_BIT      0x80        /* �t���Ŏg���Ă��Ȃ��r�b�g     */
#define LCD_E_BIT           0x40        /* �t�� E bit                   */
#define LCD_RW_BIT          0x20        /* �t�� RW bit                  */
#define LCD_RS_BIT          0x10        /* �t�� RS bit                  */
#define LCD_D7_BIT          0x08        /* �t�� D7 bit                  */

/* �t���֘A�ϐ� */
#define LCD_MAX_X           16          /* �\�������� �� 16 or 20       */
#define LCD_MAX_Y           2           /* �\�������� �c  2 or  4       */

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
static void out4(char code, char rs);	/* code ���̉��� 4bit �� LCD �֓]������ rs�� RS �ɓn������ */
static void out8(char code, char rs);	/* code 1Byte �� LCD �֓]������ rs �� RS �ɓn������ */
static void wait_ms_lcd(unsigned int t);/* Wait */
static void locate(int x, int y);		/* LCD �̃J�[�\�����ړ������� x=0�`15 y=0,1 */
static int wait(volatile int times);
/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
static volatile char            buffLcdData[ LCD_MAX_X * LCD_MAX_Y ];
                                        /* �\���o�b�t�@                 */
static volatile char            buffLcdData2[ LCD_MAX_X * LCD_MAX_Y + 10 ];
                                        /* �\���o�b�t�@�ꎞ��ƃG���A   */
static volatile int             lcdBuffPosition;
                                        /* �o�b�t�@�ɏ������ވʒu       */
static volatile int             lcdMode = 1;
                                        /* �\������No�Ǘ�               */
static volatile int             lcdNowLocate;
                                        /* ���݂̕\�����Ă���ʒu       */
static volatile unsigned long   lcdBusyCnt;
                                        /* busy�X�^�[�g�J�E���^         */
static volatile int             lcdRefreshFlag;
                                        /* ���t���b�V���t���O           */
static volatile int             lcdConnect = 1;
                                        /* �t�����ڑ�����Ă��邩       */
static volatile unsigned long   lcdCnt; /* �t���^�C�~���O�p�J�E���^     */


/************************************************************************/
/* ���W���[���� lcdBusyStart                                            */
/* �����T�v     �t����busy�X�^�[�g                                      */
/* ����         busy�l                                                  */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
static void lcdBusyStart( void )
{
    lcdBusyCnt = lcdCnt;
}

/************************************************************************/
/* ���W���[���� checkLcdBusy                                            */
/* �����T�v     �t����busy�`�F�b�N                                      */
/* ����         �Ȃ�                                                    */
/* �߂�l       2:�^�C���A�E�g 1:ok 0:busy��                            */
/************************************************************************/
static int checkLcdBusy( void )
{
    volatile int ret;

    /* D3�`D0����͒[�q�ɂ��� */
    LCD_PORT_PULLUP = 1;
    LCD_PORT_DIR   &= 0xf0;

    LCD_PORT &= ( ~LCD_E_BIT & ~LCD_RW_BIT & ~LCD_RS_BIT );

    LCD_PORT |= LCD_RW_BIT;
    wait(1);

    LCD_PORT |= LCD_E_BIT;
    wait(5);

    ret = (LCD_PORT & LCD_D7_BIT) ? 0 : 1;
    if( ret == 0 ) {
        /* busy�Ȃ�A�^�C���A�E�g���`�F�b�N */
        if( lcdCnt - lcdBusyCnt >= 11 ) ret = 2;
    }

    LCD_PORT &= ~LCD_E_BIT;
    wait(5);

    LCD_PORT |= LCD_E_BIT;              /* 4bit���[�h�Ȃ̂�             */
    wait(5);                         /* �_�~�[��E bit���Ă�          */

    LCD_PORT &= ~LCD_E_BIT;
    wait(5);

    LCD_PORT &= ~LCD_RW_BIT;

    /* D3�`D0���o�͒[�q�ɂ��� */
    LCD_PORT_PULLUP = 0;
    LCD_PORT_DIR   |= 0x0f;

    return ret;
}

/************************************************************************/
/* ���W���[���� initLcd                                                 */
/* �����T�v     �t�������@������                                        */
/* ����         �Ȃ�                                                    */
/************************************************************************/
void initLcd(void)
{
    volatile int i;
	
	// �ϐ�������
    for( i=0; i<=LCD_MAX_X*LCD_MAX_Y-1; i++ ) 
	{
        buffLcdData[ i ] = ' ';
    }
	
    /* D3�`D0���o�͒[�q�ɂ��� */
    LCD_PORT_PULLUP = 0;
    prc2 = 1;                           /* PD0�̃v���e�N�g����          */
    LCD_PORT_DIR   |= 0x0f;
	RW=0;
	wait_ms_lcd(15);     /*15ms�҂�*/
	out4(0x03, 0);  
	wait_ms_lcd(4);     /*4ms�҂�*/
	out4(0x03, 0);
	out4(0x03, 0);
	out4(0x02, 0);

	out8(0x28, 0);   /* function set(0x28=0010 1000) */
	/* DL=0(4bit data),N=1(2 line),F=0(5*7dots font) '001(DL)NF**' */	
	out8(0x0c, 0);   /* display on off control(0x0c=0000 1100) */
	/* D=1(display on),C=0(cursol off),B=0(cursol not blink) '0000 1DCB'*/	
	out8(0x06, 0);   /* entry mode set(0x06=0000 0110) */
	/* (I/D)=1(increment),S=0(display shift) '0000 01(I/D)S' */
	
	clr_lcd();       /* �t���N���A */
}

/************************************************************************/
/* ���W���[���� out4		                                                */
/* �����T�v     lcdOut�̏�ʁA���ʂɕ����ē]�����镔��       		        */
/* ����         4bit�f�[�^                                              */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
static void out4(char code, char rs)
{
   LCDDATA  = code & 0x0f;  /* code �̉��� 4bit �� �f�[�^�M�� D4-7�ɐݒ� */
   RS = rs; wait_ms_lcd(4);         /* RS ��ݒ�(1...�f�[�^�C0...�R�}���h) */
   E = 1; wait_ms_lcd(4);           /* E = 'H' */
   E = 0; wait_ms_lcd(4);           /* E = 'L' */
}

/************************************************************************/
/* ���W���[���� out8  	                                                */
/* �����T�v     �t���R�}���h�o��                                        */
/* ����         �R�}���h��ށA�f�[�^                                    */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
static void out8(char code, char rs)
{
   out4((0xf0 & code)/0x10, rs);   /* ch(�̉��� 4bit) ���o�� */
   out4(0x0f & code, rs);          /* cl(�̉��� 4bit) ���o�� */
}

/************************************************************************/
/* �^�C�}                                                               	*/
/* �ɒZ���^�C�~���O�p�^�C�} 1������1us���炢                            	*/
/************************************************************************/
static int wait(volatile int times)
{
	/* �����҂� */
    while( times-- );
}

/************************************************************************/
/* ���W���[���� locate                                               */
/* �����T�v     �t���J�[�\���ړ�                                        */
/* ����         x , y                                                   */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
static void locate(int x, int y)
{
    volatile unsigned char work = 0x80;

    /* x�̌v�Z */
    work += x;

    /* y�̌v�Z */
    if( y == 1 ) {
        work += 0x40;
    } else if( y == 2 ) {
        work += 0x14;
    } else if( y == 3 ) {
        work += 0x54;
    }

    /* �J�[�\���ړ� */
    out8( work, 0 );
	lcdBusyStart();
}

void clr_lcd(void)
{   /* �t���N���A */
   out8(0x01, 0);
}

/************************************************************************/
/* �^�C�}                                                               */
/* �����@ �^�C�}�l 1=1ms                                                */
/************************************************************************/
static void wait_ms_lcd(unsigned int t)
{
	unsigned int i,j;
	for (i = 0; i < t; i++)
	{
		for(j = 0; j < 677; j++);/*1ms�̑҂�*/
	}
}

/************************************************************************/
/* ���W���[���� lcdShowProcess                                          */
/* �����T�v     �t���\������                                            */
/* ����         �Ȃ�                                                    */
/* �߂�l       �Ȃ�                                                    */
/* ����         ���̊֐��͊��荞�݂�1ms���ƂɎ��s���Ă�������           */
/************************************************************************/
void lcdShowProcess( void )
{
    if( !lcdConnect ) return;          /* �ڑ�����Ă��邩�`�F�b�N  */

    lcdCnt++;

    switch( lcdMode ) {
    case 1: /* �f�[�^�X�V���ꂽ���`�F�b�N */
        if( lcdRefreshFlag ) {
            lcdRefreshFlag = 0;
            lcdMode = 2;
        }
        break;

    case 2: /* �ʒu������*/
        if( checkLcdBusy() ) {
            lcdNowLocate = 0;
            locate( 0, 0 );
            lcdMode = 3;
        }
        break;

    case 3: /* ���s�ʒu�̊m�F */
        if( checkLcdBusy() ) {
            if( lcdNowLocate % LCD_MAX_X == 0 ) {
                locate( 0, lcdNowLocate / LCD_MAX_X );
            }
            lcdMode = 4;
        }
        break;

    case 4: /* �f�[�^�\������ */
        if( checkLcdBusy() ) {
            out8( buffLcdData[ lcdNowLocate++ ], 1);
            lcdBusyStart();
            if( lcdNowLocate >= LCD_MAX_X * LCD_MAX_Y ) {
                lcdMode = 1;
            } else {
                lcdMode = 3;
            }
        }
        break;

    default:
        lcdMode = 1;
        break;
    }
}

/************************************************************************/
/* ���W���[���� lcdPrintf                                               */
/* �����T�v     �t���֕\���@�\���ʒu�͉ߋ��ɕ\�������ʒu�̎�����        */
/* ����         printf�Ɠ���                                            */
/* �߂�l       ���펞�F�o�͂���������@�ُ펞�F���̐�                  */
/************************************************************************/
int lcdPrintf(char *format, ...)
{
    volatile va_list argptr;
    volatile char    *p;
    volatile int     ret = 0;

    va_start(argptr, format);
    ret = vsprintf( buffLcdData2, format, argptr );
    va_end(argptr);

    if( ret > 0 ) {
        /* vsprintf������Ȃ�t���o�b�t�@�֓]�� */
        p = buffLcdData2;
        while( *p ) {
            /*buffLcdData[lcdBuffPosition++] = *p++;
            if( lcdBuffPosition >= LCD_MAX_X * LCD_MAX_Y ) {
                lcdBuffPosition = 0;
            }*/
            out8(*p, 1);
            p++;
        }
        lcdRefreshFlag = 1;
    }
    return ret;
}

/************************************************************************/
/* ���W���[���� lcdPosition                                             */
/* �����T�v     �t���̕\���ʒu�w��                                      */
/* ����         ���ʒu , �c�ʒu                                         */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
void lcdPosition(char x ,char y)
{
    /*
    if( x >= LCD_MAX_X ) return;
    if( y >= LCD_MAX_Y ) return;

    lcdBuffPosition = x + y * LCD_MAX_X;
    */
    locate(x, y);
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
�����o��

2011.04.01 Ver.1.00 �쐬
2011.09.28 Ver.1.01 checkLcdBusy�֐��̃^�C���A�E�g�����C��
2012.04.01 Ver.1.02 �œK����ON�ɂ��Ă����삷��悤�Ή�
2021.08.20 ���J�H�ƍ��ZVer��
*/
