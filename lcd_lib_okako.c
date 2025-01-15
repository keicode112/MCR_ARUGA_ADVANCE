/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     液晶制御ライブラリ                                          */
/* バージョン   Ver.1.02                                                    */
/* Date         2012.04.01                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/
/*======================================*/
/* インクルード                         */
/*======================================*/
#include    <stdio.h>                   /* 標準Cﾗｲﾌﾞﾗﾘ 入出力           */
#include    <stdarg.h>                  /* 標準Cﾗｲﾌﾞﾗﾘ 可変個数の実引数 */
#include    "sfr_r838a.h"               /* R8C/38A SFRの定義ファイル    */
#include    "lcd_lib.h"                 /* 液晶関連処理                 */

/* P0 B0-D4 データ*/
/* P2 B6-Enable LCD が選択されたかどうかの信号 */
/* P2 B5-RS (LCD に対するコマンドか文字データかの信号) */

#define  LCDDATA   p0
#define  RS        p0_4   /* コマンド・データ信号 */
#define  RW        p0_5   /* コマンド・データ信号 */
#define  E         p0_6  /* Enable 信号 */

#define LCD_PORT            p0          /* 液晶の接続されているポート   */
#define LCD_PORT_DIR        pd0         /* 上記ポートの入出力設定ポート */
#define LCD_PORT_PULLUP     pu00        /* p0_0～p0_3端子をプルアップするレジスタ*/

#define LCD_NODATA_BIT      0x80        /* 液晶で使っていないビット     */
#define LCD_E_BIT           0x40        /* 液晶 E bit                   */
#define LCD_RW_BIT          0x20        /* 液晶 RW bit                  */
#define LCD_RS_BIT          0x10        /* 液晶 RS bit                  */
#define LCD_D7_BIT          0x08        /* 液晶 D7 bit                  */

/* 液晶関連変数 */
#define LCD_MAX_X           16          /* 表示文字数 横 16 or 20       */
#define LCD_MAX_Y           2           /* 表示文字数 縦  2 or  4       */

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
static void out4(char code, char rs);	/* code 中の下位 4bit を LCD へ転送する rsは RS に渡すだけ */
static void out8(char code, char rs);	/* code 1Byte を LCD へ転送する rs は RS に渡すだけ */
static void wait_ms_lcd(unsigned int t);/* Wait */
static void locate(int x, int y);		/* LCD のカーソルを移動させる x=0～15 y=0,1 */
static int wait(volatile int times);
/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
static volatile char            buffLcdData[ LCD_MAX_X * LCD_MAX_Y ];
                                        /* 表示バッファ                 */
static volatile char            buffLcdData2[ LCD_MAX_X * LCD_MAX_Y + 10 ];
                                        /* 表示バッファ一時作業エリア   */
static volatile int             lcdBuffPosition;
                                        /* バッファに書き込む位置       */
static volatile int             lcdMode = 1;
                                        /* 表示処理No管理               */
static volatile int             lcdNowLocate;
                                        /* 現在の表示している位置       */
static volatile unsigned long   lcdBusyCnt;
                                        /* busyスタートカウンタ         */
static volatile int             lcdRefreshFlag;
                                        /* リフレッシュフラグ           */
static volatile int             lcdConnect = 1;
                                        /* 液晶が接続されているか       */
static volatile unsigned long   lcdCnt; /* 液晶タイミング用カウンタ     */


/************************************************************************/
/* モジュール名 lcdBusyStart                                            */
/* 処理概要     液晶のbusyスタート                                      */
/* 引数         busy値                                                  */
/* 戻り値       なし                                                    */
/************************************************************************/
static void lcdBusyStart( void )
{
    lcdBusyCnt = lcdCnt;
}

/************************************************************************/
/* モジュール名 checkLcdBusy                                            */
/* 処理概要     液晶のbusyチェック                                      */
/* 引数         なし                                                    */
/* 戻り値       2:タイムアウト 1:ok 0:busy中                            */
/************************************************************************/
static int checkLcdBusy( void )
{
    volatile int ret;

    /* D3～D0を入力端子にする */
    LCD_PORT_PULLUP = 1;
    LCD_PORT_DIR   &= 0xf0;

    LCD_PORT &= ( ~LCD_E_BIT & ~LCD_RW_BIT & ~LCD_RS_BIT );

    LCD_PORT |= LCD_RW_BIT;
    wait(1);

    LCD_PORT |= LCD_E_BIT;
    wait(5);

    ret = (LCD_PORT & LCD_D7_BIT) ? 0 : 1;
    if( ret == 0 ) {
        /* busyなら、タイムアウトかチェック */
        if( lcdCnt - lcdBusyCnt >= 11 ) ret = 2;
    }

    LCD_PORT &= ~LCD_E_BIT;
    wait(5);

    LCD_PORT |= LCD_E_BIT;              /* 4bitモードなので             */
    wait(5);                         /* ダミーでE bitたてる          */

    LCD_PORT &= ~LCD_E_BIT;
    wait(5);

    LCD_PORT &= ~LCD_RW_BIT;

    /* D3～D0を出力端子にする */
    LCD_PORT_PULLUP = 0;
    LCD_PORT_DIR   |= 0x0f;

    return ret;
}

/************************************************************************/
/* モジュール名 initLcd                                                 */
/* 処理概要     液晶処理　初期化                                        */
/* 引数         なし                                                    */
/************************************************************************/
void initLcd(void)
{
    volatile int i;
	
	// 変数初期化
    for( i=0; i<=LCD_MAX_X*LCD_MAX_Y-1; i++ ) 
	{
        buffLcdData[ i ] = ' ';
    }
	
    /* D3～D0を出力端子にする */
    LCD_PORT_PULLUP = 0;
    prc2 = 1;                           /* PD0のプロテクト解除          */
    LCD_PORT_DIR   |= 0x0f;
	RW=0;
	wait_ms_lcd(15);     /*15ms待つ*/
	out4(0x03, 0);  
	wait_ms_lcd(4);     /*4ms待つ*/
	out4(0x03, 0);
	out4(0x03, 0);
	out4(0x02, 0);

	out8(0x28, 0);   /* function set(0x28=0010 1000) */
	/* DL=0(4bit data),N=1(2 line),F=0(5*7dots font) '001(DL)NF**' */	
	out8(0x0c, 0);   /* display on off control(0x0c=0000 1100) */
	/* D=1(display on),C=0(cursol off),B=0(cursol not blink) '0000 1DCB'*/	
	out8(0x06, 0);   /* entry mode set(0x06=0000 0110) */
	/* (I/D)=1(increment),S=0(display shift) '0000 01(I/D)S' */
	
	clr_lcd();       /* 液晶クリア */
}

/************************************************************************/
/* モジュール名 out4		                                                */
/* 処理概要     lcdOutの上位、下位に分けて転送する部分       		        */
/* 引数         4bitデータ                                              */
/* 戻り値       なし                                                    */
/************************************************************************/
static void out4(char code, char rs)
{
   LCDDATA  = code & 0x0f;  /* code の下位 4bit を データ信号 D4-7に設定 */
   RS = rs; wait_ms_lcd(4);         /* RS を設定(1...データ，0...コマンド) */
   E = 1; wait_ms_lcd(4);           /* E = 'H' */
   E = 0; wait_ms_lcd(4);           /* E = 'L' */
}

/************************************************************************/
/* モジュール名 out8  	                                                */
/* 処理概要     液晶コマンド出力                                        */
/* 引数         コマンド種類、データ                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
static void out8(char code, char rs)
{
   out4((0xf0 & code)/0x10, rs);   /* ch(の下位 4bit) を出力 */
   out4(0x0f & code, rs);          /* cl(の下位 4bit) を出力 */
}

/************************************************************************/
/* タイマ                                                               	*/
/* 極短いタイミング用タイマ 1当たり1usくらい                            	*/
/************************************************************************/
static int wait(volatile int times)
{
	/* 少し待つ */
    while( times-- );
}

/************************************************************************/
/* モジュール名 locate                                               */
/* 処理概要     液晶カーソル移動                                        */
/* 引数         x , y                                                   */
/* 戻り値       なし                                                    */
/************************************************************************/
static void locate(int x, int y)
{
    volatile unsigned char work = 0x80;

    /* xの計算 */
    work += x;

    /* yの計算 */
    if( y == 1 ) {
        work += 0x40;
    } else if( y == 2 ) {
        work += 0x14;
    } else if( y == 3 ) {
        work += 0x54;
    }

    /* カーソル移動 */
    out8( work, 0 );
	lcdBusyStart();
}

void clr_lcd(void)
{   /* 液晶クリア */
   out8(0x01, 0);
}

/************************************************************************/
/* タイマ                                                               */
/* 引数　 タイマ値 1=1ms                                                */
/************************************************************************/
static void wait_ms_lcd(unsigned int t)
{
	unsigned int i,j;
	for (i = 0; i < t; i++)
	{
		for(j = 0; j < 677; j++);/*1msの待ち*/
	}
}

/************************************************************************/
/* モジュール名 lcdShowProcess                                          */
/* 処理概要     液晶表示処理                                            */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/* メモ         この関数は割り込みで1msごとに実行してください           */
/************************************************************************/
void lcdShowProcess( void )
{
    if( !lcdConnect ) return;          /* 接続されているかチェック  */

    lcdCnt++;

    switch( lcdMode ) {
    case 1: /* データ更新されたかチェック */
        if( lcdRefreshFlag ) {
            lcdRefreshFlag = 0;
            lcdMode = 2;
        }
        break;

    case 2: /* 位置初期化*/
        if( checkLcdBusy() ) {
            lcdNowLocate = 0;
            locate( 0, 0 );
            lcdMode = 3;
        }
        break;

    case 3: /* 改行位置の確認 */
        if( checkLcdBusy() ) {
            if( lcdNowLocate % LCD_MAX_X == 0 ) {
                locate( 0, lcdNowLocate / LCD_MAX_X );
            }
            lcdMode = 4;
        }
        break;

    case 4: /* データ表示処理 */
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
/* モジュール名 lcdPrintf                                               */
/* 処理概要     液晶へ表示　表示位置は過去に表示した位置の次から        */
/* 引数         printfと同じ                                            */
/* 戻り値       正常時：出力した文字列　異常時：負の数                  */
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
        /* vsprintfが正常なら液晶バッファへ転送 */
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
/* モジュール名 lcdPosition                                             */
/* 処理概要     液晶の表示位置指定                                      */
/* 引数         横位置 , 縦位置                                         */
/* 戻り値       なし                                                    */
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
改訂経歴

2011.04.01 Ver.1.00 作成
2011.09.28 Ver.1.01 checkLcdBusy関数のタイムアウト処理修正
2012.04.01 Ver.1.02 最適化をONにしても動作するよう対応
2021.08.20 岡谷工業高校Verへ
*/
