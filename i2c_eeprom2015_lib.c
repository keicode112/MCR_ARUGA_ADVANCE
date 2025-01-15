/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     I2CによるEEP-ROM(24C256B)処理                               */
/* バージョン   Ver.2.00                                                    */
/* Date         2014.12.30                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include    "sfr_r838a.h"               /* R8C/38A SFRの定義ファイル    */
#include    "i2c_eeprom2015_lib.h"          /* EEP-ROM関連処理              */

/*======================================*/
/* シンボル定義                         */
/*======================================*/

/* setPageWriteI2CEeprom関数１回で保存出来るサイズ */
#define EEP_SAVE_SIZE       16          /* 最大保存数 1,2,4,8,16のどれか*/

/* その他定義 */
#define ACK                 0           /* リード時のACK有効(ACK=0)     */
#define NO_ACK              1           /* リード時のACK無効(ACK=1)     */

#define STOP_RD             0           /* ストップの直前は読み込み     */
#define STOP_WR             1           /* ストップの直前は書き込み     */

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/

volatile signed char            write_buff[ 1+3+16 ];
                                        /* 書き込みデータ保存バッファ   */
                                        /* 先頭を開けて[1]から使用      */
volatile signed char            *read_address;
                                        /* データを読み込むアドレス     */
static volatile int             write_mode;
                                        /* 処理内容                     */
static volatile int             write_count;
                                        /* 書き込み個数                 */
static volatile unsigned char   eep_address;
                                        /* アドレスの選択               */

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/

/************************************************************************/
/* モジュール名 i2c_start                                               */
/* 処理概要     スタート関連レジスタ設定、スタート信号送信              */
/************************************************************************/
void i2c_start( void )
{
    stop_icsr = 0;
    while( bbsy_iccr2 == 1 );
    iccr1 |= 0x30;                      /* マスタ送信モード             */
    iccr2 = 0x90;                       /* スタート条件発行             */
}

/************************************************************************/
/* モジュール名 i2c_stop                                                */
/* 処理概要     ストップ信号送信                                        */
/************************************************************************/
void i2c_stop( int mode )
{
    if( mode == STOP_WR ) {
        iccr2 = 0x10;                   /* ストップ条件発行             */
        tend_icsr = 0;
        nackf_icsr = 0;
        while(stop_icsr == 0);
        stop_icsr = 0;
    }
    iccr1 &= 0xcf;                      /* スレーブ受信モード           */
}

/************************************************************************/
/* モジュール名 i2c_write                                               */
/* 処理概要     EEP-ROM　１バイト書き込み                               */
/* 引数         signed char データ                                      */
/* 戻り値       int  acknowledge 0:有効  1:無効                         */
/************************************************************************/
int i2c_write( signed char data )
{
    int ret;

    icdrt = data;
    while( tdre_icsr == 0 );
    while( tend_icsr == 0 );

    ret = ackbr_icier;

    return ret;
}

/************************************************************************/
/* モジュール名 i2c_read                                                */
/* 処理概要     EEP-ROM　１バイト読み込み                               */
/* 引数         int ACK：データが続く NO_ACK：データ終了                */
/* 戻り値       signed char データ                                      */
/************************************************************************/
signed char i2c_read( int ack )
{
    signed char ret;

    tend_icsr = 0;
    iccr1 &= 0xef;
    tdre_icsr = 0;
    ackbt_icier = 1;
    ret = icdrr;                        /* ダミーリード                 */

    if( ack == NO_ACK ) {
        rcvd_iccr1 = 1;                 /* 次の受信動作を禁止           */
    }

    while( rdrf_icsr == 0 );
    iccr2 = 0x10;
    while( stop_icsr == 0 );
    ret = icdrr;
    rcvd_iccr1 = 0;
    ackbt_icier = 0;

    return ret;
}

/************************************************************************/
/* モジュール名 initI2CEeprom                                           */
/* 処理概要     EEP-ROMのポート初期化                                   */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void initI2CEeprom( void )
{
    // 入出力設定
    pd3 &= 0x5f;                        /* P3_7:SDA P3_5:SCL            */

    // IICバス設定
    iicsel = 1;
    stop_icsr = 0;
    ice_iccr1 = 1;
    icmr  = 0x00;
    iccr1 = 0x82;
    icier = 0x00;
    icsr  = 0x00;
    sar   = 0x00;
    sdadly1 = 1;
    sdadly0 = 0;

    selectI2CEepromAddress( 0 );        /* アドレスの選択               */

    /* DTCの設定 */
    dtcvct17 = 0;                       /* DTCベクタアドレス IIC送信    */
    dtccr0 = 0x04;                      /* ノーマルモード               */
}

/************************************************************************/
/* モジュール名 selectI2CEepromAddress                                  */
/* 処理概要     どの番号のEEP-ROMを使用するか選択                       */
/* 引数         unsigned char EEP-ROMのアドレス 0～3                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void selectI2CEepromAddress( unsigned char address )
{
    address &= 0x03;
    eep_address = address << 1;
}

/************************************************************************/
/* モジュール名 readI2CEeprom                                           */
/* 処理概要     EEP-ROM読み込み                                         */
/* 引数         unsigned long   アドレス 0-32767                        */
/* 戻り値       signed char     データ                                  */
/************************************************************************/
signed char readI2CEeprom( unsigned long address )
{
    signed char ret;

    asm( "FCLR I" );                    /* 全体の割り込み禁止           */

    i2c_start();
    i2c_write( 0xa0 | eep_address );    /* device address(write)        */
    i2c_write( (address >> 8) & 0xff ); /* first address                */
    i2c_write( address & 0xff );        /* second address               */
    iccr2 = 0x90;                       /* スタート再送                 */
    i2c_write( 0xa1 | eep_address );    /* device address(read)         */
    ret = i2c_read( NO_ACK );           /* data read                    */
    i2c_stop( STOP_RD );

    asm( "FSET I" );                    /* 全体の割り込み許可           */

    return ret;
}

/************************************************************************/
/* モジュール名 writeI2CEeprom                                          */
/* 処理概要     EEP-ROM書き込み                                         */
/* 引数         unsigned long アドレス 0-32767 ,signed char データ      */
/* 戻り値       なし                                                    */
/************************************************************************/
void writeI2CEeprom( unsigned long address, signed char write )
{
    int i;

    asm( "FCLR I" );                    /* 全体の割り込み禁止           */

    i2c_start();
    i2c_write( 0xa0 | eep_address );    /* device address(write)        */
    i2c_write( (address >> 8) & 0xff ); /* first address                */
    i2c_write( address & 0xff );        /* second address               */
    i2c_write( write );                 /* data write                   */
    i2c_stop( STOP_WR );

    asm( "FSET I" );                    /* 全体の割り込み許可           */
}

/************************************************************************/
/* モジュール名 setPageWriteI2CEeprom                                   */
/* 処理概要     EEP-ROMページ書き込み                                   */
/*              この関数でページ書き込みを開始する                      */
/* 引数         unsigned long アドレス 0-32767 , int 個数,              */
/*              signed char* データがあるアドレス                       */
/* 戻り値       なし                                                    */
/************************************************************************/
void setPageWriteI2CEeprom(unsigned long address, int count, signed char* data )
{
    volatile signed char *p = write_buff + 1;

    /* 書き込み中ならこの関数は中止 */
    if( write_mode != 0 || count == 0 ) return;

    /* 上限のチェック */
    if( count >= EEP_SAVE_SIZE ) count = EEP_SAVE_SIZE;

    write_count = count + 3;

    *p++ = 0xa0 | eep_address;          /* device address(write)        */
    *p++ = (address >> 8) & 0xff;       /* first address                */
    *p++ = address & 0xff;              /* second address               */

    /* EE-ROM書き込みデータをバッファに保存 */
    /* #ifは、通常は0にしてください。最適化などのコンパイラオプションを */
    /* 追加して、動作しなくなった場合は、「#if 1」にしてください。      */
#if 0
    do {
        *p++ = *data++;                 /* データ保存                   */
    } while( --count );
#else
    /* 16バイト write_buffへ転送 */
    read_address = (signed char*)data;  /* 転送元セット                 */
    _asm( " mov.b _read_address >> 16   ,r1h " );
    _asm( " mov.w _read_address & 0ffffh, a0 " );
    _asm( " mov.w #_write_buff+4        , a1 " );
    _asm( " mov.w #8                    , r3 " );
    _asm( " smovf.w"                           );
#endif
    // DTCの設定
    dtbls0 = 1;                         /* 1回の起動で転送ｽﾙﾃﾞｰﾀﾌﾞﾛｯｸｻｲｽ*/
    dtcct0 = write_count;               /* データ転送回数               */
    dtrld0 = 0;
    dtsar0 = (unsigned short)write_buff+1;  /* ｿｰｽｱﾄﾞﾚｽ(元)             */
    dtdar0 = (unsigned short)&icdrt;        /* ﾃﾞｨｽﾃｨﾈｰｼｮﾝｱﾄﾞﾚｽ(先)     */
    dtcen26 = 1;                        /* IIC送信ﾃﾞｰﾀｴﾝﾌﾟﾃｨ割込で起動  */

    // IICの設定
    stop_icsr = 0;
    iccr1 |= 0x30;                      /* マスタ送信モード             */
    iccr2  = 0xb0;                      /* スタート条件発行             */
    tie_icier = 1;                      /* 送信ﾃﾞｰﾀｴﾝﾌﾟﾃｨ割込要求許可   */
                                        /* ここからDTCによる転送開始    */
    write_mode = 2;
}

/************************************************************************/
/* モジュール名 I2CEepromProcess                                        */
/* 処理概要     EEP-ROMページ書き込み                                   */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void I2CEepromProcess( void )
{
    switch( write_mode ) {
    case 0:
        // 何もしない
        break;

    case 2:
        // 転送が完了したか
        if( dtcct0 == 0 ) {
            tie_icier = 0;      /* 送信データエンプティ割り込み要求禁止 */
            write_mode = 3;
            break;
        }
        break;

    case 3:
        // 最後のデータ送信終了したか
        if( tend_icsr == 1 ) {
            write_mode = 11;
            break;
        }
        break;

    case 11:
        // 停止条件発行
        //stop_icsr = 0;
        iccr2 = 0x30;                   /* 停止条件発行                 */
        tend_icsr = 0;
        write_mode = 12;
        break;

    case 12:
        // 停止条件の検出
        if( stop_icsr != 0 ) {
            stop_icsr = 0;
            iccr1 &= 0xcf;              /* スレーブ受信モード           */
            ackbt_icier = 0;
            write_mode = 0;
            break;
        }
        break;
    }
}

/************************************************************************/
/* モジュール名 clearI2CEeprom                                          */
/* 処理概要     EEP-ROM　オールクリア                                   */
/* 引数         char* クリア中にモニタするLEDのポート                   */
/*              int   クリア中にモニタするLEDのビット                   */
/* 戻り値       なし                                                    */
/************************************************************************/
void clearI2CEeprom( char *led_port , int bit )
{
    unsigned int    address = 0;
    int             i;

    while( address < 32768 ) {
        if( !(address % 0x800) ) *led_port ^= (1<<bit);

        i2c_start();
        i2c_write( 0xa0 | eep_address ); /* device address(write)       */
        i2c_write( address >> 8 );      /* first address                */
        i2c_write( address & 0xff );    /* second address               */
        for( i=0; i<64; i++ ) {
            i2c_write( 0 );             /* data write                   */
        }
        i2c_stop( STOP_WR );

        while( !checkI2CEeprom() );     /* wait                         */

        address += 64;
    }
}

/************************************************************************/
/* モジュール名 checkI2CEeprom                                          */
/* 処理概要     EEP-ROM書き込み後、次に読み書きできるかチェック         */
/* 引数         なし                                                    */
/* 戻り値       1:次読み書きOK 0:まだ                                   */
/************************************************************************/
int checkI2CEeprom( void )
{
    int ret;

    if( write_mode != 0 ) return 0;

    i2c_start();
    ret = !i2c_write( 0xa0 | eep_address ); /* device address(write)    */
    i2c_stop( STOP_WR );

    return ret;
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
改訂経歴

2010.04.01 Ver.1.00 作成
2011.04.01 Ver.1.01 変数にvolatile追加
2011.08.13 Ver.1.02 EEP-ROMに保存するデータ型をchar型からunsigned char型に変更
2014.12.30 Ver.2.00 setPageWriteI2CEeprom関数、I2CEepromProcess関数の転送を
                    DTCを使った転送にする
*/
