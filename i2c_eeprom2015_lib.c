/****************************************************************************/
/* �Ώۃ}�C�R�� R8C/38A                                                     */
/* ̧�ٓ��e     I2C�ɂ��EEP-ROM(24C256B)����                               */
/* �o�[�W����   Ver.2.00                                                    */
/* Date         2014.12.30                                                  */
/* Copyright    �W���p���}�C�R���J�[�����[���s�ψ���                        */
/****************************************************************************/

/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include    "sfr_r838a.h"               /* R8C/38A SFR�̒�`�t�@�C��    */
#include    "i2c_eeprom2015_lib.h"          /* EEP-ROM�֘A����              */

/*======================================*/
/* �V���{����`                         */
/*======================================*/

/* setPageWriteI2CEeprom�֐��P��ŕۑ��o����T�C�Y */
#define EEP_SAVE_SIZE       16          /* �ő�ۑ��� 1,2,4,8,16�̂ǂꂩ*/

/* ���̑���` */
#define ACK                 0           /* ���[�h����ACK�L��(ACK=0)     */
#define NO_ACK              1           /* ���[�h����ACK����(ACK=1)     */

#define STOP_RD             0           /* �X�g�b�v�̒��O�͓ǂݍ���     */
#define STOP_WR             1           /* �X�g�b�v�̒��O�͏�������     */

/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/

volatile signed char            write_buff[ 1+3+16 ];
                                        /* �������݃f�[�^�ۑ��o�b�t�@   */
                                        /* �擪���J����[1]����g�p      */
volatile signed char            *read_address;
                                        /* �f�[�^��ǂݍ��ރA�h���X     */
static volatile int             write_mode;
                                        /* �������e                     */
static volatile int             write_count;
                                        /* �������݌�                 */
static volatile unsigned char   eep_address;
                                        /* �A�h���X�̑I��               */

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/

/************************************************************************/
/* ���W���[���� i2c_start                                               */
/* �����T�v     �X�^�[�g�֘A���W�X�^�ݒ�A�X�^�[�g�M�����M              */
/************************************************************************/
void i2c_start( void )
{
    stop_icsr = 0;
    while( bbsy_iccr2 == 1 );
    iccr1 |= 0x30;                      /* �}�X�^���M���[�h             */
    iccr2 = 0x90;                       /* �X�^�[�g�������s             */
}

/************************************************************************/
/* ���W���[���� i2c_stop                                                */
/* �����T�v     �X�g�b�v�M�����M                                        */
/************************************************************************/
void i2c_stop( int mode )
{
    if( mode == STOP_WR ) {
        iccr2 = 0x10;                   /* �X�g�b�v�������s             */
        tend_icsr = 0;
        nackf_icsr = 0;
        while(stop_icsr == 0);
        stop_icsr = 0;
    }
    iccr1 &= 0xcf;                      /* �X���[�u��M���[�h           */
}

/************************************************************************/
/* ���W���[���� i2c_write                                               */
/* �����T�v     EEP-ROM�@�P�o�C�g��������                               */
/* ����         signed char �f�[�^                                      */
/* �߂�l       int  acknowledge 0:�L��  1:����                         */
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
/* ���W���[���� i2c_read                                                */
/* �����T�v     EEP-ROM�@�P�o�C�g�ǂݍ���                               */
/* ����         int ACK�F�f�[�^������ NO_ACK�F�f�[�^�I��                */
/* �߂�l       signed char �f�[�^                                      */
/************************************************************************/
signed char i2c_read( int ack )
{
    signed char ret;

    tend_icsr = 0;
    iccr1 &= 0xef;
    tdre_icsr = 0;
    ackbt_icier = 1;
    ret = icdrr;                        /* �_�~�[���[�h                 */

    if( ack == NO_ACK ) {
        rcvd_iccr1 = 1;                 /* ���̎�M������֎~           */
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
/* ���W���[���� initI2CEeprom                                           */
/* �����T�v     EEP-ROM�̃|�[�g������                                   */
/* ����         �Ȃ�                                                    */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
void initI2CEeprom( void )
{
    // ���o�͐ݒ�
    pd3 &= 0x5f;                        /* P3_7:SDA P3_5:SCL            */

    // IIC�o�X�ݒ�
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

    selectI2CEepromAddress( 0 );        /* �A�h���X�̑I��               */

    /* DTC�̐ݒ� */
    dtcvct17 = 0;                       /* DTC�x�N�^�A�h���X IIC���M    */
    dtccr0 = 0x04;                      /* �m�[�}�����[�h               */
}

/************************************************************************/
/* ���W���[���� selectI2CEepromAddress                                  */
/* �����T�v     �ǂ̔ԍ���EEP-ROM���g�p���邩�I��                       */
/* ����         unsigned char EEP-ROM�̃A�h���X 0�`3                    */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
void selectI2CEepromAddress( unsigned char address )
{
    address &= 0x03;
    eep_address = address << 1;
}

/************************************************************************/
/* ���W���[���� readI2CEeprom                                           */
/* �����T�v     EEP-ROM�ǂݍ���                                         */
/* ����         unsigned long   �A�h���X 0-32767                        */
/* �߂�l       signed char     �f�[�^                                  */
/************************************************************************/
signed char readI2CEeprom( unsigned long address )
{
    signed char ret;

    asm( "FCLR I" );                    /* �S�̂̊��荞�݋֎~           */

    i2c_start();
    i2c_write( 0xa0 | eep_address );    /* device address(write)        */
    i2c_write( (address >> 8) & 0xff ); /* first address                */
    i2c_write( address & 0xff );        /* second address               */
    iccr2 = 0x90;                       /* �X�^�[�g�đ�                 */
    i2c_write( 0xa1 | eep_address );    /* device address(read)         */
    ret = i2c_read( NO_ACK );           /* data read                    */
    i2c_stop( STOP_RD );

    asm( "FSET I" );                    /* �S�̂̊��荞�݋���           */

    return ret;
}

/************************************************************************/
/* ���W���[���� writeI2CEeprom                                          */
/* �����T�v     EEP-ROM��������                                         */
/* ����         unsigned long �A�h���X 0-32767 ,signed char �f�[�^      */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
void writeI2CEeprom( unsigned long address, signed char write )
{
    int i;

    asm( "FCLR I" );                    /* �S�̂̊��荞�݋֎~           */

    i2c_start();
    i2c_write( 0xa0 | eep_address );    /* device address(write)        */
    i2c_write( (address >> 8) & 0xff ); /* first address                */
    i2c_write( address & 0xff );        /* second address               */
    i2c_write( write );                 /* data write                   */
    i2c_stop( STOP_WR );

    asm( "FSET I" );                    /* �S�̂̊��荞�݋���           */
}

/************************************************************************/
/* ���W���[���� setPageWriteI2CEeprom                                   */
/* �����T�v     EEP-ROM�y�[�W��������                                   */
/*              ���̊֐��Ńy�[�W�������݂��J�n����                      */
/* ����         unsigned long �A�h���X 0-32767 , int ��,              */
/*              signed char* �f�[�^������A�h���X                       */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
void setPageWriteI2CEeprom(unsigned long address, int count, signed char* data )
{
    volatile signed char *p = write_buff + 1;

    /* �������ݒ��Ȃ炱�̊֐��͒��~ */
    if( write_mode != 0 || count == 0 ) return;

    /* ����̃`�F�b�N */
    if( count >= EEP_SAVE_SIZE ) count = EEP_SAVE_SIZE;

    write_count = count + 3;

    *p++ = 0xa0 | eep_address;          /* device address(write)        */
    *p++ = (address >> 8) & 0xff;       /* first address                */
    *p++ = address & 0xff;              /* second address               */

    /* EE-ROM�������݃f�[�^���o�b�t�@�ɕۑ� */
    /* #if�́A�ʏ��0�ɂ��Ă��������B�œK���Ȃǂ̃R���p�C���I�v�V������ */
    /* �ǉ����āA���삵�Ȃ��Ȃ����ꍇ�́A�u#if 1�v�ɂ��Ă��������B      */
#if 0
    do {
        *p++ = *data++;                 /* �f�[�^�ۑ�                   */
    } while( --count );
#else
    /* 16�o�C�g write_buff�֓]�� */
    read_address = (signed char*)data;  /* �]�����Z�b�g                 */
    _asm( " mov.b _read_address >> 16   ,r1h " );
    _asm( " mov.w _read_address & 0ffffh, a0 " );
    _asm( " mov.w #_write_buff+4        , a1 " );
    _asm( " mov.w #8                    , r3 " );
    _asm( " smovf.w"                           );
#endif
    // DTC�̐ݒ�
    dtbls0 = 1;                         /* 1��̋N���œ]�����ް���ۯ����*/
    dtcct0 = write_count;               /* �f�[�^�]����               */
    dtrld0 = 0;
    dtsar0 = (unsigned short)write_buff+1;  /* ������ڽ(��)             */
    dtdar0 = (unsigned short)&icdrt;        /* �ި�èȰ��ݱ��ڽ(��)     */
    dtcen26 = 1;                        /* IIC���M�ް�����è�����ŋN��  */

    // IIC�̐ݒ�
    stop_icsr = 0;
    iccr1 |= 0x30;                      /* �}�X�^���M���[�h             */
    iccr2  = 0xb0;                      /* �X�^�[�g�������s             */
    tie_icier = 1;                      /* ���M�ް�����è�����v������   */
                                        /* ��������DTC�ɂ��]���J�n    */
    write_mode = 2;
}

/************************************************************************/
/* ���W���[���� I2CEepromProcess                                        */
/* �����T�v     EEP-ROM�y�[�W��������                                   */
/* ����         �Ȃ�                                                    */
/* �߂�l       �Ȃ�                                                    */
/************************************************************************/
void I2CEepromProcess( void )
{
    switch( write_mode ) {
    case 0:
        // �������Ȃ�
        break;

    case 2:
        // �]��������������
        if( dtcct0 == 0 ) {
            tie_icier = 0;      /* ���M�f�[�^�G���v�e�B���荞�ݗv���֎~ */
            write_mode = 3;
            break;
        }
        break;

    case 3:
        // �Ō�̃f�[�^���M�I��������
        if( tend_icsr == 1 ) {
            write_mode = 11;
            break;
        }
        break;

    case 11:
        // ��~�������s
        //stop_icsr = 0;
        iccr2 = 0x30;                   /* ��~�������s                 */
        tend_icsr = 0;
        write_mode = 12;
        break;

    case 12:
        // ��~�����̌��o
        if( stop_icsr != 0 ) {
            stop_icsr = 0;
            iccr1 &= 0xcf;              /* �X���[�u��M���[�h           */
            ackbt_icier = 0;
            write_mode = 0;
            break;
        }
        break;
    }
}

/************************************************************************/
/* ���W���[���� clearI2CEeprom                                          */
/* �����T�v     EEP-ROM�@�I�[���N���A                                   */
/* ����         char* �N���A���Ƀ��j�^����LED�̃|�[�g                   */
/*              int   �N���A���Ƀ��j�^����LED�̃r�b�g                   */
/* �߂�l       �Ȃ�                                                    */
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
/* ���W���[���� checkI2CEeprom                                          */
/* �����T�v     EEP-ROM�������݌�A���ɓǂݏ����ł��邩�`�F�b�N         */
/* ����         �Ȃ�                                                    */
/* �߂�l       1:���ǂݏ���OK 0:�܂�                                   */
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
�����o��

2010.04.01 Ver.1.00 �쐬
2011.04.01 Ver.1.01 �ϐ���volatile�ǉ�
2011.08.13 Ver.1.02 EEP-ROM�ɕۑ�����f�[�^�^��char�^����unsigned char�^�ɕύX
2014.12.30 Ver.2.00 setPageWriteI2CEeprom�֐��AI2CEepromProcess�֐��̓]����
                    DTC���g�����]���ɂ���
*/
