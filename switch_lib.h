/*======================================*/
/* 定数設定                             */
/*======================================*/
//#define     SW_0    (0x01 << 0)         /* getSwNow関数のbit0の位置     */
//#define     SW_1    (0x01 << 1)         /* getSwNow関数のbit1の位置     */
//#define     SW_2    (0x01 << 2)         /* getSwNow関数のbit2の位置     */
//#define     SW_3    (0x01 << 3)         /* getSwNow関数のbit3の位置     */
//#define     SW_4    (0x01 << 4)         /* getSwNow関数のbit4の位置     */

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
//void initSwitch( void );
//unsigned char getSwNow( void );
//unsigned char getSwFlag( unsigned char flag );
//void switchProcess( void );

//new

/*======================================*/
/* 定数設定                             */
/*======================================*/
#define     MENU_DOWN	(0x01 << 0)			/* getSwNow関数のbit0の位置     */
#define     MENU_UP		(0x01 << 1)			/* getSwNow関数のbit1の位置     */
#define     DATA_DOWN	(0x01 << 2)			/* getSwNow関数のbit2の位置     */
#define     DATA_UP		(0x01 << 3)			/* getSwNow関数のbit3の位置     */
#define     SET			(0x09)				/* getSwNow関数のbit7の位置     */

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void initSwitch( void );
unsigned short int getSwNow( void );
unsigned char getSwFlag( unsigned char flag );
void switchProcess( void );