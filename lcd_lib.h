/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void initLcd( void );
void clr_lcd(void);         /* 液晶のクリア */

void lcdShowProcess( void );
int lcdPrintf(char far *format, ...);
void lcdPosition(char x ,char y);