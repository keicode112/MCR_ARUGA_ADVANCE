/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void initLcd( void );
void clr_lcd(void);         /* �t���̃N���A */

void lcdShowProcess( void );
int lcdPrintf(char far *format, ...);
void lcdPosition(char x ,char y);