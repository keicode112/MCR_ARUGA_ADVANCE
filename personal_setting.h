// ���s�p�����[�^
#define STREAT_JUDGE_ANGLE 40

#define CORNER_SPEED 65         // ���g�p
#define CORNER_PWM 100          // �R�[�i�[�E�o���̍ō�PWM
#define CORNER_FREEPWM_ANGLE 70 // �R�[�i�[�@���[�^�t���[�i�p�[�V�����j����p�x

#define CORNER_START_POWER 70 // �R�[�i�[�@���[�^�t���[�i�p�[�V�����j����p�x

#define SLOPE_UP_SPEED 60   // ��̏�葬�x 60= 4.0m/s
#define SLOPE_DOWN_SPEED 60 // ��̉��葬�x 60= 4.0m/s

// ���[���`�F���W
#define LANE_ANGLE_R 66 // �E���[���A���O��
#define LANE_ANGLE_L 66 // �����[���A���O��

// �N�����N
#define CRANK_ANGLE_R 110 // �E�N�����N�A���O��
#define CRANK_ANGLE_L 110 // ���N�����N�A���O��

// �{�����[���p�x�ϊ�
// #define VR_DEG_CHANGE 33 / 100 // �|��
#define VR_DEG_CHANGE 1 / 5 // �|��

// �{�����[���̃Z���^�[�l
#define VR_CENTER 515

// ��
#define SLOPE_CENTER 290     // ��Z���T�̃Z���^�[�l
#define SLOPE_UP_START 90    // ���J�n����
#define SLOPE_UP_FIN 415     // ���I��蔻��
#define SLOPE_DOWN_START 415 // ����͂��ߔ���
#define SLOPE_DOWN_FIN 96    // ����I��蔻��

// ���[�^IO
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
