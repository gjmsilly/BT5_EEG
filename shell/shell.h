/**
 * @file shell.h
 * @author Letter (NevermindZZT@gmail.com)
 * @brief letter shell
 * @version 2.0.0
 * @date 2018-12-29
 * 
 * @Copyright (c) 2018 Letter
 * 
 */

#ifndef     __SHELL_H__
#define     __SHELL_H__

#include "shell_cfg.h"
#include <stdint.h>

#if SHELL_USING_AUTH == 1
    #if !defined(SHELL_USER_PASSWORD)
        #error "please config shell user password (int shell_cfg.h) "
    #endif  
#endif      

#define     SHELL_VERSION               "2.0.8"                 /**< 鐗堟湰鍙� */

/**
 * @brief shell閿�煎畾涔�
 * 
 */
#define     SHELL_KEY_LF                0x0A
#define     SHELL_KEY_CR                0x0D
#define     SHELL_KEY_TAB               0x09
#define     SHELL_KEY_BACKSPACE         0x08
#define     SHELL_KEY_DELETE            0x7F
#define     SHELL_KEY_ESC               0x1B

#define     SHELL_KEY_CTRL_T            0x14
#define     SHELL_KEY_CTRL_A            0x01
#define     SHELL_KEY_CTRL_B            0x02
#define     SHELL_KEY_CTRL_C            0x03
#define     SHELL_KEY_CTRL_D            0x04
#define     SHELL_KEY_CTRL_E            0x05
#define     SHELL_KEY_CTRL_F            0x06
#define     SHELL_KEY_CTRL_G            0x07
#define     SHELL_KEY_CTRL_H            0x08
#define     SHELL_KEY_CTRL_I            0x09
#define     SHELL_KEY_CTRL_J            0x0A
#define     SHELL_KEY_CTRL_K            0x0B
#define     SHELL_KEY_CTRL_L            0x0C
#define     SHELL_KEY_CTRL_M            0x0D
#define     SHELL_KEY_CTRL_N            0x0E
#define     SHELL_KEY_CTRL_O            0x0F
#define     SHELL_KEY_CTRL_P            0x10
#define     SHELL_KEY_CTRL_Q            0x11
#define     SHELL_KEY_CTRL_R            0x12
#define     SHELL_KEY_CTRL_S            0x13
#define     SHELL_KEY_CTRL_T            0x14
#define     SHELL_KEY_CTRL_U            0x15
#define     SHELL_KEY_CTRL_V            0x16
#define     SHELL_KEY_CTRL_W            0x17
#define     SHELL_KEY_CTRL_X            0x18
#define     SHELL_KEY_CTRL_Y            0x19
#define     SHELL_KEY_CTRL_Z            0x1A

/**
 * @brief shell鍙橀噺绫诲瀷瀹氫箟
 * 
 */
#define     SHELL_VAR_INT               0
#define     SHELL_VAR_SHORT             1
#define     SHELL_VAR_CHAR              2
#define     SHELL_VAR_POINTER           3
#define     SHELL_VAL                   4

#if defined(__CC_ARM) || (defined(__ARMCC_VERSION) && __ARMCC_VERSION >= 6000000)
    #define SECTION(x)                  __attribute__((section(x)))
#elif defined(__ICCARM__)
    #define SECTION(x)                  @ x
#elif defined(__GNUC__)
    #define SECTION(x)                  __attribute__((section(x)))
#else
    #define SECTION(x)
#endif

/**
 * @brief shell鍛戒护瀵煎嚭
 * 
 * @attention 鍛戒护瀵煎嚭鏂瑰紡鏀寔keil,iar鐨勭紪璇戝櫒浠ュ強gcc锛屽叿浣撳弬鑰價eadme
 */
#if SHELL_USING_CMD_EXPORT == 1
#if SHELL_LONG_HELP == 1
#define     SHELL_EXPORT_CMD(cmd, func, desc)                               \
            const char shellCmd##cmd[] = #cmd;                              \
            const char shellDesc##cmd[] = #desc;                            \
            const SHELL_CommandTypeDef                                      \
            shellCommand##cmd SECTION("shellCommand") =                     \
            {                                                               \
                shellCmd##cmd,                                              \
                (int (*)())func,                                            \
                shellDesc##cmd,                                             \
                (void *)0                                                   \
            }
#define     SHELL_EXPORT_CMD_EX(cmd, func, desc, help)                      \
            const char shellCmd##cmd[] = #cmd;                              \
            const char shellDesc##cmd[] = #desc;                            \
            const char shellHelp##cmd[] = #help;                            \
            const SHELL_CommandTypeDef                                      \
            shellCommand##cmd SECTION("shellCommand") =                     \
            {                                                               \
                shellCmd##cmd,                                              \
                (int (*)())func,                                            \
                shellDesc##cmd,                                             \
                shellHelp##cmd                                              \
            }
#else /** SHELL_LONG_HELP == 1 */
#define     SHELL_EXPORT_CMD(cmd, func, desc)                               \
            const char shellCmd##cmd[] = #cmd;                              \
            const char shellDesc##cmd[] = #desc;                            \
            const SHELL_CommandTypeDef                                      \
            shellCommand##cmd SECTION("shellCommand") =                     \
            {                                                               \
                shellCmd##cmd,                                              \
                (int (*)())func,                                            \
                shellDesc##cmd                                              \
            }
#define     SHELL_EXPORT_CMD_EX(cmd, func, desc, help)                      \
            const char shellCmd##cmd[] = #cmd;                              \
            const char shellDesc##cmd[] = #desc;                            \
            const SHELL_CommandTypeDef                                      \
            shellCommand##cmd SECTION("shellCommand") =                     \
            {                                                               \
                shellCmd##cmd,                                              \
                (int (*)())func,                                            \
                shellDesc##cmd                                              \
            }
#endif /** SHELL_LONG_HELP == 1 */

#if SHELL_USING_VAR == 1
    #define SHELL_EXPORT_VAR(var, variable, desc, type)                     \
            const char shellVar##var[] = #var;                              \
            const char shellDesc##var[] = #desc;                            \
            const SHELL_VaribaleTypeDef                                     \
            shellVariable##var SECTION("shellVariable") =                   \
            {                                                               \
                shellVar##var,                                              \
                (void *)(variable),                                         \
                shellDesc##var,                                             \
                type                                                        \
            }
#else 
    #define SHELL_EXPORT_VAR(var, variable, desc, type) 
#endif /** SHELL_USING_VAR == 1 */

#else
#define     SHELL_EXPORT_CMD(cmd, func, desc)
#define     SHELL_EXPORT_CMD_EX(cmd, func, desc, help)
#define     SHELL_EXPORT_VAR(var, variable, desc, type) 
#endif /** SHELL_USING_CMD_EXPORT == 1 */

#define     SHELL_EXPORT_VAR_INT(var, variable, desc)                       \
            SHELL_EXPORT_VAR(var, &variable, desc, SHELL_VAR_INT)
#define     SHELL_EXPORT_VAR_SHORT(var, variable, desc)                     \
            SHELL_EXPORT_VAR(var, &variable, desc, SHELL_VAR_SHORT)
#define     SHELL_EXPORT_VAR_CHAR(var, variable, desc)                      \
            SHELL_EXPORT_VAR(var, &variable, desc, SHELL_VAR_CHAR)
#define     SHELL_EXPORT_VAR_POINTER(var, variable, desc)                   \
            SHELL_EXPORT_VAR(var, variable, desc, SHELL_VAR_POINTER)
#define     SHELL_EXPORT_VAL(val, value, desc)                              \
            SHELL_EXPORT_VAR(val, value, desc, SHELL_VAL)


/**
 * @brief shell鍛戒护鏉＄洰
 * 
 * @note 鐢ㄤ簬shell鍛戒护閫氳繃鍛戒护琛ㄧ殑鏂瑰紡瀹氫箟
 */
#if SHELL_USING_CMD_EXPORT == 0
#if SHELL_LONG_HELP == 1
#define     SHELL_CMD_ITEM(cmd, func, desc)                                 \
            {                                                               \
                #cmd,                                                       \
                (int (*)())func,                                            \
                #desc,                                                      \
                (void *)0                                                   \
            }
#define     SHELL_CMD_ITEM_EX(cmd, func, desc, help)                        \
            {                                                               \
                #cmd,                                                       \
                (int (*)())func,                                            \
                #desc,                                                      \
                #help                                                       \
            }   
#else /** SHELL_LONG_HELP == 1 */
#define     SHELL_CMD_ITEM(cmd, func, desc)                                 \
            {                                                               \
                #cmd,                                                       \
                (int (*)())func,                                            \
                #desc                                                       \
            }
#define     SHELL_CMD_ITEM_EX(cmd, func, desc, help)                        \
            {                                                               \
                #cmd,                                                       \
                (int (*)())func,                                            \
                #desc,                                                      \
            }  
#endif /** SHELL_LONG_HELP == 1 */

#define     SHELL_VAR_ITEM(var, variable, desc, type)                       \
            {                                                               \
                #var,                                                       \
                varialbe,                                                   \
                #desc,                                                      \
                type,                                                       \
            }
#define     SHELL_VAR_ITEM_INT(var, variable, desc)                         \
            SHELL_VAR_ITEM(var, &variable, desc, SHELL_VAR_INT)
#define     SHELL_VAR_ITEM_SHORT(var, variable, desc)                       \
            SHELL_VAR_ITEM(var, &variable, desc, SHELL_VAR_SHORT)
#define     SHELL_VAR_ITEM_CHAR(var, variable, desc)                        \
            SHELL_VAR_ITEM(var, &variable, desc, SHELL_VAR_CHAR)
#define     SHELL_VAR_ITEM_POINTER(var, variable, desc)                     \
            SHELL_VAR_ITEM(var, variable, desc, SHELL_VAR_POINTER)

#endif /** SHELL_USING_CMD_EXPORT == 0 */

/**
 * @brief shell璇诲彇鏁版嵁鍑芥暟鍘熷瀷
 * 
 * @param char shell璇诲彇鐨勫瓧绗�
 * 
 * @return char 0 璇诲彇鏁版嵁鎴愬姛
 * @return char -1 璇诲彇鏁版嵁澶辫触
 */
typedef signed char (*shellRead)(char *);

/**
 * @brief shell鍐欐暟鎹嚱鏁板師鍨�
 * 
 * @param const char 闇�鍐欑殑瀛楃
 */
typedef void (*shellWrite)(const char);

/**
 * @brief shell鎸囦护鎵ц鍑芥暟鍘熷瀷
 * 
 */
typedef int (*shellFunction)();


/**
 * @brief shell杈撳叆鐘舵��
 * 
 */
typedef enum
{
    SHELL_IN_NORMAL = 0,
    SHELL_ANSI_ESC,
    SHELL_ANSI_CSI,
}SHELL_InputMode;


/**
 * @brief shell 鍛戒护瀹氫箟
 * 
 */
typedef struct
{
    const char *name;                                           /**< shell鍛戒护鍚嶇О */
    shellFunction function;                                     /**< shell鍛戒护鍑芥暟 */
    const char *desc;                                           /**< shell鍛戒护鎻忚堪 */
#if SHELL_LONG_HELP == 1
    const char *help;                                           /**< shell闀垮府鍔╀俊鎭� */
#endif
}SHELL_CommandTypeDef;


#if SHELL_USING_VAR == 1
/**
 * @brief shell 鍙橀噺瀹氫箟
 * 
 */
typedef struct
{
    const char *name;                                           /**< shell鍙橀噺鍚嶇О */
    const void *value;                                          /**< shell鍙橀噺鍊� */
    const char *desc;                                           /**< shell鍙橀噺鎻忚堪 */
    const int type;                                             /**< shell鍙橀噺绫诲瀷 */
} SHELL_VaribaleTypeDef;
#endif /** SHELL_USING_VAR == 1 */


/**
 * @brief shell瀵硅薄瀹氫箟
 * 
 */
typedef struct
{
    char *command;                                              /**< shell鍛戒护鎻愮ず绗� */
    char buffer[SHELL_COMMAND_MAX_LENGTH];                      /**< shell鍛戒护缂撳啿 */
    unsigned short length;                                      /**< shell鍛戒护闀垮害 */
    unsigned short cursor;                                      /**< shell鍏夋爣浣嶇疆 */
    char *param[SHELL_PARAMETER_MAX_NUMBER];                    /**< shell鍙傛暟 */
    char history[SHELL_HISTORY_MAX_NUMBER][SHELL_COMMAND_MAX_LENGTH];  /**< 鍘嗗彶璁板綍 */
    unsigned short historyCount;                                /**< 鍘嗗彶璁板綍鏁伴噺 */
    short historyFlag;                                          /**< 褰撳墠璁板綍浣嶇疆 */
    short historyOffset;                                        /**< 鍘嗗彶璁板綍鍋忕Щ */
    SHELL_CommandTypeDef *commandBase;                          /**< 鍛戒护琛ㄥ熀鍧� */
    unsigned short commandNumber;                               /**< 鍛戒护鏁伴噺 */
#if SHELL_USING_VAR == 1
    SHELL_VaribaleTypeDef *variableBase;                        /**< 鍙橀噺琛ㄥ熀鍧� */
    unsigned short variableNumber;                              /**< 鍙橀噺鏁伴噺 */
#endif
    int keyFuncBase;                                            /**< 鎸夐敭鍝嶅簲琛ㄥ熀鍧� */
    unsigned short keyFuncNumber;                               /**< 鎸夐敭鍝嶅簲鏁伴噺 */
    struct
    {
        unsigned char inputMode : 2;                            /**< 杈撳叆妯″紡 */
        unsigned char isActive: 1;                              /**< 鏄惁鏄綋鍓嶆椿鍔╯hell */
        unsigned char tabFlag : 1;                              /**< tab鏍囧織 */
        unsigned char authFlag : 1;                             /**< 瀵嗙爜鏍囧織 */
    } status;                                                   /**< shell鐘舵�� */
    shellRead read;                                             /**< shell璇诲瓧绗� */
    shellWrite write;                                           /**< shell鍐欏瓧绗� */
#if SHELL_LONG_HELP == 1 || (SHELL_USING_AUTH && SHELL_LOCK_TIMEOUT > 0)
    int activeTime;                                             /**< shell婵�娲绘椂闂存埑 */
#endif
}SHELL_TypeDef;


/**
 * @brief shell鎸夐敭鍔熻兘瀹氫箟
 * 
 */
typedef struct
{
    unsigned char keyCode;                                      /**< shell鎸夐敭閿�� */
    void (*keyFunction)(SHELL_TypeDef *);                       /**< 鎸夐敭鍝嶅簲鍑芥暟 */
} SHELL_KeyFunctionDef;


void shellInit(SHELL_TypeDef *shell);
void shellSetCommandList(SHELL_TypeDef *shell, SHELL_CommandTypeDef *base, unsigned short size);

#if SHELL_USING_VAR == 1
void shellSetVariableList(SHELL_TypeDef *shell, SHELL_VaribaleTypeDef *base, unsigned short size);
int shellGetVariable(SHELL_TypeDef *shell, char *var);
#endif /** SHELL_USING_VAR == 1 */

void shellSetKeyFuncList(SHELL_TypeDef *shell, SHELL_KeyFunctionDef *base, unsigned short size);
SHELL_TypeDef *shellGetCurrent(void);
void shellPrint(SHELL_TypeDef *shell, char *fmt, ...);
unsigned short shellDisplay(SHELL_TypeDef *shell, const char *string);
void shellHandler(SHELL_TypeDef *shell, char data);
#define     shellInput      shellHandler

void shellHelp(int argc, char *argv[]);
void shellClear(void);

#if SHELL_USING_TASK == 1

#include  <xdc/std.h>
extern SHELL_TypeDef shell;

#define SHELL_TASK_STACK_SIZE   600 // min size 512 for TI-RTOS task
#define SHELL_TASK_PRIORITY     1

void shellTask(UArg arg0, UArg arg1);
void shell_createTask(void);
void User_Shell_Init(void);

#endif

#endif

