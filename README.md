# BT5_EEG 使用说明
![](https://github.com/gjmsilly/BT5_EEG/blob/master/imgs/CC2640R2F_EEG_v2.png)

## Jumper Settings
- 系统供电可以选择适配器供电/电池供电，对应短接SYS/BATT。
- 板载BQ25895充电管理芯片，支持I2C设置充电模式，监控电池电压。
通过I2C接口跳线帽选择用板载芯片建立I2C通信或使用外接调试器。

## PIN settings
下表列出了CC2640R2F_EEG v2使用的驱动及对应的管脚设置。


| Board Resource           | Pin                                                    |
| :------------------------| :------------------------------------------------------|
| Board_LED0               | DIO0 (Green LED)                                       |
| Board_LED1               | DIO1 (Red LED)                                         |
| Board_I2C0               | DIO4-`SCL`, DIO5-`SDA`                                 |
| Board_SPI1               | DIO24-`MISO`, DIO30-`MOSI`, DIO25-`CLK`, DIO14-`CSN`   |
| Board_UART0              | DIO3-`TX`, DIO2-`RX`                                   |

## Porting Guide
1. 本项目基于TI SimpleLink CC2640R2 SDK (4.20.00.04)开发，移植本项目前确保安装正确版本sdk。
sdk安装至默认路径下`C:\ti\simplelink_cc2640r2_sdk_4_20_00_04`。

![](https://github.com/gjmsilly/BT5_EEG/blob/master/imgs/sdk_version_manage.png)

2. 下载[工程支持包](https://github.com/gjmsilly/BT5_EEG )，进入sdk安装路径，进行如下添加、替换。

| 文件/文件夹名称                 | 添加/替换文件路径                              | 说明                 	          |
| :-------------------------------| :----------------------------------------------|:---------------------------------|
| CC2640R2F_EEG                   | `.\source\ti\ble5stack\boards`                 | 板级支持包                       |
| board.c , board.h               | `.\source\ti\ble5stack\target`                 | 芯片选型                         |
| cc2640r2em                      | `.\source\ti\ble5stack\target`                 | 芯片选型                         |
| BT5_EEG                         | `.\examples\rtos\CC2640R2_LAUNCHXL\ble5stack`  | 应用程序及CCS工程导入配置文件    |
| EEG                             | `.\source\ti\ble5stack\profiles\EEG`           | 蓝牙服务配置文件                 |

3. CCS中导入工程。
   `Project` -> `import CCS projects` -> `.\examples\rtos\CC2640R2_LAUNCHXL\ble5stack\BT5_EEG`
![](https://github.com/gjmsilly/BT5_EEG/blob/master/imgs/import_ccs_project.png) 

4. 添加外设文件`bq25895.c` `bq25895.h`

## 更新日志
- 2020/7/25   v1.0

  - 在TI SDK基础上移植项目BT5_EEG至CC2640R2F_EEG v2板。
  - BT5_EEG 留有一个8bit可读可写特性。
  
- 2020/7/27   v1.1

  - 规范变量名称，service后缀标识服务，BT5_EEG前缀标识应用。
  - 修改特性 batterylevel，支持不定长字节读写，支持notify。
  - 添加bq25895读写程序。
  