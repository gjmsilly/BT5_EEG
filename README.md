# BT5_EEG 使用说明
![](https://github.com/gjmsilly/BT5_EEG/blob/master/imgs/CC2640R2F_EEG_v2.png)

## Jumper Settings
- 系统供电可以选择适配器供电/电池供电，对应短接SYS/BATT。
- 板载BQ25895充电管理芯片，支持I2C设置充电模式，监控电池电压。
通过I2C接口跳线帽选择用CC2640R2F或使用外接调试器建立通信。

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

2. 下载[工程支持包](https://github.com/gjmsilly/BT5_EEG )，***工程支持包与sdk的文件结构一致，只需对相同相对路径的文件、文件夹进行替换即可***。

| 文件/文件夹名称                 | 添加/替换文件相对路径                          | 说明                 	          |
| :-------------------------------| :----------------------------------------------|:---------------------------------|
| CC2640R2F_EEG                   | `.\source\ti\ble5stack\boards`                 | 板级支持包                       |
| board.c , board.h               | `.\source\ti\ble5stack\target`                 | 芯片配置                         |
| cc2640r2em                      | `.\source\ti\ble5stack\target`                 | 芯片配置                         |
| EEG                             | `.\source\ti\ble5stack\profiles`               | 蓝牙服务：EEGservice             |
| uart_printf.c ， uart_printf.h  | `·\source\ti\bles5tack\hal\src\target\_common` | printf重定向至串口配置文件       |
| BT5_EEG                         | `.\examples\rtos\CC2640R2_LAUNCHXL\ble5stack`  | 应用程序及CCS工程导入配置文件    |
| ble_user_config.h               | `.\source\ti\bles5tack\icall\inc`              | BLE HCI PDU配置                  |

3. CCS中导入工程
   `Project` -> `import CCS projects` -> `.\examples\rtos\CC2640R2_LAUNCHXL\ble5stack\BT5_EEG`
   
![](https://github.com/gjmsilly/BT5_EEG/blob/master/imgs/import_ccs_project.png) 

4. 本工程支持shell调试模式，如需使用shell，在`properties` -> `ARM Compiler` -> `Predefined Symbols` 中添加预定义符`SHELL_MODE`，并把工程支持包中的shell文件夹拖至工程目录。 
![](https://github.com/gjmsilly/BT5_EEG/blob/master/imgs/shell_mode.png) 

若启用shell模式，需要增大堆栈，建议将堆栈选项设置为空以让系统自动分配足够堆栈。
![](https://github.com/gjmsilly/BT5_EEG/blob/master/imgs/allocate_heap.png) 

## 基本功能说明

- 不支持功能

  - 本工程基于TI SimpleLink CC2640R2 SDK的simple Peripheral开发，并对工程做了最小剪裁，不支持RSSI监测、PHY自动更新等功能。

- 外设调试相关

  - 支持shell调试板载BQ25895及接插件ads1299，需通过宏定义启动该shell调试模式。

- 基本功能说明

  - 上电默认ads1299采样内部测试信号，采样频率250Hz。
  - 本工程含一个蓝牙服务EEGservice，该服务包含两个特性，详见`BT5_EEG\source\ti\bles5tack\profiles\EEG\EEG_attributes.xls`。对特性2写值即对ads1299发送指令。当ads1299处于连续采样模式并开始采集时，开启特性1的notify，即可收到实时采样数据。
  - 单片ads1299支持最大8通道采样，每通道采样数据长3字节，也即每完成一次采样，采样数据包长24字节。另加状态数据3字节和1字节回环数据，数据包总长28字节。每完成4次采样，数据通过蓝牙notify发出，即notify每次数据包长112字节。***为支持该数据包长度，主机（手机）在建立蓝牙连接后需要将MTU修改至112以上***。
  - 本工程设置为数据“即采即发”：在250Hz采样频率，每4包蓝牙发送的情况下，每16ms会发起一次蓝牙数据收发。***请注意部分主机并不支持该频率的收发，因而会导致无法建立连接，该情况并非工程本身问题。***


## 更新日志
- 2020/7/25   v1.0

  - 在TI SDK基础上移植项目BT5_EEG至CC2640R2F_EEG v2板。
  - BT5_EEG 留有一个8bit可读可写特性。
  
- 2020/7/27   v1.1

  - 规范变量名称，service后缀标识服务，BT5_EEG前缀标识应用。
  - 修改特性 batterylevel，支持不定长字节读写，支持notify。
  - 添加bq25895读写程序。

- 2020/7/29   v1.2

  - 添加shell任务线程，修改BT5_EEG线程优先级至shell线程之上。
  - 禁用display中间件，采用uart写函数输出，关闭所有调试信息的输出。
  
- 2020/8/1   v1.3

  - xdc tool 添加system_printf并重定向至串口，设置在Idle线程时输出调试信息。
  - 优化CCS工程导入配置文件，一步导入工程。
  - 添加SHELL_MODE模式，通过预定义符选择shell线程的创建与否。

- 2020/8/4   v1.4  
  
  - 进一步裁剪应用程序，禁用GAP BOND MGR/SNV/长广播。
  
- 2020/8/6   v1.5  
  
  - 添加ads1299通信程序，支持蓝牙服务控制寄存器读写。
  
- 2020/8/14   v1.52  
  
  - 完善ads1299功能，支持蓝牙控制ads1299及数据实时收发。
  - 进一步剪裁应用程序，删除RSSI监控及连接参数更新。

## References
- [BLE5-Stack User’s Guide](http://dev.ti.com/tirex/explore/content/simplelink_cc2640r2_sdk_3_20_00_21/docs/ble5stack/ble_user_guide/html/ble-stack-5.x-guide/index-cc2640.html#stack-user-s-guide)
- [Adding basic printf over uart with TI-RTOS](https://processors.wiki.ti.com/index.php/CC26xx_Adding_basic_printf_over_uart_with_TI-RTOS)
- [裁剪例程](https://e2echina.ti.com/question_answer/wireless_connectivity/bluetooth/f/103/t/189813?tisearch=e2e-sitesearch&keymatch=ble%20%E8%87%AA%E5%8A%A8%E6%96%AD)