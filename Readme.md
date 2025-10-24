#### 截至2025.10.24进度，详见进度汇报.pdf
#### 通信模型和状态机切换，Model.png为图例
#### 测距原理和Debug日志，位于mydriver文件夹下的deca_instance.c的顶部

#### 重要文件结构
> ├─.vscode
> ├─Core
> │  └─Src
> │     ├─main.c        主函数
> │     └─tim.c         定时器外部中断用于休眠和计时
> ├─decadriver
> ├─Drivers
> ├─MDK-ARM
> ├─mydriver
> │  ├─atcmd.c/.h       AT指令处理相关
> │  ├─instance.h       deca_instance.c的头文件，主要的结构体和宏的声明
> │  └─deca_instance.c  任务函数所在的源文件
> └─STMFLASH            flash相关