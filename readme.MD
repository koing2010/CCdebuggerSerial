#  一 **支持的指令** <br>
ResetCmd = 0x01 <br>
ResetRspCmd = 0x81 <br>
ReadDataCmd = 0x02 <br>
ReadDataRspCmd = 0x82 <br>
WriteFlashCmd = 0x03 <br>
WriteFlashRspCmd = 0x83 <br>
EraseFullChipCmd = 0x04 <br>
EraseFullChipRspCmd = 0x84 <br>
GetChipIDCmd = 0x05 <br>
GetChipIDCmdRsp = 0x85 <br>
DebugCmd = 0x08 <br>
DebugRspCmd = 0x88 <br>
<br>
#  二 **协议格式**  <br>
## 1 **格式Frame**<br>
FrameStart(1byte) + Cmd(1)  +  lenth(2) + sequence(1) + data(n) + CheckSum(1)  <br>
0xFE + CMD + lenth(地位在前高位在后) + sequence(帧序号)  + data(可有可无)  <br>

数据长度： lenth = cmd(1) + lenth(2) +  sequence(1) + n = 4 + n  <br>
CheckSum: 是从cmd 开始到 data结束的一字节校验和 <br>
<br>
## 2  具体说明 <br>
### 2.1 **ResetCmd，EraseFullChipCmd 和GetChipIDCmd 没有数据部分就是单纯的执行** <br>
 FE + 0x01 + 0x04  0x00 + 0x01 + CheckSum  (没有数据部分所以n = 0)<br>
### 2.2  **写flah格式**<br>
 数据部分的格式 W_address(4) + W_size(2) + W_data(x) <br>
 往地址0x0080 写 1024 字节如下 <br>
 0xFE  03 00 00 01 80 00 00 00 00 04 + DATA(1024) + CheckSum <br>
### 2.3 **写flah应答格式格式**<br>
  数据部分的格式: status(1)<br>
 
### 2.4 **读flah格式**<br>
 数据部分的格式 R_address(4) + R_size(2) <br>
 
### 2.5  **读flah应答格式**<br>
 数据部分的格式 status(1) + R_address(4) + R_size(2)+ DATA(size)  <br>
 status(1): 0 成功， 1 失败. 失败的话后面的读取内容是没有的 <br>
 
### 2.6  **调试指令** <br> 
数据部分的格式： CMD(1) + bytes(1-3) <br>
CMD 是写入的调试指令 <br>
numbyte(1)  指令的操作数据的个数<br>
 bytes(1-3)  指令的操作数据，个数是1~3个<br> <br>
应答格式: status(1) + byte(1) <br>
调试指令的应答是，有返回数据的指令，则返回应答指令执行结果。无返回数据的指令，可以忽略这一位的数据  <br>
 
 
 
 ### 2.7   **读ChipID 应答格式** <br>
 数据部分的格式 status(1) + ChipID(1) + Version(1)  <br>
 CC2530 ChipID = 0xA5  version 忽略<br><br>
 
# 三 **各个指令使用注意事项**<br>
## 1 **使用时序**<br>
连接待变成设备(DUP),先调用 RESET 指令，让模块进入debug模式，读取chipID,确定芯片型号<br>
<br>
## 2 **功能测试**<br>
 CM 单片机调试接口指令  <br>
0x55一字节调试指令 0x56 两字节调试指令  0x57三字节调试指令 <br>
### 2.1 IO 功能测试<br>
指令格式： DebugCmd + CMD(1)+ bytes(1-3)<br>
 IO 方向控制寄存器地址  <br>
 P0DIR:  0xFD    <br> P1DIR:  0xFE  <br>   P2DIR:  0xFF <br>   

 IO 输入输出寄存器地址  <br>
 P0:  0x80    <br> P1:  0x90  <br>   P2:  0xA0 <br>  

例: 把P0DIR全部设置为输出，P0_IO 全部输出1. 指令格式如下<br>

 #### a. 设置方向： <br>
 DebugCmd + 0x57(cmd) + 0x75(MOV direct,#data ) + 0xFD(direct) + 0xFF(data) <br>
0xFD对应 P0DIR地址， 0xFF是设置参数 ,返回值可以忽略 <br>

#### b. 设置IO输出： <br> 
DebugCmd + 0x57(cmd) + 0x75(MOV direct,#data ) + 0x80(direct) + 0xFF(data) <br>
0x80对应 P0_IO的地址， 0xFF是设置参数 ,返回值可以忽略 <br>

#### c. 将a中 P0DIR 地址写入0x00,则P0设置为输入模式 <br>

#### d. 读取IO输出： <br>
DebugCmd + 0x56(cmd) + 0xE5(MOV A,direct ) + 0x80(direct) <br>
0x80对应 P0_IO的地址， 返回值就是当前IO输入的状态 <br>

 
<br><br>
