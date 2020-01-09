#  1 **支持的指令** <br>
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
<br>
#  2 **协议格式**  <br>
### 1 <br>
FrameStart(1byte) + Cmd(1)  +  lenth(2) + sequence(1) + data(n) + CheckSum(1)  <br>
0xFE + CMD + lenth(地位在前高位在后) + sequence(帧序号)  + data(可有可无)  <br>

数据长度： lenth = cmd(1) + lenth(2) +  sequence(1) + n = 4 + n  <br>
CheckSum: 是从cmd 开始到 data结束的一字节校验和 <br>
<br>
### 2  具体说明 <br>
 **ResetCmd，EraseFullChipCmd 和GetChipIDCmd 没有数据部分就是单纯的执行** <br>
 FE + 0x01 + 0x04  0x00 + 0x01 + CheckSum  (没有数据部分所以n = 0)<br>
 **写flah格式**<br>
 数据部分的格式 W_address(4) + W_size(2) + W_data(x) <br>
 往地址0x0080 写 1024 字节如下 <br>
 0xFE  03 00 00 01 80 00 00 00 00 04 + DATA(1024) + CheckSum <br>
 **写flah应答格式格式**<br>
  数据部分的格式: status(1)
 
 **读flah格式**<br>
 数据部分的格式 R_address(4) + R_size(2) <br>
 **读flah应答格式**<br>
 数据部分的格式 status(1) + R_address(4) + R_size(2)+ DATA(size)  <br>
 status(1): 0 成功， 1 失败. 失败的话后面的读取内容是没有的 <br>
 
 **读ChipID 应答格式** <br>
 数据部分的格式 status(1) + ChipID(1) + Version(1)  <br>
 CC2530 ChipID = 0xA5  version 忽略<br>
	

