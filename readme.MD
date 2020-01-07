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
#  2 **协议格式**  <br>
### 1 <br>
FrameStart(1byte) + Cmd(1)  +  lenth(2) + sequence(1) + data(n) + FSC(1)  <br>
0xFE + CMD + lenth(地位在前高位在后) + sequence(帧序号)  + data(可有可无)  <br>

数据长度： lenth = cmd(1) + lenth(2) +  sequence(1) + n = 4 + n  <br>

### 2  具体说明 <br>
 ResetCmd和GetChipIDCmd 没有数据部分就是单纯的执行 <br>
  <br>
  
# 3 **校验**
### XOR of all the bytes   <br>
def calcFCS(pMsg):   <br>
	result = 0  <br>
	for i in range(len(pMsg)):  <br>
		result = result^pMsg[i]  <br>
	return result & 0xFF  <br>

