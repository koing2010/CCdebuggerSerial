ResetCmd = 0x01
ResetRspCmd = 0x81
ReadDataCmd = 0x02
ReadDataRspCmd = 0x82
WriteFlashCmd = 0x03
WriteFlashRspCmd = 0x83
EraseFullChipCmd = 0x04
EraseFullChipRspCmd = 0x84
GetChipIDCmd = 0x05
GetChipIDCmdRsp = 0x85


FrameStart(1byte) + Cmd(1) + lenth(2) + +data(n) + FSC(1)
0xFE   +   CMD  + lenth(地位在前高位在后) + data(个事可有没有)

数据长度： lenth = cmd(1) + lenth(2) + n = 3 + n

