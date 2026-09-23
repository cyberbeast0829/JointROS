# 一个设备端点的描述（ListEndpoints / LookupEndpoint 的输出）。

uint8 ACCESS_READ=1                # 位 0
uint8 ACCESS_WRITE=2               # 位 1

string path                        # 完整路径
uint16 id                          # 端点 id（固件 SDO 用）
uint8 type                         # TYPE_*（见 ParamValue）
uint8 access                       # ACCESS_* 位组合（与 SDK 一致）
bool readable                      # = (access & ACCESS_READ) != 0（方便工具直接用）
bool writable                      # = (access & ACCESS_WRITE) != 0
