# 设备身份信息（QUERY_DEVICE_INFO 0x46）。

uint32 hw_version
uint32 fw_version
uint64 serial
bool classic                       # true = Classic 链路（非 CAN-FD）
bool valid                         # 本次查询是否成功（false 时其余字段无意义）
