# 描述符（设备端点表）的状态与来源。ADR-10 路线 B：缓存的就是**原始 JSON**。
#
# 诚实声明：设备侧的 VersionCRC 只有**下载过程**才知道，所以启动时我们能校验的是
#   ① 缓存文件自身完整性（本包写的头里带 payload CRC32）；
#   ② 设备 fw_version 是否与缓存头一致。
# 想严格校验"缓存内容 == 当前设备描述符"，只能真下载一遍再比对。

string bus

uint32 total_len                   # 设备侧描述符总长
uint32 json_bytes                  # 我们捕获到的原始 JSON 字节数
uint16 crc                         # 设备给的 VersionCRC（未下载时为缓存里的值）
uint32 fw_version
uint32 endpoint_count              # 设备声明的端点数
uint32 parsed_total                # SDK 成功解析出的端点数
uint32 frames_rx                   # 本次下载接收的帧数

bool complete                      # 只有 complete 才允许落缓存（§10.2：中断不落盘）
bool shared_hit                    # 复用了 SDK 内部已缓存的描述符
bool from_cache                    # 本次来自本地缓存（而非从设备下载）

string cache_path                  # 本地缓存文件路径（诊断/排障用）
