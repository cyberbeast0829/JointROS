/**
 * @file    jr_desc_cache.hpp
 * @brief   描述符缓存（路线 B：缓存**原始 JSON**）—— DESIGN ADR-10
 *
 * @par 为什么是路线 B（原始 JSON）而不是"已解析结果"
 *  B 的失效键只有 `(fw_version, crc)` 两个：改 filter、升级 SDK 都**不用**重新下载。
 *  路线 A（`desc_export/import` 的紧凑格式）还把 `retain/filter_paths/SDK 内部格式`
 *  三者绑在一起 —— 现场最容易变成"缓存不命中但不报错"。
 *
 * @par 一个必须讲清楚的限制（不吹牛）
 *  设备侧的 `VersionCRC` 只有**下载过程**才知道。因此启动时我们能校验的是：
 *   ① 缓存文件的完整性（本文件头里的 payload CRC32）；
 *   ② 设备 `fw_version`（`QUERY_DEVICE_INFO` 0x46，便宜）是否与缓存头一致。
 *  想**严格**校验"缓存内容 == 当前设备描述符"，只有真下载一遍再比对（`verify_on_start`）。
 *  这条已写进 `DescCfg` 的注释，不在文档里含糊过去。
 */

#ifndef JR_ROS2_RT_JR_DESC_CACHE_HPP
#define JR_ROS2_RT_JR_DESC_CACHE_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace jr {

enum class CacheOutcome { kHit, kMiss, kError };

struct DescCacheMeta {
    std::uint32_t fw_version = 0u;
    std::uint16_t crc = 0u;
    std::uint32_t json_len = 0u;
};

class DescCache {
public:
    /** 默认缓存目录：`<XDG_CACHE_HOME|~/.cache|%LOCALAPPDATA%>/jr`。 */
    static void default_dir(char *out, std::size_t cap) noexcept;

    /**
     * 读取缓存。
     * @param dir        缓存目录（空 = 默认）
     * @param key        "总线名" 或设备序列号等（会被安全化）
     * @param expect_fw  设备当前 fw_version（0 = 不校验）
     * @param json_out   输出：原始 JSON 字节
     */
    CacheOutcome load(const char *dir, const char *key, std::uint32_t expect_fw,
                      std::vector<std::uint8_t> *json_out, DescCacheMeta *meta_out, char *msg,
                      std::size_t msg_len) const noexcept;

    /**
     * 写缓存。**只有 `complete == true` 时才允许调用**（DESIGN §10.2 的证伪用例：
     * 下载被中断必须拒绝落盘，否则下次上电就用一份坏缓存启动）。
     */
    CacheOutcome save(const char *dir, const char *key, std::uint32_t fw_version,
                      std::uint16_t crc, const void *json, std::size_t len, char *msg,
                      std::size_t msg_len) noexcept;

    /** 删除缓存（诊断/产线用）。 */
    CacheOutcome remove(const char *dir, const char *key, char *msg, std::size_t msg_len) noexcept;

    /** 缓存文件路径（供诊断显示）。 */
    static void path_of(const char *dir, const char *key, char *out, std::size_t cap) noexcept;

    /** 标准 CRC-32（IEEE），用于缓存完整性校验。 */
    static std::uint32_t crc32(const void *data, std::size_t len) noexcept;
};

}  // namespace jr

#endif /* JR_ROS2_RT_JR_DESC_CACHE_HPP */
