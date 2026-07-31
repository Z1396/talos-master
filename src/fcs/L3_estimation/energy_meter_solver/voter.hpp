/**
 * @file voter.hpp
 * @brief 能量机关大小符判别投票器
 *
 * 本文件实现了大小符判别投票器，用于自动判断当前能量机关是大符还是小符。
 * 能量机关在激活后会随机呈现大符或小符状态，需要通过观测进行判别。
 *
 * 核心算法原理：
 * 1. 投票机制：通过统计激活装甲板数量进行投票
 *    - 大符激活时同时亮2个装甲板
 *    - 小符激活时只亮1个装甲板
 * 2. 决策逻辑：
 *    - 当有效票数达到阈值（8票）时做出决策
 *    - 或当观察帧数达到上限（20帧）时强制决策
 * 3. 结果：返回决策结果，包含大小符类型和投票统计
 *
 * 关键数据结构：
 * - Voter: 投票器类，管理投票状态和决策逻辑
 * - Decision: 决策结果结构体，包含大小符类型和投票统计
 *
 * 使用流程：
 * 1. 每帧调用update()传入观测到的装甲板数量
 * 2. 当返回std::optional<Decision>有效值时，判别完成
 * 3. 若需重新判别，调用reset()清空状态
 */

#pragma once

#include <cstddef>
#include <optional>

namespace energy_meter {

/**
 * @class Voter
 * @brief 大小符判别投票器
 *
 * 通过统计激活装甲板数量投票判别大小符。
 * 大符激活时同时亮2个装甲板，小符激活时只亮1个装甲板。
 */
class Voter {
public:
    /**
     * @struct Decision
     * @brief 判别决策结果
     *
     * 包含大小符判别结果和投票统计信息。
     */
    struct Decision {
        bool is_big{true};      ///< 是否为大符（true=大符，false=小符）
        int big_votes{0};       ///< 大符票数（观测到2个装甲板的帧数）
        int small_votes{0};     ///< 小符票数（观测到1个装甲板的帧数）
        int frames{0};          ///< 总观察帧数
    };

    /**
     * @brief 更新投票状态并尝试做出决策
     *
     * 每帧调用此函数，传入当前观测到的激活装甲板数量。
     * - 若观测到2个装甲板，则投大符票
     * - 若观测到1个装甲板，则投小符票
     * - 当有效票数达到阈值（8票）或观察帧数达到上限（20帧）时做出决策
     *
     * @param target_count 当前观测到的激活装甲板数量（1或2）
     * @return 若已做出决策，返回Decision结构体；否则返回std::nullopt
     *
     * @note 一旦做出决策，后续调用将返回std::nullopt，直到调用reset()
     *
     * @warning 若观测帧数达到上限（20帧）但票数不足，将强制决策
     */
    auto update(std::size_t target_count) -> std::optional<Decision>;

    /**
     * @brief 重置投票器状态
     *
     * 清空所有投票统计，准备重新判别。
     * 在能量机关重新激活时调用。
     */
    void reset();

    /**
     * @brief 查询是否已做出决策
     *
     * @return 若已做出决策返回true，否则返回false
     */
    [[nodiscard]] auto determined() const -> bool { return determined_; }

    /**
     * @brief 查询当前判别结果
     *
     * @return 若已做出决策，返回是否为大符；否则默认返回true
     *
     * @warning 在未做出决策时调用可能返回不准确的结果
     */
    [[nodiscard]] auto is_big() const -> bool { return is_big_; }

private:
    /**
     * @brief 构造决策结果结构体
     *
     * @return 包含当前判别结果的Decision结构体
     */
    [[nodiscard]] auto decision() const -> Decision;

    bool determined_{false};  ///< 是否已做出决策
    bool is_big_{true};       ///< 当前判别结果（true=大符，false=小符）
    int big_votes_{0};        ///< 大符票数
    int small_votes_{0};      ///< 小符票数
    int frames_{0};           ///< 总观察帧数
};

} // namespace energy_meter
