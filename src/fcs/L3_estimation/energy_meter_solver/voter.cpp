/**
 * @file voter.cpp
 * @brief 能量机关大小符判别投票器实现
 *
 * 本文件实现了大小符判别投票器的核心逻辑。
 * 通过统计激活装甲板数量投票判别大小符。
 *
 * 算法流程：
 * 1. 每帧观测装甲板数量（1或2个）
 * 2. 根据数量进行投票（大符投big_votes，小符投small_votes）
 * 3. 当总票数达到阈值（8票）或帧数达到上限（20帧）时决策
 * 4. 决策结果：票数多的一方胜出
 */

#include "L3_estimation/energy_meter_solver/voter.hpp"

namespace energy_meter {

namespace {

/// 最小有效票数阈值
/// 当总票数达到此阈值时，立即做出决策
/// 设置为8票可确保在正常帧率下约0.3秒内完成判别
constexpr int VOTE_MIN_VALID  = 8;

/// 最大观察帧数上限
/// 当观察帧数达到此上限时，强制做出决策
/// 设置为20帧可确保在异常情况下也能做出决策（如频繁丢帧）
constexpr int VOTE_MAX_FRAMES = 20;

} // namespace

/**
 * @brief 更新投票状态并尝试做出决策
 *
 * 实现投票逻辑：
 * 1. 若已做出决策，直接返回std::nullopt
 * 2. 统计当前帧的装甲板数量，增加对应票数
 * 3. 检查是否满足决策条件（票数>=8或帧数>=20）
 * 4. 若满足条件，做出决策并返回结果
 *
 * @param target_count 当前观测到的激活装甲板数量
 * @return 若做出决策，返回Decision结构体；否则返回std::nullopt
 */
auto Voter::update(std::size_t target_count) -> std::optional<Decision> {
    // 若已做出决策，不再处理
    if (determined_) {
        return std::nullopt;
    }

    // 统计观察帧数
    ++frames_;

    // 根据装甲板数量进行投票
    // 大符激活时同时亮2个装甲板，小符激活时只亮1个装甲板
    if (target_count == 2) {
        ++big_votes_;
    } else if (target_count == 1) {
        ++small_votes_;
    }
    // 注意：若target_count为其他值（如0），不投票（可能为异常帧）

    // 计算总票数
    const int total_votes = big_votes_ + small_votes_;

    // 检查是否满足决策条件
    // 条件1：总票数达到阈值（8票）
    // 条件2：观察帧数达到上限（20帧）- 强制决策，避免无限等待
    const bool vote_ready = (total_votes >= VOTE_MIN_VALID) || (frames_ >= VOTE_MAX_FRAMES);

    // 若未满足决策条件，继续等待
    if (!vote_ready) {
        return std::nullopt;
    }

    // 做出决策：票数多的一方胜出
    determined_ = true;
    is_big_     = big_votes_ > small_votes_;

    // 返回决策结果
    return decision();
}

/**
 * @brief 重置投票器状态
 *
 * 清空所有统计信息，准备重新判别。
 * 在能量机关重新激活或需要重新判别时调用。
 */
void Voter::reset() {
    determined_  = false;  // 重置决策状态
    is_big_      = true;   // 重置为默认值（大符）
    big_votes_   = 0;      // 清空大符票数
    small_votes_ = 0;      // 清空小符票数
    frames_      = 0;      // 清空帧数统计
}

/**
 * @brief 构造决策结果结构体
 *
 * 将当前投票状态封装为Decision结构体。
 *
 * @return 包含判别结果和统计信息的Decision结构体
 */
auto Voter::decision() const -> Decision {
    return Decision{
        .is_big      = is_big_,      // 判别结果：true=大符，false=小符
        .big_votes   = big_votes_,   // 大符票数
        .small_votes = small_votes_, // 小符票数
        .frames      = frames_,      // 总观察帧数
    };
}

} // namespace energy_meter
