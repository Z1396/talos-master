/**
 * @file data_associator.hpp
 * @brief 基于Sigma点门控的装甲板数据关联器
 *
 * 本文件实现了鲁棒的观测-航迹关联算法，用于将装甲板检测与跟踪目标匹配。
 *
 * 核心算法原理：
 * - **Sigma点变换**：使用UT（Unscented Transform）传播状态不确定性到测量空间
 * - **马氏距离门控**：基于预测测量与实际观测的马氏距离进行筛选
 * - **贪婪分配**：使用贪婪算法求解一对一匹配，避免匈牙利算法的计算开销
 * - **先验概率融合**：支持装甲板ID先验概率，提升匹配鲁棒性
 *
 * 关键技术细节：
 * - **UT变换**：比线性化方法（EKF）更精确，尤其适用于强非线性测量模型
 * - **角度展开**：使用unwrap_rad处理角度跳变，保证Sigma点连续性
 * - **LDLT分解**：使用Cholesky分解求解马氏距离，数值稳定性优于直接求逆
 * - **可见性判断**：根据装甲板朝向过滤不可见装甲板，减少误匹配
 *
 * 性能优化：
 * - **预计算缓存**：批量计算Sigma点和测量协方差，避免重复计算
 * - **提前终止**：观测为空时立即返回，避免无效计算
 * - **对角测量噪声**：假设测量噪声为对角矩阵，简化LDLT分解
 *
 * 应用场景：
 * - 多目标跟踪中的观测分配
 * - 装甲板ID识别与匹配
 * - 机器人跟踪中的数据关联
 *
 * 设计决策：
 * - 使用贪婪算法而非匈牙利算法：降低计算复杂度（O(n²) vs O(n³)）
 * - 使用LDLT而非LLT：数值稳定性更好，无需假设正定矩阵
 * - 预计算可见性：减少无效Sigma点计算
 */

#pragma once

#include "new_motion_model.hpp"
#include "util.hpp"

#include "core/types.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace fcs::L3 {

/**
 * @brief 模板化数据关联器（支持任意运动模型）
 *
 * 使用Sigma点门控进行鲁棒的观测-航迹关联。
 *
 * @tparam Model 运动模型类型（需满足MotionModel concept）
 */
template <MotionModel Model>
class DataAssociator {
public:
    static constexpr int NZ = Model::NZ;         ///< 测量维度
    static constexpr int NX = Model::NX;         ///< 状态维度

    using VecZ  = typename Model::VecZ;          ///< 测量向量类型
    using VecX  = Eigen::Matrix<double, NX, 1>;  ///< 状态向量类型
    using MatXX = Eigen::Matrix<double, NX, NX>; ///< 状态协方差矩阵类型
    using MatZ  = Eigen::Matrix<double, NZ, NZ>; ///< 测量协方差矩阵类型

    /**
     * @brief 数据关联结果
     *
     * 存储匹配成功的装甲板观测及其对应的装甲板ID和代价。
     */
    struct MatchResult {
        std::vector<ArmorMeasurement> armors; ///< 匹配成功的装甲板观测
        std::vector<int> armor_ids;           ///< 对应的装甲板ID（0-3）
        std::vector<double> costs;            ///< 匹配代价（马氏距离²）
    };

    /**
     * @brief 执行观测-航迹关联（简化接口，无先验）
     *
     * @param observations 批量装甲板观测
     * @param x_pred 预测状态向量
     * @param Sx_pred 预测协方差平方根（Cholesky因子）
     * @param model 运动模型实例
     * @param target_name 目标装甲板名称（如"hero", "sentry"）
     * @param gate_threshold 马氏距离门控阈值（通常取9-16）
     * @param armors_num 目标装甲板数量（通常为4）
     * @return 匹配结果（包含匹配的装甲板、ID和代价）
     *
     * @see match(const ArmorMeasurementBatch&, const VecX&, const MatXX&, Model&, ArmorName,
     * double, int, const std::vector<double>*)
     */
    [[nodiscard]] MatchResult match(
        const ArmorMeasurementBatch& observations, const VecX& x_pred, const MatXX& Sx_pred,
        Model& model, ArmorName target_name, double gate_threshold, int armors_num) const noexcept {
        return match(
            observations, x_pred, Sx_pred, model, target_name, gate_threshold, armors_num, nullptr);
    }

    /**
     * @brief 执行观测-航迹关联（完整接口，支持先验）
     *
     * 核心算法流程：
     * 1. **预处理观测**：转换为球坐标测量，过滤不匹配目标名称的观测
     * 2. **Sigma点生成**：对每个装甲板ID生成2*NX个Sigma点
     * 3. **测量预测**：传播Sigma点到测量空间，计算预测测量和协方差
     * 4. **代价计算**：计算每个（观测，装甲板ID）对的马氏距离
     * 5. **贪婪分配**：选择代价最小的有效匹配，保证一对一约束
     *
     * 关键步骤详解：
     *
     * **Step 1: 观测预处理**
     * - 将Cartesian坐标（x,y,z）转换为球坐标（yaw, pitch, distance）
     * - 对距离取对数：log(distance)，避免距离对角度的数值影响
     *
     * **Step 2-3: Sigma点变换（UT变换）**
     * - Sigma点：χ⁰=x, χⁱ=x+γ*Sx_col_i, χⁱ⁺ᴺᴳ=x-γ*Sx_col_i（i=1..NX）
     * - 权重：w⁰=1-λ, wⁱ=wⁱ⁺ᴺᴳ=λ/(2*NX)（这里使用简化权重）
     * - 测量预测：zⁱ=h(χⁱ)，然后加权平均
     * - 协方差预测：Pzz = Σ wⁱ(zⁱ-z̄)(zⁱ-z̄)ᵀ
     *
     * **Step 4: 马氏距离计算**
     * - 新息：ν = z_obs - z_pred
     * - 新息协方差：S = Pzz + R（测量协方差）
     * - 马氏距离：d² = νᵀS⁻¹ν
     * - 代价融合先验：nll = d² - 2*log(prior)
     *
     * **Step 5: 贪婪分配**
     * - 迭代选择最小代价的有效匹配
     * - 标记已使用的观测和装甲板ID，避免重复匹配
     *
     * @param observations 批量装甲板观测
     * @param x_pred 预测状态向量
     * @param Sx_pred 预测协方差平方根（用于生成Sigma点）
     * @param model 运动模型实例
     * @param target_name 目标装甲板名称
     * @param gate_threshold 马氏距离门控阈值
     * @param armors_num 目标装甲板数量
     * @param id_prior 装甲板ID先验概率（可选）
     * @return 匹配结果
     *
     * @note 时间复杂度：O(n_obs * armors_num * NX)，优于匈牙利算法O(n³)
     * @note 数值稳定性：使用LDLT分解而非直接求逆，避免病态矩阵问题
     */
    [[nodiscard]] MatchResult match(
        const ArmorMeasurementBatch& observations, const VecX& x_pred, const MatXX& Sx_pred,
        Model& model, ArmorName target_name, double gate_threshold, int armors_num,
        const std::vector<double>* id_prior) const noexcept {

        MatchResult result;

        // ===== 边界条件：观测为空时立即返回 =====
        if (observations.empty()) {
            return result;
        }

        const int n_obs = static_cast<int>(observations.size());

        // ===== Step 1: 初始化代价矩阵 =====
        // 代价矩阵：cost[j][k] 表示第j个观测与第k个装甲板ID的匹配代价
        // 初始化为无穷大，表示默认不匹配
        std::vector<std::vector<double>> cost(
            n_obs, std::vector<double>(armors_num, std::numeric_limits<double>::infinity()));

        // ===== Step 2: 预处理观测 =====
        // 将Cartesian坐标转换为球坐标测量，并缓存
        std::vector<VecZ> meas_list(n_obs);
        for (int j = 0; j < n_obs; ++j) {
            // 过滤不匹配目标名称的观测
            if (observations.measurements[j].name != target_name) {
                continue;
            }
            const auto& m  = observations.measurements[j];
            const auto ypd = xyz2ypd(m.transform.translation());

            // 构建测量向量：[yaw, pitch, log(distance), armor_yaw]
            VecZ z;
            z[0]         = ypd[0];                           // yaw（方位角）
            z[1]         = ypd[1];                           // pitch（俯仰角）
            z[2]         = std::log(std::max(ypd[2], 1e-9)); // log(distance)，避免负数和数值溢出
            z[3]         = m.transform.euler_rot().yaw;      // armor_yaw（装甲板朝向）
            meas_list[j] = z;
        }

        // ===== Step 3: 预计算测量预测和协方差（Sigma点变换）=====
        // 对每个装甲板ID生成Sigma点并计算测量预测和协方差
        struct PredMeas {
            VecZ z_pred;         ///< 预测测量
            MatZ Pzz;            ///< 测量协方差
            bool visible{false}; ///< 是否可见（根据朝向判断）
        };
        std::vector<PredMeas> pred_cache(armors_num);

        // Sigma点参数：gamma = sqrt(NX)，用于生成Sigma点
        const double gamma = std::sqrt(static_cast<double>(NX));
        // Sigma点权重：w = 1/(2*NX)，简化权重（假设均值为零）
        const double w = 1.0 / (2.0 * static_cast<double>(NX));

        // 遍历每个装甲板ID，生成Sigma点并计算测量预测
        for (int id = 0; id < armors_num; ++id) {
            // 计算名义测量预测（用于判断可见性）
            const VecZ z0 = model.h(x_pred, id);

            // 可见性判断：根据装甲板朝向过滤不可见装甲板
            // 原理：装甲板法向量必须指向相机方向才可见
            if (!armor_measurement_visible_from_origin(z0)) {
                pred_cache[id] = {z0, MatZ::Zero(), false};
                continue; // 跳过不可见装甲板，避免无效计算
            }

            // ===== Sigma点生成（UT变换核心）=====
            // 生成2*NX个Sigma点：χⁱ⁺ = x + γ*Sx_col_i, χⁱ⁻ = x - γ*Sx_col_i
            std::array<VecZ, 2 * NX> z_sig{};
            for (int k = 0; k < NX; ++k) {
                // 生成正负Sigma点（状态空间）
                const VecX xp = x_pred + gamma * Sx_pred.col(k);
                const VecX xm = x_pred - gamma * Sx_pred.col(k);

                // 传播Sigma点到测量空间
                VecZ zp = model.h(xp, id);
                VecZ zm = model.h(xm, id);

                // ===== 角度展开：处理角度跳变 =====
                // 目的：避免Sigma点在±π附近跳变，保证角度连续性
                // 方法：使用unwrap_rad将角度展开到与z0最接近的值
                zp[0] = unwrap_rad(z0[0], zp[0]); // yaw
                zp[1] = unwrap_rad(z0[1], zp[1]); // pitch
                zp[3] = unwrap_rad(z0[3], zp[3]); // armor_yaw

                zm[0] = unwrap_rad(z0[0], zm[0]);
                zm[1] = unwrap_rad(z0[1], zm[1]);
                zm[3] = unwrap_rad(z0[3], zm[3]);

                // 存储Sigma点
                z_sig[k]      = zp;
                z_sig[NX + k] = zm;
            }

            // ===== 计算预测测量均值 =====
            // z̄ = Σ wⁱ * zⁱ
            VecZ z_bar = VecZ::Zero();
            for (const auto& zs : z_sig) {
                z_bar.noalias() += w * zs;
            }

            // ===== 计算测量协方差 =====
            // Pzz = Σ wⁱ * (zⁱ - z̄)(zⁱ - z̄)ᵀ
            // 注意：对角度使用shortest_rad计算差值，处理周期性
            MatZ Pzz = MatZ::Zero();
            for (const auto& zs : z_sig) {
                VecZ dz;
                dz[0] = shortest_rad(z_bar[0], zs[0]); // yaw差值（归一化）
                dz[1] = zs[1] - z_bar[1];              // pitch差值（非周期）
                dz[2] = zs[2] - z_bar[2];              // log(distance)差值（非周期）
                dz[3] = shortest_rad(z_bar[3], zs[3]); // armor_yaw差值（归一化）
                Pzz.noalias() += w * (dz * dz.transpose());
            }

            pred_cache[id] = {z_bar, Pzz, true};
        }

        // ===== Step 4: 计算马氏距离代价 =====
        // 遍历每个（观测，装甲板ID）对，计算匹配代价
        for (int j = 0; j < n_obs; ++j) {
            // 过滤不匹配目标名称的观测
            if (observations.measurements[j].name != target_name) {
                continue; // 只匹配目标名称一致的装甲板
            }

            for (int id = 0; id < armors_num; ++id) {
                // 跳过不可见装甲板
                if (!pred_cache[id].visible) {
                    continue;
                }
                const auto& z_pred = pred_cache[id].z_pred;

                // ===== 计算新息（innovation）：观测与预测的差值 =====
                VecZ nu;
                nu[0] = shortest_rad(z_pred[0], meas_list[j][0]); // yaw新息（归一化）
                nu[1] = meas_list[j][1] - z_pred[1];              // pitch新息
                nu[2] = meas_list[j][2] - z_pred[2];              // log(distance)新息
                nu[3] = shortest_rad(z_pred[3], meas_list[j][3]); // armor_yaw新息（归一化）

                // ===== 构建新息协方差矩阵 =====
                // S = Pzz + R（测量协方差 + 过程协方差）
                const Eigen::Matrix<double, NZ, 1> R_diag = model.R_diag(meas_list[j]);
                MatZ S                                    = pred_cache[id].Pzz;
                S.diagonal() += R_diag; // 对角假设：直接相加

                // 数值稳定性处理：
                // 1. 强制对称：S = (S + Sᵀ) / 2
                S = (S + S.transpose()) * 0.5;
                // 2. 正则化：添加小量到对角线，避免奇异矩阵
                S.diagonal().array() += 1e-9;

                // ===== LDLT分解求解马氏距离 =====
                // d² = νᵀS⁻¹ν = νᵀ(LDLᵀ)⁻¹ν
                // LDLT比LLT更稳定，不需要矩阵正定
                const Eigen::LDLT<MatZ> ldlt(S);
                if (ldlt.info() != Eigen::Success) {
                    continue; // 分解失败，跳过此匹配
                }

                // 计算马氏距离平方
                double d2 = nu.transpose() * ldlt.solve(nu);

                // ===== 门控：过滤距离过大的匹配 =====
                if (std::isfinite(d2) && d2 < gate_threshold) {
                    double nll = d2; // 负对数似然（初始为马氏距离²）

                    // ===== 融合先验概率 =====
                    // nll = d² - 2*log(prior)
                    if (id_prior != nullptr && static_cast<int>(id_prior->size()) == armors_num) {
                        const double p = std::clamp((*id_prior)[id], 1e-6, 1.0);
                        nll += -2.0 * std::log(p);
                    }
                    cost[j][id] = nll;
                }
            }
        }

        // ===== Step 5: 贪婪一对一分配 =====
        // 迭代选择最小代价的有效匹配，避免匈牙利算法的O(n³)复杂度
        std::vector<bool> used_obs(n_obs, false);     // 已使用的观测标记
        std::vector<bool> used_id(armors_num, false); // 已使用的装甲板ID标记

        while (true) {
            // 查找最小代价匹配
            double best = std::numeric_limits<double>::infinity();
            int best_j  = -1;
            int best_id = -1;

            for (int j = 0; j < n_obs; ++j) {
                if (used_obs[j])
                    continue;     // 跳过已使用的观测
                for (int id = 0; id < armors_num; ++id) {
                    if (used_id[id])
                        continue; // 跳过已使用的装甲板ID
                    if (cost[j][id] < best) {
                        best    = cost[j][id];
                        best_j  = j;
                        best_id = id;
                    }
                }
            }

            // 终止条件：没有有效匹配
            if (best_j < 0 || best_id < 0) {
                break;
            }

            // 标记已使用
            used_obs[best_j] = true;
            used_id[best_id] = true;

            // 添加到结果
            result.armors.push_back(observations.measurements[best_j]);
            result.armor_ids.push_back(best_id);
            result.costs.push_back(best);
        }

        return result;
    }
};

} // namespace fcs::L3
