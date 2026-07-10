// ===========================================================================
// matrix.hpp - 固定大小矩阵模板（EKF 所需）
//
// 设计要点：
//   1. 编译期固定大小 N x M，栈分配，无动态内存
//   2. 支持 +、-、*、转置、求逆
//   3. 仅实现 EKF 所需运算，不追求通用性
// ===========================================================================
#pragma once

#include <array>
#include <cstddef>
#include <stdexcept>

namespace armor {

// 固定大小矩阵 N 行 M 列
template <std::size_t N, std::size_t M>
class Matrix {
public:
    static constexpr std::size_t kRows = N;
    static constexpr std::size_t kCols = M;

    constexpr Matrix() = default;

    // 访问元素
    [[nodiscard]] double& operator()(std::size_t i, std::size_t j) {
        return data_[i * M + j];
    }
    [[nodiscard]] const double& operator()(std::size_t i, std::size_t j) const {
        return data_[i * M + j];
    }

    // 零矩阵
    static constexpr Matrix zeros() {
        Matrix m{};
        m.data_.fill(0.0);
        return m;
    }

    // 单位矩阵（仅方阵）
    static constexpr Matrix identity() requires (N == M) {
        Matrix m = zeros();
        for (std::size_t i = 0; i < N; ++i) {
            m(i, i) = 1.0;
        }
        return m;
    }

    // 矩阵加法
    [[nodiscard]] Matrix operator+(const Matrix& other) const {
        Matrix result;
        for (std::size_t i = 0; i < N * M; ++i) {
            result.data_[i] = data_[i] + other.data_[i];
        }
        return result;
    }

    // 矩阵减法
    [[nodiscard]] Matrix operator-(const Matrix& other) const {
        Matrix result;
        for (std::size_t i = 0; i < N * M; ++i) {
            result.data_[i] = data_[i] - other.data_[i];
        }
        return result;
    }

    // 标量乘法
    [[nodiscard]] Matrix operator*(double scalar) const {
        Matrix result;
        for (std::size_t i = 0; i < N * M; ++i) {
            result.data_[i] = data_[i] * scalar;
        }
        return result;
    }

    // 矩阵乘法 A(N×M) * B(M×K) = C(N×K)
    template <std::size_t K>
    [[nodiscard]] Matrix<N, K> operator*(const Matrix<M, K>& other) const {
        Matrix<N, K> result = Matrix<N, K>::zeros();
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = 0; j < K; ++j) {
                for (std::size_t k = 0; k < M; ++k) {
                    result(i, j) += (*this)(i, k) * other(k, j);
                }
            }
        }
        return result;
    }

    // 转置
    [[nodiscard]] Matrix<M, N> transpose() const {
        Matrix<M, N> result;
        for (std::size_t i = 0; i < N; ++i) {
            for (std::size_t j = 0; j < M; ++j) {
                result(j, i) = (*this)(i, j);
            }
        }
        return result;
    }

    // 4x4 求逆（高斯-约旦消元，仅方阵且可逆）
    [[nodiscard]] Matrix inverse() const requires (N == M && N == 4) {
        // 构造增广矩阵 [A | I]
        Matrix<4, 8> aug = Matrix<4, 8>::zeros();
        for (std::size_t i = 0; i < 4; ++i) {
            for (std::size_t j = 0; j < 4; ++j) {
                aug(i, j) = (*this)(i, j);
            }
            aug(i, 4 + i) = 1.0;
        }

        // 前向消元
        for (std::size_t col = 0; col < 4; ++col) {
            // 找主元
            std::size_t max_row = col;
            double max_val = std::abs(aug(col, col));
            for (std::size_t row = col + 1; row < 4; ++row) {
                if (std::abs(aug(row, col)) > max_val) {
                    max_val = std::abs(aug(row, col));
                    max_row = row;
                }
            }
            // 交换行
            if (max_row != col) {
                for (std::size_t j = 0; j < 8; ++j) {
                    std::swap(aug(col, j), aug(max_row, j));
                }
            }
            // 归一化主元行
            double pivot = aug(col, col);
            for (std::size_t j = 0; j < 8; ++j) {
                aug(col, j) /= pivot;
            }
            // 消去其他行
            for (std::size_t row = 0; row < 4; ++row) {
                if (row == col) continue;
                double factor = aug(row, col);
                for (std::size_t j = 0; j < 8; ++j) {
                    aug(row, j) -= factor * aug(col, j);
                }
            }
        }

        // 提取逆矩阵
        Matrix inv;
        for (std::size_t i = 0; i < 4; ++i) {
            for (std::size_t j = 0; j < 4; ++j) {
                inv(i, j) = aug(i, 4 + j);
            }
        }
        return inv;
    }

private:
    std::array<double, N * M> data_{};
};

// 列向量（N×1 矩阵的别名）
template <std::size_t N>
using Vector = Matrix<N, 1>;

}  // namespace armor
