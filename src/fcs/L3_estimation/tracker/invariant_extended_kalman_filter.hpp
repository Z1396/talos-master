#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <functional>
#include <utility>

namespace fcs::L3 {

template <class NominalT, int N_Z>
class InvariantExtendedKalmanFilter {
public:
    using Nominal = NominalT;
    using Scalar  = typename Nominal::Scalar;

    using MatrixXX = typename Nominal::TMatrixType;
    using MatrixZX = Eigen::Matrix<Scalar, N_Z, MatrixXX::RowsAtCompileTime>;
    using MatrixXZ = Eigen::Matrix<Scalar, MatrixXX::RowsAtCompileTime, N_Z>;
    using MatrixZZ = Eigen::Matrix<Scalar, N_Z, N_Z>;
    using MatrixX1 = typename Nominal::VectorType;

    using PredictFunc = std::function<Nominal(const Nominal&, Scalar)>;
    using UpdateQFunc = std::function<MatrixXX(Scalar)>;

    static constexpr int N_X = MatrixX1::RowsAtCompileTime;

    InvariantExtendedKalmanFilter() = default;

    explicit InvariantExtendedKalmanFilter(
        PredictFunc f, UpdateQFunc update_q, const MatrixXX& P0,
        Scalar finite_difference_step = Scalar(1e-6))
        : f_(std::move(f))
        , update_Q_(std::move(update_q))
        , finite_difference_step_(finite_difference_step)
        , P_post_(P0) {
        F_.setIdentity();
        H_.setZero();
        Q_.setZero();
        R_.setZero();
        K_.setZero();
    }

    void setState(const Nominal& x0) { x_post_ = x0; }

    [[nodiscard]] const Nominal& X() const noexcept { return x_post_; }
    [[nodiscard]] const MatrixXX& P() const noexcept { return P_post_; }

    const Nominal& predict(Scalar dt) {
        dt                  = std::max(Scalar(0), dt);
        const Nominal x_pri = f_(x_post_, dt);
        F_                  = linearize_process_(x_post_, x_pri, dt);
        Q_                  = update_Q_(dt);

        P_post_ = symmetrized_(F_ * P_post_ * F_.transpose() + Q_);
        x_post_ = x_pri;

        return x_post_;
    }

    const Nominal& update(
        const Eigen::Matrix<Scalar, N_Z, 1>& innovation, const MatrixZX& H, const MatrixZZ& R) {
        H_ = H;
        R_ = R;

        const MatrixZZ S = H_ * P_post_ * H_.transpose() + R_;
        K_               = P_post_ * H_.transpose() * S.inverse();

        const MatrixX1 dx = K_ * innovation;
        x_post_           = x_post_ * Nominal::exp(dx);

        const MatrixXX IKH = MatrixXX::Identity() - K_ * H_;
        P_post_ = symmetrized_(IKH * P_post_ * IKH.transpose() + K_ * R_ * K_.transpose());

        return x_post_;
    }

private:
    [[nodiscard]] static MatrixXX symmetrized_(const MatrixXX& P) {
        return Scalar(0.5) * (P + P.transpose());
    }

    [[nodiscard]] MatrixXX
        linearize_process_(const Nominal& x, const Nominal& x_pred, Scalar dt) const {
        MatrixXX F;
        F.setZero();

        const Scalar eps = finite_difference_step_;
        for (int i = 0; i < N_X; ++i) {
            MatrixX1 d = MatrixX1::Zero();
            d[i]       = eps;

            const Nominal x_plus  = x * Nominal::exp(d);
            const Nominal y_plus  = f_(x_plus, dt);
            const MatrixX1 e_plus = Nominal::log(x_pred.inv() * y_plus);

            d[i]                   = -eps;
            const Nominal x_minus  = x * Nominal::exp(d);
            const Nominal y_minus  = f_(x_minus, dt);
            const MatrixX1 e_minus = Nominal::log(x_pred.inv() * y_minus);

            F.col(i) = (e_plus - e_minus) / (Scalar(2) * eps);
        }

        return F;
    }

    PredictFunc f_{};
    UpdateQFunc update_Q_{};
    Scalar finite_difference_step_{Scalar(1e-6)};

    MatrixXX F_{MatrixXX::Identity()};
    MatrixZX H_{MatrixZX::Zero()};
    MatrixXX Q_{MatrixXX::Zero()};
    MatrixZZ R_{MatrixZZ::Zero()};
    MatrixXX P_post_{MatrixXX::Identity()};
    MatrixXZ K_{MatrixXZ::Zero()};

    Nominal x_post_{};
};

} // namespace fcs::L3
