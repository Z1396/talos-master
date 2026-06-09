#include <gtest/gtest.h>

#include "L3_estimation/manager.hpp"
#include "L3_estimation/tracker/config.hpp"
#include "L3_estimation/tracker/data_associator.hpp"
#include "core/types.hpp"

#include <cmath>
#include <magic_enum.hpp>
#include <numbers>

namespace fcs::L3::testing {

struct AssociatorVisibilityModel {
    using Scalar                    = double;
    static constexpr int NX         = 1;
    static constexpr int NZ         = MEASURE_MAX;
    static constexpr int ARMORS_NUM = 2;

    using VecX  = Eigen::Matrix<double, NX, 1>;
    using VecZ  = Eigen::Matrix<double, NZ, 1>;
    using MatXX = Eigen::Matrix<double, NX, NX>;
    using MatZZ = Eigen::Matrix<double, NZ, NZ>;

    [[nodiscard]] VecX f(const VecX& x, double) const noexcept { return x; }

    [[nodiscard]] VecZ h(const VecX& x) const noexcept { return h(x, 0); }

    [[nodiscard]] VecZ h(const VecX&, int armor_id) const noexcept {
        VecZ z             = VecZ::Zero();
        z[ARMOR_YAW]       = 0.0;
        z[ARMOR_PITCH]     = 0.0;
        z[ARMOR_DISTANCE]  = (armor_id == 0) ? 4.0 : 5.0;
        z[ARMOR_YAW_ARMOR] = (armor_id == 0) ? 0.0 : std::numbers::pi;
        return z;
    }

    [[nodiscard]] MatXX Q_sqrt(double) const noexcept { return MatXX::Zero(); }

    [[nodiscard]] MatZZ R_sqrt(const VecZ&) const noexcept { return MatZZ::Identity(); }

    [[nodiscard]] Eigen::Matrix<double, NZ, 1> R_diag(const VecZ&) const noexcept {
        return Eigen::Matrix<double, NZ, 1>::Constant(1e-3);
    }
};

// ============================================================================
// Test Fixtures
// ============================================================================

class TrackerManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create minimal valid config
        config_.robot.model              = RobotEkfMotionModel::Params{};
        config_.robot.lost_threshold     = 1.0;
        config_.robot.tracking_threshold = 3;
        config_.robot.matcher_gate       = 10.0;

        config_.outpost.model              = OutpostEkfMotionModel::Params{};
        config_.outpost.lost_threshold     = 1.0;
        config_.outpost.tracking_threshold = 3;
        config_.outpost.matcher_gate       = 10.0;

        config_.robot_inekf.radius0 = 0.22;
        config_.robot_inekf.radius1 = 0.22;
        config_.robot_inekf.height  = 0.0;
    }

    TrackerConfig config_;

    // Helper: Create a simple measurement for testing
    [[nodiscard]] ArmorMeasurement
        create_measurement(ArmorName name, ArmorColor color, float distance = 5.0f) {
        ArmorMeasurement meas;
        meas.name                     = name;
        meas.color                    = color;
        meas.distance_to_image_center = distance;
        // Default-constructed transform is identity
        meas.transform = {};
        return meas;
    }

    [[nodiscard]] ArmorMeasurement create_pose_measurement(
        ArmorName name, ArmorColor color, const Eigen::Vector3d& position, double yaw,
        float image_center_distance = 0.0f) {
        ArmorMeasurement meas;
        meas.name                     = name;
        meas.color                    = color;
        meas.type                     = ArmorType::Small;
        meas.confidence               = 1.0f;
        meas.distance_to_image_center = image_center_distance;
        meas.transform                = ArmorMeasurement::Transform::from_rpy(
            0.0, 0.0, yaw, position.x(), position.y(), position.z());
        return meas;
    }

    // Helper: Create a measurement batch for testing
    [[nodiscard]] ArmorMeasurementBatch
        create_batch(const std::vector<ArmorMeasurement>& measurements) {
        ArmorMeasurementBatch batch;
        batch.timestamp_ns = 1000000; // 1ms
        batch.frame_id     = 42;
        batch.measurements = measurements;
        return batch;
    }
};

// ============================================================================
// TargetKey Tests
// ============================================================================

TEST_F(TrackerManagerTest, TargetKey_EqualStructuresAreEqual) {
    const core::TargetKey key1{ArmorName::One, ArmorColor::Red};
    const core::TargetKey key2{ArmorName::One, ArmorColor::Red};

    EXPECT_EQ(key1, key2);
}

TEST_F(TrackerManagerTest, TargetKey_DifferentNamesAreNotEqual) {
    const core::TargetKey key1{ArmorName::One, ArmorColor::Red};
    const core::TargetKey key2{ArmorName::Two, ArmorColor::Red};

    EXPECT_NE(key1, key2);
}

TEST_F(TrackerManagerTest, TargetKey_DifferentColorsAreNotEqual) {
    const core::TargetKey key1{ArmorName::One, ArmorColor::Red};
    const core::TargetKey key2{ArmorName::One, ArmorColor::Blue};

    EXPECT_NE(key1, key2);
}

TEST_F(TrackerManagerTest, TargetKey_HashFunctionIsDeterministic) {
    const core::TargetKey key{ArmorName::One, ArmorColor::Red};
    core::TargetKeyHash hasher;

    const size_t h1 = hasher(key);
    const size_t h2 = hasher(key);

    EXPECT_EQ(h1, h2);
}

TEST_F(TrackerManagerTest, TargetKey_DifferentHashesProduceDifferentValues) {
    core::TargetKeyHash hasher;

    const core::TargetKey key1{ArmorName::One, ArmorColor::Red};
    const core::TargetKey key2{ArmorName::One, ArmorColor::Blue};

    EXPECT_NE(hasher(key1), hasher(key2));
}

// ============================================================================
// TrackerManager Construction Tests
// ============================================================================

TEST_F(TrackerManagerTest, Construction_PreallocatesAllValidCombinations) {
    TrackerManager manager(config_);

    // Count valid (name, color) combinations
    const auto all_names  = magic_enum::enum_values<ArmorName>();
    const auto all_colors = magic_enum::enum_values<ArmorColor>();

    int expected_count = 0;
    for (const auto name : all_names) {
        if (name == ArmorName::Invalid)
            continue;

        for (const auto color : all_colors) {
            if (color == ArmorColor::Neutral)
                continue;
            expected_count++;
        }
    }

    // Verify we can get trackers for all combinations
    int actual_count = 0;
    for (const auto name : all_names) {
        if (name == ArmorName::Invalid)
            continue;

        for (const auto color : all_colors) {
            if (color == ArmorColor::Neutral)
                continue;

            auto tracker = manager.get_tracker(name, color);
            ASSERT_TRUE(tracker.has_value())
                << "Missing tracker for " << magic_enum::enum_name(name) << "/"
                << magic_enum::enum_name(color);
            actual_count++;
        }
    }

    EXPECT_EQ(actual_count, expected_count);
}

TEST_F(TrackerManagerTest, Construction_DoesNotAllocateInvalidName) {
    TrackerManager manager(config_);

    auto tracker = manager.get_tracker(ArmorName::Invalid, ArmorColor::Red);
    EXPECT_FALSE(tracker.has_value());
}

TEST_F(TrackerManagerTest, Construction_DoesNotAllocateNeutralColor) {
    TrackerManager manager(config_);

    auto tracker = manager.get_tracker(ArmorName::One, ArmorColor::Neutral);
    EXPECT_FALSE(tracker.has_value());
}

TEST_F(TrackerManagerTest, Construction_AllTrackersStartInIdleState) {
    TrackerManager manager(config_);

    auto all_outputs = manager.all_outputs();

    for (const auto& output : all_outputs) {
        EXPECT_EQ(output.status, TrackerStatus::Idle);
        EXPECT_EQ(output.target_name, ArmorName::Invalid);
        EXPECT_EQ(output.target_color, ArmorColor::Neutral);
    }
}

// ============================================================================
// get_tracker() Tests
// ============================================================================

TEST_F(TrackerManagerTest, GetTracker_ReturnsValidTrackerForExistingTarget) {
    TrackerManager manager(config_);

    auto tracker = manager.get_tracker(ArmorName::One, ArmorColor::Red);

    ASSERT_TRUE(tracker.has_value());
    EXPECT_EQ(tracker->get().target_name(), ArmorName::Invalid);
    EXPECT_EQ(tracker->get().status(), TrackerStatus::Idle);
}

TEST_F(TrackerManagerTest, GetTracker_ReturnsNulloptForNonExistentTarget) {
    TrackerManager manager(config_);

    // Invalid name
    auto tracker1 = manager.get_tracker(ArmorName::Invalid, ArmorColor::Red);
    EXPECT_FALSE(tracker1.has_value());

    // Neutral color
    auto tracker2 = manager.get_tracker(ArmorName::One, ArmorColor::Neutral);
    EXPECT_FALSE(tracker2.has_value());
}

TEST_F(TrackerManagerTest, GetTracker_ConstVersionWorks) {
    const TrackerManager manager(config_);

    auto tracker = manager.get_tracker(ArmorName::One, ArmorColor::Red);
    ASSERT_TRUE(tracker.has_value());
}

// ============================================================================
// update_all() Tests
// ============================================================================

TEST_F(TrackerManagerTest, UpdateAll_InitializesTrackerWithFirstMeasurement) {
    TrackerManager manager(config_);

    const auto batch = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.0f)});

    const auto outputs = manager.update_all(batch);

    ASSERT_EQ(outputs.size(), 1);
    EXPECT_EQ(outputs[0].target_name, ArmorName::One);
    EXPECT_EQ(outputs[0].target_color, ArmorColor::Red);
    EXPECT_NE(outputs[0].status, TrackerStatus::Idle);
}

TEST_F(TrackerManagerTest, UpdateAll_RoutesMeasurementsToCorrectTrackers) {
    TrackerManager manager(config_);

    const auto batch = create_batch(
        {create_measurement(ArmorName::One, ArmorColor::Red, 1.0f),
         create_measurement(ArmorName::Two, ArmorColor::Blue, 2.0f)});

    const auto outputs = manager.update_all(batch);

    ASSERT_EQ(outputs.size(), 2);

    // Verify we got outputs for both targets
    bool has_hero_red = false;
    bool has_two_blue = false;

    for (const auto& output : outputs) {
        if (output.target_name == ArmorName::One && output.target_color == ArmorColor::Red) {
            has_hero_red = true;
        }
        if (output.target_name == ArmorName::Two && output.target_color == ArmorColor::Blue) {
            has_two_blue = true;
        }
    }

    EXPECT_TRUE(has_hero_red);
    EXPECT_TRUE(has_two_blue);
}

TEST_F(TrackerManagerTest, UpdateAll_UpdatesExistingTracker) {
    TrackerManager manager(config_);

    // First update - initialize
    auto batch1         = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.0f)});
    batch1.timestamp_ns = 1000000;
    auto outputs1       = manager.update_all(batch1);

    // Second update - update existing tracker
    auto batch2         = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.5f)});
    batch2.timestamp_ns = 2000000; // 1ms later
    auto outputs2       = manager.update_all(batch2);

    ASSERT_EQ(outputs2.size(), 1);
    // Status should have progressed (Idle -> Detecting or Tracking)
    EXPECT_NE(outputs2[0].status, TrackerStatus::Idle);
    EXPECT_EQ(outputs2[0].target_name, ArmorName::One);
    EXPECT_EQ(outputs2[0].target_color, ArmorColor::Red);
}

TEST_F(TrackerManagerTest, UpdateAll_ReturnsOnlyUpdatedTrackers) {
    TrackerManager manager(config_);

    // Initialize Hero/Red tracker
    auto batch1 = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.0f)});
    manager.update_all(batch1);

    // Update only Hero/Red (not Two/Blue)
    auto batch2  = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.5f)});
    auto outputs = manager.update_all(batch2);

    EXPECT_EQ(outputs.size(), 1);
    EXPECT_EQ(outputs[0].target_name, ArmorName::One);
    EXPECT_EQ(outputs[0].target_color, ArmorColor::Red);
}

TEST_F(TrackerManagerTest, UpdateAll_HandlesEmptyMeasurementBatch) {
    TrackerManager manager(config_);

    const auto batch   = create_batch({});
    const auto outputs = manager.update_all(batch);

    EXPECT_EQ(outputs.size(), 0);
}

TEST_F(TrackerManagerTest, DataAssociator_RejectsInvisibleArmorId) {
    AssociatorVisibilityModel model;
    AssociatorVisibilityModel::VecX x = AssociatorVisibilityModel::VecX::Zero();

    const int hidden_id = 1;
    const auto z_hidden = model.h(x, hidden_id);
    ASSERT_FALSE(armor_measurement_visible_from_origin(z_hidden));

    const auto hidden_position =
        ypd2xyz({z_hidden[ARMOR_YAW], z_hidden[ARMOR_PITCH], z_hidden[ARMOR_DISTANCE]});
    const auto batch = create_batch({create_pose_measurement(
        ArmorName::One, ArmorColor::Red, hidden_position, z_hidden[ARMOR_YAW_ARMOR])});

    DataAssociator<AssociatorVisibilityModel> associator;
    const auto result = associator.match(
        batch, x, AssociatorVisibilityModel::MatXX::Zero(), model, ArmorName::One, 10.0,
        AssociatorVisibilityModel::ARMORS_NUM);

    EXPECT_TRUE(result.armors.empty());
    EXPECT_TRUE(result.armor_ids.empty());
}

TEST_F(TrackerManagerTest, TrackerNew_RejectsInvisibleArmorIdMatch) {
    config_.robot.tracking_threshold = 0;
    TrackerNew tracker(config_);

    const double radius        = config_.robot_inekf.radius0;
    auto initial_batch         = create_batch({create_pose_measurement(
        ArmorName::One, ArmorColor::Red, Eigen::Vector3d(5.0 - radius, 0.0, 0.0), 0.0)});
    initial_batch.timestamp_ns = 1000000;

    ASSERT_TRUE(tracker.first_meet(initial_batch).has_value());
    ASSERT_EQ(tracker.status(), TrackerStatus::Detecting);

    auto hidden_batch         = create_batch({create_pose_measurement(
        ArmorName::One, ArmorColor::Red, Eigen::Vector3d(5.0 + radius, 0.0, 0.0),
        std::numbers::pi)});
    hidden_batch.timestamp_ns = 2000000;

    tracker.predict(0.001);
    EXPECT_FALSE(tracker.update(hidden_batch));
    EXPECT_EQ(tracker.status(), TrackerStatus::Idle);
}

// ============================================================================
// Query API Tests
// ============================================================================

TEST_F(TrackerManagerTest, AllOutputs_ReturnsAllPreallocatedTrackers) {
    TrackerManager manager(config_);

    auto all_outputs = manager.all_outputs();

    // Count should match valid (name, color) combinations
    const auto all_names  = magic_enum::enum_values<ArmorName>();
    const auto all_colors = magic_enum::enum_values<ArmorColor>();

    int expected_count = 0;
    for (const auto name : all_names) {
        if (name == ArmorName::Invalid)
            continue;
        for (const auto color : all_colors) {
            if (color == ArmorColor::Neutral)
                continue;
            expected_count++;
        }
    }

    EXPECT_EQ(static_cast<int>(all_outputs.size()), expected_count);
}

TEST_F(TrackerManagerTest, ActiveOutputs_ReturnsOnlyTrackingTrackers) {
    TrackerManager manager(config_);

    // Before any updates, no trackers are active (is_tracking() == false for Idle/Detecting)
    auto active1 = manager.active_outputs();
    EXPECT_EQ(active1.size(), 0);

    // Initialize a tracker with first_meet
    auto batch         = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.0f)});
    batch.timestamp_ns = 1000000;
    manager.update_all(batch);

    // After initialization, tracker is in Detecting (not yet Tracking)
    // Detecting status is NOT considered "active" (is_tracking() == false)
    auto active2 = manager.active_outputs();
    EXPECT_EQ(active2.size(), 0);

    // Verify the tracker exists and is in Detecting status
    auto all_outputs       = manager.all_outputs();
    bool one_red_detecting = false;
    for (const auto& output : all_outputs) {
        if (output.target_name == ArmorName::One && output.target_color == ArmorColor::Red) {
            EXPECT_EQ(output.status, TrackerStatus::Detecting);
            one_red_detecting = true;
            break;
        }
    }
    EXPECT_TRUE(one_red_detecting) << "One/Red tracker should be in Detecting state";

    // Note: We don't test reaching Tracking status here because it requires
    // complex measurement sequences. The important thing is that the query
    // correctly filters by is_tracking() status.
}

TEST_F(TrackerManagerTest, ActiveOutputs_FiltersCorrectly) {
    TrackerManager manager(config_);

    // Initialize Hero/Red tracker
    auto batch = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.0f)});

    // Multiple updates to reach Tracking status
    for (int i = 0; i < 10; ++i) {
        batch.timestamp_ns = (i + 1) * 1000000;
        manager.update_all(batch);
    }

    auto active = manager.active_outputs();

    // All active trackers should pass is_tracking() check
    for (const auto& output : active) {
        EXPECT_TRUE(output.is_tracking());
    }
}

TEST_F(TrackerManagerTest, OutputsWithStatus_FiltersByStatus) {
    TrackerManager manager(config_);

    // All trackers start in Idle
    auto idle_outputs = manager.outputs_with_status(TrackerStatus::Idle);
    EXPECT_GT(idle_outputs.size(), 0);

    for (const auto& output : idle_outputs) {
        EXPECT_EQ(output.status, TrackerStatus::Idle);
    }

    // No trackers in Tracking initially
    auto tracking_outputs = manager.outputs_with_status(TrackerStatus::Tracking);
    EXPECT_EQ(tracking_outputs.size(), 0);
}

TEST_F(TrackerManagerTest, OutputsWithName_FiltersByNameOnly) {
    TrackerManager manager(config_);

    auto hero_outputs = manager.outputs_with_name(ArmorName::One);

    // Should have one tracker per color (except Neutral)
    const auto all_colors = magic_enum::enum_values<ArmorColor>();
    int expected_count    = 0;
    for (const auto color : all_colors) {
        if (color != ArmorColor::Neutral) {
            expected_count++;
        }
    }

    EXPECT_EQ(static_cast<int>(hero_outputs.size()), expected_count);

    // All returned trackers should have key.name == One (but target_name is Invalid until
    // initialized)
    for (const auto& output : hero_outputs) {
        // The filter is by (name, color) key, not by output.target_name
        // output.target_name is Invalid for uninitialized trackers
        // We can't directly check the key from the output, so just verify we got the right count
    }
}

// ============================================================================
// Pipeable Filter Tests
// ============================================================================

TEST_F(TrackerManagerTest, PipeableFilter_WithColorWorks) {
    using namespace fcs::L3::filters;

    TrackerManager manager(config_);

    auto all      = manager.all_outputs();
    auto red_only = all | with_color(ArmorColor::Red);

    // Convert view to vector for counting
    TrackerOutputs red_outputs;
    for (const auto& output : red_only) {
        red_outputs.push_back(output);
    }

    // All should be Red
    for (const auto& output : red_outputs) {
        EXPECT_EQ(output.target_color, ArmorColor::Red);
    }
}

TEST_F(TrackerManagerTest, PipeableFilter_WithNameWorks) {
    using namespace fcs::L3::filters;

    TrackerManager manager(config_);

    auto all       = manager.all_outputs();
    auto hero_only = all | with_name(ArmorName::One);

    TrackerOutputs hero_outputs;
    for (const auto& output : hero_only) {
        hero_outputs.push_back(output);
    }

    for (const auto& output : hero_outputs) {
        EXPECT_EQ(output.target_name, ArmorName::One);
    }
}

TEST_F(TrackerManagerTest, PipeableFilter_IsTrackingWorks) {
    using namespace fcs::L3::filters;

    TrackerManager manager(config_);

    // Initialize a tracker and bring it to Tracking status
    auto batch = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.0f)});

    for (int i = 0; i < 10; ++i) {
        batch.timestamp_ns = (i + 1) * 1000000;
        manager.update_all(batch);
    }

    auto all      = manager.all_outputs();
    auto tracking = all | is_tracking;

    TrackerOutputs tracking_outputs;
    for (const auto& output : tracking) {
        tracking_outputs.push_back(output);
    }

    for (const auto& output : tracking_outputs) {
        EXPECT_TRUE(output.is_tracking());
    }
}

TEST_F(TrackerManagerTest, PipeableFilter_CompositionWorks) {
    using namespace fcs::L3::filters;

    TrackerManager manager(config_);

    // Initialize Hero/Red tracker
    auto batch = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.0f)});

    for (int i = 0; i < 10; ++i) {
        batch.timestamp_ns = (i + 1) * 1000000;
        manager.update_all(batch);
    }

    // Chain filters: all -> tracking -> red
    auto all      = manager.all_outputs();
    auto filtered = all | is_tracking | with_color(ArmorColor::Red);

    TrackerOutputs results;
    for (const auto& output : filtered) {
        results.push_back(output);
    }

    for (const auto& output : results) {
        EXPECT_TRUE(output.is_tracking());
        EXPECT_EQ(output.target_color, ArmorColor::Red);
    }
}

TEST_F(TrackerManagerTest, PipeableFilter_WithStatusWorks) {
    using namespace fcs::L3::filters;

    TrackerManager manager(config_);

    auto all       = manager.all_outputs();
    auto idle_only = all | with_status(TrackerStatus::Idle);

    TrackerOutputs idle_outputs;
    for (const auto& output : idle_only) {
        idle_outputs.push_back(output);
    }

    for (const auto& output : idle_outputs) {
        EXPECT_EQ(output.status, TrackerStatus::Idle);
    }
}

TEST_F(TrackerManagerTest, PipeableFilter_IsRobotWorks) {
    using namespace fcs::L3::filters;

    TrackerManager manager(config_);

    // Initialize Hero tracker (Robot, not Outpost)
    auto batch = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.0f)});
    manager.update_all(batch);

    auto all    = manager.all_outputs();
    auto robots = all | is_robot;

    TrackerOutputs robot_outputs;
    for (const auto& output : robots) {
        robot_outputs.push_back(output);
    }

    for (const auto& output : robot_outputs) {
        EXPECT_TRUE(output.is_robot());
        EXPECT_FALSE(output.is_outpost());
    }
}

TEST_F(TrackerManagerTest, PipeableFilter_WithTargetWorks) {
    using namespace fcs::L3::filters;

    TrackerManager manager(config_);

    // Initialize One/Red tracker first
    auto batch = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.0f)});
    manager.update_all(batch);

    auto all      = manager.all_outputs();
    auto hero_red = all | with_target(ArmorName::One, ArmorColor::Red);

    TrackerOutputs results;
    for (const auto& output : hero_red) {
        results.push_back(output);
    }

    EXPECT_EQ(results.size(), 1);
    EXPECT_EQ(results[0].target_name, ArmorName::One);
    EXPECT_EQ(results[0].target_color, ArmorColor::Red);
}

// ============================================================================
// Per-Tracker Timestamp Tests
// ============================================================================

TEST_F(TrackerManagerTest, PerTrackerTimestampsAreIndependent) {
    TrackerManager manager(config_);

    // Update Hero/Red at t=1ms
    auto batch1         = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.0f)});
    batch1.timestamp_ns = 1000000;
    manager.update_all(batch1);

    // Update Two/Blue at t=5ms
    auto batch2 = create_batch({create_measurement(ArmorName::Two, ArmorColor::Blue, 1.0f)});
    batch2.timestamp_ns = 5000000;
    manager.update_all(batch2);

    // Update Hero/Red again at t=10ms
    auto batch3         = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.5f)});
    batch3.timestamp_ns = 10000000;
    auto outputs3       = manager.update_all(batch3);

    // Hero/Red should have dt = 9ms (10ms - 1ms)
    // Two/Blue tracker should still be at 5ms timestamp
    ASSERT_EQ(outputs3.size(), 1);
    EXPECT_EQ(outputs3[0].target_name, ArmorName::One);
}

// ============================================================================
// TrackerOutput Color Field Tests
// ============================================================================

TEST_F(TrackerManagerTest, TrackerOutput_ContainsColorField) {
    TrackerManager manager(config_);

    const auto batch = create_batch({create_measurement(ArmorName::One, ArmorColor::Red, 1.0f)});

    const auto outputs = manager.update_all(batch);

    ASSERT_EQ(outputs.size(), 1);
    EXPECT_EQ(outputs[0].target_color, ArmorColor::Red);
}

TEST_F(TrackerManagerTest, TrackerOutput_ColorMatchesMeasurement) {
    TrackerManager manager(config_);

    struct TestCase {
        ArmorName name;
        ArmorColor color;
    };

    std::vector<TestCase> test_cases = {
        {  ArmorName::One,  ArmorColor::Red},
        {  ArmorName::Two, ArmorColor::Blue},
        {ArmorName::Three,  ArmorColor::Red}
    };

    for (const auto& test : test_cases) {
        const auto batch = create_batch({create_measurement(test.name, test.color, 1.0f)});

        const auto outputs = manager.update_all(batch);

        ASSERT_EQ(outputs.size(), 1);
        EXPECT_EQ(outputs[0].target_name, test.name);
        EXPECT_EQ(outputs[0].target_color, test.color);
    }
}

} // namespace fcs::L3::testing
