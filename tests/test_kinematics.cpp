#include "kinematics/manipulator2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <chrono>
#include <cmath>
#include <iostream>

using namespace kinematics;
using Catch::Matchers::WithinAbs;

TEST_CASE("Manipulator2D Constructor Validation", "[manipulator2d]") {
    SECTION("Valid link lengths") {
        REQUIRE_NOTHROW(Manipulator2D(1.0, 1.0));
        Manipulator2D robot(1.5, 2.5);
        REQUIRE(robot.getL1() == 1.5);
        REQUIRE(robot.getL2() == 2.5);
    }

    SECTION("Invalid link lengths throw exception") {
        REQUIRE_THROWS_AS(Manipulator2D(0.0, 1.0), std::invalid_argument);
        REQUIRE_THROWS_AS(Manipulator2D(1.0, -0.5), std::invalid_argument);
        REQUIRE_THROWS_AS(Manipulator2D(-1.0, -1.0), std::invalid_argument);
    }
}

TEST_CASE("Forward Kinematics - Known Geometric Positions", "[manipulator2d][fk]") {
    const double l1 = 1.0;
    const double l2 = 0.8;
    Manipulator2D robot(l1, l2);

    constexpr double TOL = 1e-6;

    SECTION("θ1 = 0, θ2 = 0 (Fully Extended along X-axis)") {
        JointState state{0.0, 0.0};
        EndEffectorPose pose = robot.computeFK(state);
        REQUIRE_THAT(pose.x, WithinAbs(l1 + l2, TOL));
        REQUIRE_THAT(pose.y, WithinAbs(0.0, TOL));
    }

    SECTION("θ1 = 0, θ2 = π/2 (L-shape along X/Y)") {
        JointState state{0.0, M_PI_2};
        EndEffectorPose pose = robot.computeFK(state);
        REQUIRE_THAT(pose.x, WithinAbs(l1, TOL));
        REQUIRE_THAT(pose.y, WithinAbs(l2, TOL));
    }

    SECTION("θ1 = π/2, θ2 = 0 (Fully Extended along Y-axis)") {
        JointState state{M_PI_2, 0.0};
        EndEffectorPose pose = robot.computeFK(state);
        REQUIRE_THAT(pose.x, WithinAbs(0.0, TOL));
        REQUIRE_THAT(pose.y, WithinAbs(l1 + l2, TOL));
    }

    SECTION("θ1 = π/4, θ2 = -π/2") {
        JointState state{M_PI_4, -M_PI_2};
        EndEffectorPose pose = robot.computeFK(state);
        double expected_x = (l1 + l2) / std::sqrt(2.0);
        double expected_y = (l1 - l2) / std::sqrt(2.0);
        REQUIRE_THAT(pose.x, WithinAbs(expected_x, TOL));
        REQUIRE_THAT(pose.y, WithinAbs(expected_y, TOL));
    }
}

TEST_CASE("Inverse Kinematics - Known Positions & Solutions", "[manipulator2d][ik]") {
    const double l1 = 1.0;
    const double l2 = 1.0;
    Manipulator2D robot(l1, l2);

    constexpr double TOL = 1e-6;

    SECTION("Point (2, 0) - Boundary Extended") {
        EndEffectorPose target{2.0, 0.0};
        
        IKResult res_up = robot.computeIK(target, ElbowConfig::ELBOW_UP);
        REQUIRE(res_up.status == IKStatus::SUCCESS);
        REQUIRE_THAT(res_up.joint_state.theta1, WithinAbs(0.0, TOL));
        REQUIRE_THAT(res_up.joint_state.theta2, WithinAbs(0.0, TOL));

        IKResult res_down = robot.computeIK(target, ElbowConfig::ELBOW_DOWN);
        REQUIRE(res_down.status == IKStatus::SUCCESS);
        REQUIRE_THAT(res_down.joint_state.theta1, WithinAbs(0.0, TOL));
        REQUIRE_THAT(res_down.joint_state.theta2, WithinAbs(0.0, TOL));
    }

    SECTION("Point (1, 1) - Elbow Up vs Elbow Down") {
        EndEffectorPose target{1.0, 1.0};

        IKResult res_up = robot.computeIK(target, ElbowConfig::ELBOW_UP);
        REQUIRE(res_up.status == IKStatus::SUCCESS);
        REQUIRE(res_up.joint_state.theta2 > 0.0);
        EndEffectorPose fk_up = robot.computeFK(res_up.joint_state);
        REQUIRE_THAT(fk_up.x, WithinAbs(target.x, TOL));
        REQUIRE_THAT(fk_up.y, WithinAbs(target.y, TOL));

        IKResult res_down = robot.computeIK(target, ElbowConfig::ELBOW_DOWN);
        REQUIRE(res_down.status == IKStatus::SUCCESS);
        REQUIRE(res_down.joint_state.theta2 < 0.0);
        EndEffectorPose fk_down = robot.computeFK(res_down.joint_state);
        REQUIRE_THAT(fk_down.x, WithinAbs(target.x, TOL));
        REQUIRE_THAT(fk_down.y, WithinAbs(target.y, TOL));
    }
}

TEST_CASE("FK/IK Round-Trip Accuracy Verification", "[manipulator2d][roundtrip]") {
    const double l1 = 1.2;
    const double l2 = 0.9;
    Manipulator2D robot(l1, l2);

    constexpr double TOL = 1e-6;

    for (double t1 = -M_PI_2; t1 <= M_PI_2; t1 += M_PI / 6.0) {
        for (double t2 = -M_PI + 0.1; t2 <= M_PI - 0.1; t2 += M_PI / 6.0) {
            JointState orig_state{t1, t2};
            EndEffectorPose pose = robot.computeFK(orig_state);

            // Test IK recovery for Elbow Up
            IKResult res_up = robot.computeIK(pose, ElbowConfig::ELBOW_UP);
            REQUIRE(res_up.status == IKStatus::SUCCESS);
            EndEffectorPose rec_up = robot.computeFK(res_up.joint_state);
            REQUIRE_THAT(rec_up.x, WithinAbs(pose.x, TOL));
            REQUIRE_THAT(rec_up.y, WithinAbs(pose.y, TOL));

            // Test IK recovery for Elbow Down
            IKResult res_down = robot.computeIK(pose, ElbowConfig::ELBOW_DOWN);
            REQUIRE(res_down.status == IKStatus::SUCCESS);
            EndEffectorPose rec_down = robot.computeFK(res_down.joint_state);
            REQUIRE_THAT(rec_down.x, WithinAbs(pose.x, TOL));
            REQUIRE_THAT(rec_down.y, WithinAbs(pose.y, TOL));
        }
    }
}

TEST_CASE("Reachability Bounds & Out-of-Bounds Detection", "[manipulator2d][ik][bounds]") {
    const double l1 = 1.0;
    const double l2 = 0.5;
    Manipulator2D robot(l1, l2);

    SECTION("Point outside maximum reach") {
        EndEffectorPose target{1.6, 0.0};
        IKResult res = robot.computeIK(target);
        REQUIRE(res.status == IKStatus::OUT_OF_REACH);
    }

    SECTION("Point inside minimum reach") {
        EndEffectorPose target{0.2, 0.0};
        IKResult res = robot.computeIK(target);
        REQUIRE(res.status == IKStatus::OUT_OF_REACH);
    }

    SECTION("Diagonal point beyond reach") {
        EndEffectorPose target{1.2, 1.2};
        IKResult res = robot.computeIK(target);
        REQUIRE(res.status == IKStatus::OUT_OF_REACH);
    }

    SECTION("Non-finite coordinates (NaN and Inf)") {
        double nan_val = std::numeric_limits<double>::quiet_NaN();
        double inf_val = std::numeric_limits<double>::infinity();

        REQUIRE(robot.computeIK(EndEffectorPose{nan_val, 1.0}).status == IKStatus::OUT_OF_REACH);
        REQUIRE(robot.computeIK(EndEffectorPose{1.0, nan_val}).status == IKStatus::OUT_OF_REACH);
        REQUIRE(robot.computeIK(EndEffectorPose{inf_val, 1.0}).status == IKStatus::OUT_OF_REACH);
        REQUIRE(robot.computeIK(EndEffectorPose{1.0, -inf_val}).status == IKStatus::OUT_OF_REACH);
    }
}

TEST_CASE("Singularity & Jacobian Analysis", "[manipulator2d][singularity]") {
    const double l1 = 1.0;
    const double l2 = 1.0;
    Manipulator2D robot(l1, l2);

    SECTION("Origin singularity for equal link lengths (l1 == l2)") {
        EndEffectorPose target{0.0, 0.0};
        IKResult res = robot.computeIK(target);
        REQUIRE(res.status == IKStatus::SINGULARITY);
    }

    SECTION("Jacobian determinant zero at fully extended configuration") {
        JointState extended_state{0.0, 0.0};
        double det = robot.computeJacobianDeterminant(extended_state);
        REQUIRE_THAT(det, WithinAbs(0.0, 1e-6));
        REQUIRE(robot.isSingular(extended_state));
    }

    SECTION("Jacobian determinant non-zero at bent configuration") {
        JointState bent_state{0.0, M_PI_2};
        double det = robot.computeJacobianDeterminant(bent_state);
        REQUIRE_THAT(det, WithinAbs(l1 * l2, 1e-6));
        REQUIRE_FALSE(robot.isSingular(bent_state));
    }
}

TEST_CASE("Performance Latency Micro-Benchmark", "[manipulator2d][benchmark]") {
    Manipulator2D robot(1.0, 1.0);
    constexpr int N = 100000;

    // FK Micro-benchmark
    auto start_fk = std::chrono::high_resolution_clock::now();
    double dummy_sum_fk = 0.0;
    for (int i = 0; i < N; ++i) {
        JointState state{0.1 + i * 0.00001, 0.2 - i * 0.00001};
        EndEffectorPose pose = robot.computeFK(state);
        dummy_sum_fk += pose.x + pose.y;
    }
    auto end_fk = std::chrono::high_resolution_clock::now();
    double avg_fk_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_fk - start_fk).count() / static_cast<double>(N);

    // IK Micro-benchmark
    auto start_ik = std::chrono::high_resolution_clock::now();
    double dummy_sum_ik = 0.0;
    for (int i = 0; i < N; ++i) {
        EndEffectorPose target{0.5 + (i % 500) * 0.001, 0.5 + (i % 500) * 0.001};
        IKResult res = robot.computeIK(target);
        dummy_sum_ik += res.joint_state.theta1;
    }
    auto end_ik = std::chrono::high_resolution_clock::now();
    double avg_ik_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_ik - start_ik).count() / static_cast<double>(N);

    std::cout << "[Benchmark] FK Avg Latency: " << avg_fk_ns << " ns/call (" << (1e9 / avg_fk_ns) / 1e6 << " M ops/sec)\n";
    std::cout << "[Benchmark] IK Avg Latency: " << avg_ik_ns << " ns/call (" << (1e9 / avg_ik_ns) / 1e6 << " M ops/sec)\n";

    // Prevent compiler optimizations
    REQUIRE(dummy_sum_fk != 0.0);
    REQUIRE(dummy_sum_ik != 0.0);

    // Verify sub-microsecond native latency (< 10000 ns per call under emulation/debug)
    REQUIRE(avg_fk_ns < 10000.0);
    REQUIRE(avg_ik_ns < 10000.0);
}
