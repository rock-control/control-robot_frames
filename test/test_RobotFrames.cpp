#include <boost/test/unit_test.hpp>
#include <robot_frames/RobotFrames.hpp>

using namespace robot_frames;

struct Fixture {
    Fixture() {
        // set up fixture
        std::ostringstream ss;
        ss << std::getenv("ROCK_PREFIX") << "/../robot_frames/test/kuka_lbr_no_meshes.urdf";
        std::string urdf_path = ss.str();

        try {
            calc.load_robot_model(urdf_path, false);
        }
        catch (std::runtime_error ex) {
            std::ostringstream ss2;
            ss2 << "Exception while initializing from urdf file: " << urdf_path;
            BOOST_FAIL(ss2.str());
        }

        base::samples::Joints joints;
        joints.time = base::Time::fromMicroseconds(1234);
        joints.names.push_back("kuka_lbr_l_joint_1");
        joints.elements.push_back(base::JointState::Position(-1.1804315909695302));
        joints.names.push_back("kuka_lbr_l_joint_2");
        joints.elements.push_back(base::JointState::Position(1.1215726284323944));
        joints.names.push_back("kuka_lbr_l_joint_3");
        joints.elements.push_back(base::JointState::Position(0.6500091381982209));
        joints.names.push_back("kuka_lbr_l_joint_4");
        joints.elements.push_back(base::JointState::Position(-0.8300540209911509));
        joints.names.push_back("kuka_lbr_l_joint_5");
        joints.elements.push_back(base::JointState::Position(-0.41944077655932693));
        joints.names.push_back("kuka_lbr_l_joint_6");
        joints.elements.push_back(base::JointState::Position(-0.01006051707455561));
        joints.names.push_back("kuka_lbr_l_joint_7");
        joints.elements.push_back(base::JointState::Position(1.2400760596614133));
        joints.names.push_back("kuka_lbr_r_joint_1");
        joints.elements.push_back(base::JointState::Position(0.0));
        joints.names.push_back("kuka_lbr_r_joint_2");
        joints.elements.push_back(base::JointState::Position(0.0));
        joints.names.push_back("kuka_lbr_r_joint_3");
        joints.elements.push_back(base::JointState::Position(0.0));
        joints.names.push_back("kuka_lbr_r_joint_4");
        joints.elements.push_back(base::JointState::Position(0.0));
        joints.names.push_back("kuka_lbr_r_joint_5");
        joints.elements.push_back(base::JointState::Position(0.0));
        joints.names.push_back("kuka_lbr_r_joint_6");
        joints.elements.push_back(base::JointState::Position(0.0));
        joints.names.push_back("kuka_lbr_r_joint_7");
        joints.elements.push_back(base::JointState::Position(0.0));

        calc.update(joints);
    }
    ~Fixture() {
        // tear down fixture
    }

    robot_frames::TransformationCalculator calc;
};

BOOST_FIXTURE_TEST_SUITE( robot_frames_test_suite, Fixture )

//____________________________________________________________________________//

BOOST_AUTO_TEST_CASE(test_get_all_joint_names)
{
    std::vector<std::string> expected_joint_names;
    expected_joint_names.push_back("kuka_lbr_center_to_back_left_corner");
    expected_joint_names.push_back("kuka_lbr_center_to_back_right_corner");
    expected_joint_names.push_back("kuka_lbr_front_right_corner_to_cam_calib_marker");
    expected_joint_names.push_back("kuka_lbr_base_to_center");
    expected_joint_names.push_back("kuka_lbr_front_right_corner_to_collision_object");
    expected_joint_names.push_back("kuka_lbr_center_to_front_left_camera");
    expected_joint_names.push_back("kuka_lbr_center_to_front_left_corner");
    expected_joint_names.push_back("kuka_lbr_center_to_front_right_camera");
    expected_joint_names.push_back("kuka_lbr_center_to_front_right_corner");
    expected_joint_names.push_back("kuka_lbr_l_joint_0");
    expected_joint_names.push_back("kuka_lbr_l_joint_1");
    expected_joint_names.push_back("kuka_lbr_l_joint_2");
    expected_joint_names.push_back("kuka_lbr_l_joint_3");
    expected_joint_names.push_back("kuka_lbr_l_joint_4");
    expected_joint_names.push_back("kuka_lbr_l_joint_5");
    expected_joint_names.push_back("kuka_lbr_l_joint_6");
    expected_joint_names.push_back("kuka_lbr_l_joint_7");
    expected_joint_names.push_back("kuka_lbr_l_to_tcp");
    expected_joint_names.push_back("kuka_lbr_r_joint_0");
    expected_joint_names.push_back("kuka_lbr_r_joint_1");
    expected_joint_names.push_back("kuka_lbr_r_joint_2");
    expected_joint_names.push_back("kuka_lbr_r_joint_3");
    expected_joint_names.push_back("kuka_lbr_r_joint_4");
    expected_joint_names.push_back("kuka_lbr_r_joint_5");
    expected_joint_names.push_back("kuka_lbr_r_joint_6");
    expected_joint_names.push_back("kuka_lbr_r_joint_7");
    expected_joint_names.push_back("kuka_lbr_r_to_tcp");
    expected_joint_names.push_back("kuka_lbr_back_right_corner_to_sensor_mount");
    expected_joint_names.push_back("kuka_lbr_center_to_top_right_camera");

    std::vector<std::string> joint_names = calc.get_all_joint_names();

    BOOST_CHECK_EQUAL_COLLECTIONS(joint_names.begin(), joint_names.end(),
                                  expected_joint_names.begin(), expected_joint_names.end());
}

BOOST_AUTO_TEST_CASE(test_get_moving_joint_names)
{
    std::vector<std::string> expected_joint_names;
    expected_joint_names.push_back("kuka_lbr_l_joint_1");
    expected_joint_names.push_back("kuka_lbr_l_joint_2");
    expected_joint_names.push_back("kuka_lbr_l_joint_3");
    expected_joint_names.push_back("kuka_lbr_l_joint_4");
    expected_joint_names.push_back("kuka_lbr_l_joint_5");
    expected_joint_names.push_back("kuka_lbr_l_joint_6");
    expected_joint_names.push_back("kuka_lbr_l_joint_7");
    expected_joint_names.push_back("kuka_lbr_r_joint_1");
    expected_joint_names.push_back("kuka_lbr_r_joint_2");
    expected_joint_names.push_back("kuka_lbr_r_joint_3");
    expected_joint_names.push_back("kuka_lbr_r_joint_4");
    expected_joint_names.push_back("kuka_lbr_r_joint_5");
    expected_joint_names.push_back("kuka_lbr_r_joint_6");
    expected_joint_names.push_back("kuka_lbr_r_joint_7");

    std::vector<std::string> joint_names = calc.get_moving_joint_names();

    BOOST_CHECK_EQUAL_COLLECTIONS(joint_names.begin(), joint_names.end(),
                                  expected_joint_names.begin(), expected_joint_names.end());
}

BOOST_AUTO_TEST_CASE(test_get_static_joint_names)
{
    std::vector<std::string> expected_joint_names;
    expected_joint_names.push_back("kuka_lbr_center_to_back_left_corner");
    expected_joint_names.push_back("kuka_lbr_center_to_back_right_corner");
    expected_joint_names.push_back("kuka_lbr_front_right_corner_to_cam_calib_marker");
    expected_joint_names.push_back("kuka_lbr_base_to_center");
    expected_joint_names.push_back("kuka_lbr_front_right_corner_to_collision_object");
    expected_joint_names.push_back("kuka_lbr_center_to_front_left_camera");
    expected_joint_names.push_back("kuka_lbr_center_to_front_left_corner");
    expected_joint_names.push_back("kuka_lbr_center_to_front_right_camera");
    expected_joint_names.push_back("kuka_lbr_center_to_front_right_corner");
    expected_joint_names.push_back("kuka_lbr_l_joint_0");
    expected_joint_names.push_back("kuka_lbr_l_to_tcp");
    expected_joint_names.push_back("kuka_lbr_r_joint_0");
    expected_joint_names.push_back("kuka_lbr_r_to_tcp");
    expected_joint_names.push_back("kuka_lbr_back_right_corner_to_sensor_mount");
    expected_joint_names.push_back("kuka_lbr_center_to_top_right_camera");

    std::vector<std::string> joint_names = calc.get_static_joint_names();

    BOOST_CHECK_EQUAL_COLLECTIONS(joint_names.begin(), joint_names.end(),
                                  expected_joint_names.begin(), expected_joint_names.end());
}

BOOST_AUTO_TEST_CASE(test_get_all_segment_names)
{
    std::vector<std::string> expected_segment_names;
    expected_segment_names.push_back("kuka_lbr_sensor_mount");
    expected_segment_names.push_back("kuka_lbr_center");
    expected_segment_names.push_back("kuka_lbr_back_left_corner");
    expected_segment_names.push_back("kuka_lbr_back_right_corner");
    expected_segment_names.push_back("kuka_lbr_front_left_camera");
    expected_segment_names.push_back("kuka_lbr_front_left_corner");
    expected_segment_names.push_back("kuka_lbr_front_right_camera");
    expected_segment_names.push_back("kuka_lbr_front_right_corner");
    expected_segment_names.push_back("kuka_lbr_top_right_camera");
    expected_segment_names.push_back("kuka_lbr_cam_calib_marker");
    expected_segment_names.push_back("kuka_lbr_collision_object");
    expected_segment_names.push_back("kuka_lbr_l_link_0");
    expected_segment_names.push_back("kuka_lbr_l_link_1");
    expected_segment_names.push_back("kuka_lbr_l_link_2");
    expected_segment_names.push_back("kuka_lbr_l_link_3");
    expected_segment_names.push_back("kuka_lbr_l_link_4");
    expected_segment_names.push_back("kuka_lbr_l_link_5");
    expected_segment_names.push_back("kuka_lbr_l_link_6");
    expected_segment_names.push_back("kuka_lbr_l_link_7");
    expected_segment_names.push_back("kuka_lbr_l_tcp");
    expected_segment_names.push_back("kuka_lbr_r_link_0");
    expected_segment_names.push_back("kuka_lbr_r_link_1");
    expected_segment_names.push_back("kuka_lbr_r_link_2");
    expected_segment_names.push_back("kuka_lbr_r_link_3");
    expected_segment_names.push_back("kuka_lbr_r_link_4");
    expected_segment_names.push_back("kuka_lbr_r_link_5");
    expected_segment_names.push_back("kuka_lbr_r_link_6");
    expected_segment_names.push_back("kuka_lbr_r_link_7");
    expected_segment_names.push_back("kuka_lbr_r_tcp");

    std::vector<std::string> segment_names = calc.get_all_segment_names();

    BOOST_CHECK_EQUAL_COLLECTIONS(segment_names.begin(), segment_names.end(),
                                  expected_segment_names.begin(), expected_segment_names.end());
}

BOOST_AUTO_TEST_CASE(test_get_static_segment_names)
{
    std::vector<std::string> expected_segment_names;
    expected_segment_names.push_back("kuka_lbr_back_left_corner");
    expected_segment_names.push_back("kuka_lbr_back_right_corner");
    expected_segment_names.push_back("kuka_lbr_cam_calib_marker");
    expected_segment_names.push_back("kuka_lbr_center");
    expected_segment_names.push_back("kuka_lbr_collision_object");
    expected_segment_names.push_back("kuka_lbr_front_left_camera");
    expected_segment_names.push_back("kuka_lbr_front_left_corner");
    expected_segment_names.push_back("kuka_lbr_front_right_camera");
    expected_segment_names.push_back("kuka_lbr_front_right_corner");
    expected_segment_names.push_back("kuka_lbr_l_link_0");
    expected_segment_names.push_back("kuka_lbr_l_tcp");
    expected_segment_names.push_back("kuka_lbr_r_link_0");
    expected_segment_names.push_back("kuka_lbr_r_tcp");
    expected_segment_names.push_back("kuka_lbr_sensor_mount");
    expected_segment_names.push_back("kuka_lbr_top_right_camera");

    std::vector<std::string> segment_names = calc.get_static_segment_names();

    BOOST_CHECK_EQUAL_COLLECTIONS(segment_names.begin(), segment_names.end(),
                                  expected_segment_names.begin(), expected_segment_names.end());
}

static void check_dynamic_transforms(std::vector<base::samples::RigidBodyState> transforms)
{
    /* The expected values have been obtained from the
     * ROS robot_state_publisher, with the only difference
     * in the handling of source + target frame:
     *
     *     ROS header.frame_id == Rock targetFrame
     *     ROS child_frame_id  == Rock sourceFrame
     */

    BOOST_CHECK_EQUAL(14, transforms.size());

    base::samples::RigidBodyState transform;
    size_t i = 0;
    Eigen::Affine3d expected_transform;

    /* ____________ dynamic transforms __________
     *
     * dynamic transforms have time == joint_state.time
     */
    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<   0.381,    0.925,    0.000,    0.000,
                                    -0.925,    0.381,   -0.000,    0.000,
                                    -0.000,    0.000,    1.000,    0.158,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_l_link_0");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_l_link_1");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<  -0.434,    0.901,   -0.000,    0.000,
                                    -0.000,   -0.000,    1.000,    0.000,
                                     0.901,    0.434,    0.000,    0.203,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_l_link_1");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_l_link_2");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<  -0.796,    0.605,   -0.000,    0.000,
                                    -0.000,   -0.000,    1.000,    0.204,
                                     0.605,    0.796,    0.000,    0.000,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_l_link_2");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_l_link_3");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<   0.675,    0.738,    0.000,    0.000,
                                    -0.000,    0.000,   -1.000,    0.000,
                                    -0.738,    0.675,    0.000,    0.215,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_l_link_3");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_l_link_4");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<  -0.913,   -0.407,   -0.000,    0.000,
                                    -0.000,    0.000,    1.000,    0.184,
                                    -0.407,    0.913,   -0.000,    0.000,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_l_link_4");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_l_link_5");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<   1.000,    0.010,    0.000,    0.000,
                                    -0.000,    0.000,   -1.000,    0.000,
                                    -0.010,    1.000,    0.000,    0.215,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_l_link_5");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_l_link_6");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<  -0.325,    0.946,   -0.000,    0.000,
                                     0.000,    0.000,    1.000,    0.081,
                                     0.946,    0.325,   -0.000,    0.000,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_l_link_6");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_l_link_7");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,    0.000,
                                     0.000,    1.000,    0.000,    0.000,
                                     0.000,    0.000,    1.000,    0.158,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_r_link_0");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_r_link_1");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<  -1.000,    0.000,   -0.000,    0.000,
                                    -0.000,   -0.000,    1.000,    0.000,
                                    -0.000,    1.000,    0.000,    0.203,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_r_link_1");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_r_link_2");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<  -1.000,    0.000,   -0.000,    0.000,
                                    -0.000,   -0.000,    1.000,    0.204,
                                    -0.000,    1.000,    0.000,    0.000,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_r_link_2");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_r_link_3");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,    0.000,
                                     0.000,    0.000,   -1.000,    0.000,
                                     0.000,    1.000,    0.000,    0.215,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_r_link_3");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_r_link_4");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<  -1.000,    0.000,   -0.000,    0.000,
                                     0.000,    0.000,    1.000,    0.184,
                                     0.000,    1.000,   -0.000,    0.000,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_r_link_4");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_r_link_5");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,    0.000,
                                     0.000,    0.000,   -1.000,    0.000,
                                     0.000,    1.000,    0.000,    0.215,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_r_link_5");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_r_link_6");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(1234));
    expected_transform.matrix() <<  -1.000,    0.000,   -0.000,    0.000,
                                     0.000,    0.000,    1.000,    0.081,
                                     0.000,    1.000,   -0.000,    0.000,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_r_link_6");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_r_link_7");
    ++i;
}

static void check_static_transforms(std::vector<base::samples::RigidBodyState> transforms)
{
    /* The expected values have been obtained from the
     * ROS robot_state_publisher, with the only difference
     * in the handling of source + target frame:
     *
     *     ROS header.frame_id == Rock targetFrame
     *     ROS child_frame_id  == Rock sourceFrame
     */

    BOOST_CHECK_EQUAL(15, transforms.size());

    base::samples::RigidBodyState transform;
    size_t i = 0;
    Eigen::Affine3d expected_transform;

    /* ____________ static transforms __________
     *
     * static transforms have time == 0
     */
    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,   -0.022,
                                     0.000,    1.000,    0.000,    0.058,
                                     0.000,    0.000,    1.000,    0.000,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_back_right_corner");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_sensor_mount");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,    0.000,
                                     0.000,    1.000,    0.000,    0.000,
                                     0.000,    0.000,    1.000,    0.900,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_base");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_center");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,   -0.365,
                                     0.000,    1.000,    0.000,    0.583,
                                     0.000,    0.000,    1.000,    0.000,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_center");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_back_left_corner");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,   -0.365,
                                     0.000,    1.000,    0.000,   -0.583,
                                     0.000,    0.000,    1.000,    0.000,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_center");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_back_right_corner");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   0.000,   -0.296,    0.955,    0.365,
                                     0.000,    0.955,    0.296,    0.100,
                                    -1.000,    0.000,    0.000,   -0.200,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_center");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_front_left_camera");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,    0.365,
                                     0.000,    1.000,    0.000,    0.583,
                                     0.000,    0.000,    1.000,    0.000,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_center");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_front_left_corner");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   0.000,    0.296,    0.955,    0.365,
                                    -0.000,    0.955,   -0.296,   -0.100,
                                    -1.000,    0.000,    0.000,   -0.200,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_center");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_front_right_camera");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,    0.365,
                                     0.000,    1.000,    0.000,   -0.583,
                                     0.000,    0.000,    1.000,    0.000,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_center");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_front_right_corner");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   0.711,   -0.560,    0.426,   -0.365,
                                    -0.703,   -0.550,    0.451,   -0.504,
                                    -0.018,   -0.620,   -0.785,    0.963,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_center");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_top_right_camera");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,   -0.148,
                                     0.000,    0.000,   -1.000,    0.106,
                                     0.000,    1.000,    0.000,    0.000,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_front_right_corner");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_cam_calib_marker");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,   -0.050,
                                     0.000,    1.000,    0.000,    0.050,
                                     0.000,    0.000,    1.000,    0.000,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_front_right_corner");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_collision_object");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,   -0.192,
                                     0.000,    1.000,    0.000,    0.406,
                                     0.000,    0.000,    1.000,   -0.002,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_center");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_l_link_0");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,    0.000,
                                     0.000,    1.000,    0.000,    0.000,
                                     0.000,    0.000,    1.000,    0.045,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_l_link_7");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_l_tcp");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<  -0.000,   -1.000,    0.000,   -0.039,
                                     0.000,   -0.000,   -1.000,   -0.591,
                                     1.000,   -0.000,    0.000,   -0.267,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_center");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_r_link_0");
    ++i;

    transform = transforms[i];
    BOOST_CHECK_EQUAL(transform.time, base::Time::fromMicroseconds(0));
    expected_transform.matrix() <<   1.000,    0.000,    0.000,    0.000,
                                     0.000,    1.000,    0.000,    0.000,
                                     0.000,    0.000,    1.000,    0.045,
                                     0.000,    0.000,    0.000,    1.000;
    BOOST_CHECK_EQUAL(transform.targetFrame, "kuka_lbr_r_link_7");
    BOOST_CHECK_EQUAL(transform.sourceFrame, "kuka_lbr_r_tcp");
    ++i;
}

BOOST_AUTO_TEST_CASE(test_get_moving_joints_transforms)
{
    std::vector<base::samples::RigidBodyState> transforms;
    calc.get_moving_joints_transforms(transforms);
    check_dynamic_transforms(transforms);
}

BOOST_AUTO_TEST_CASE(test_get_static_joints_transforms)
{
    std::vector<base::samples::RigidBodyState> transforms;
    calc.get_static_joints_transforms(transforms);
    check_static_transforms(transforms);
}

BOOST_AUTO_TEST_CASE(test_get_all_transforms)
{
    std::vector<base::samples::RigidBodyState> transforms;
    calc.get_all_transforms(transforms);
    BOOST_CHECK_EQUAL(29, transforms.size());   // 14 dynamic, then 15 static

    // split into two
    std::vector<base::samples::RigidBodyState> dynamic_transforms(transforms.begin(), transforms.begin() + 14);
    std::vector<base::samples::RigidBodyState> static_transforms(transforms.begin() + 14, transforms.end());

    check_dynamic_transforms(dynamic_transforms);
    check_static_transforms(static_transforms);
}


// TODO: calc.get_parent_name_by_segment_name();
// TODO: calc.get_segment_by_joint();
// TODO: calc.get_segment_by_segment_name();
// TODO: calc.get_segment_name_from_joint_name();
// TODO: calc.get_transform_by_joint_name();
// TODO: calc.get_tree_element();

// TODO: calc.is_fixed(joint);
// TODO: calc.is_fixed(segment);
// TODO: calc.is_valid_joint();
// TODO: calc.is_valid_joint_name();
// TODO: calc.is_valid_joint_type();
// TODO: calc.knownJoint();

// TODO: calc.init_blacklist();
// TODO: calc.add_to_blacklist();
// TODO: calc.clear_blacklist();
// TODO: calc.set_blacklist();

BOOST_AUTO_TEST_SUITE_END()
