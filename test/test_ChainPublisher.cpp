#include <boost/test/unit_test.hpp>
#include <robot_frames/RobotFrames.hpp>

using namespace robot_frames;

struct Fixture {
    Fixture() {
        // set up fixture
        

        joints.time = base::Time::fromMicroseconds(1234);
        joints.names.push_back("kuka_lbr_l_joint_1");
        joints.elements.push_back(base::JointState::Position(0.0));
        joints.names.push_back("kuka_lbr_l_joint_2");
        joints.elements.push_back(base::JointState::Position(0.0));
        joints.names.push_back("kuka_lbr_l_joint_3");
        joints.elements.push_back(base::JointState::Position(0.0));
        joints.names.push_back("kuka_lbr_l_joint_4");
        joints.elements.push_back(base::JointState::Position(-0.00));
        joints.names.push_back("kuka_lbr_l_joint_5");
        joints.elements.push_back(base::JointState::Position(-0.0));
        joints.names.push_back("kuka_lbr_l_joint_6");
        joints.elements.push_back(base::JointState::Position(-0.0));
        joints.names.push_back("kuka_lbr_l_joint_7");
        joints.elements.push_back(base::JointState::Position(0.0));
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


    // base -> kuka_lbr_cam_calib_marker 
    robot_frames::Chain chain1;
        chain1.name = "kuka_lbr_cam_calib_marker";
        chain1.root_link = "kuka_lbr_center";
        chain1.tip_link = "kuka_lbr_r_link_2";

    // base -> kuka_lbr_front_left_camera 
    robot_frames::Chain chain2; 
        chain2.name = "kuka_lbr_front_left_camera";
        chain2.root_link = "__base__";
        chain2.tip_link = "kuka_lbr_front_left_camera";

    // kuka_lbr_center -> kuka_lbr_r_link_1 
    robot_frames::Chain chain3;
        chain3.name = "kuka_lbr_r_link_1";
        chain3.root_link = "kuka_lbr_center";
        chain3.tip_link = "kuka_lbr_r_link_1";

    chains_1.push_back(chain1);

    chains_2.push_back(chain1);
    chains_2.push_back(chain2);

    chains_3.push_back(chain1);
    chains_3.push_back(chain2);
    chains_3.push_back(chain3);


    // urdf path 
    std::ostringstream ss;
    ss << std::getenv("ROCK_PREFIX") << "/../robot_frames/test/kuka_lbr_no_meshes.urdf";
    urdf_path = ss.str();

    }
    ~Fixture() {
        // tear down fixture
    }

    ChainTransformationCalculator* transformer;
    std::string urdf_path;

    // create chains
    std::vector<robot_frames::Chain> chains_1;
    std::vector<robot_frames::Chain> chains_2;
    std::vector<robot_frames::Chain> chains_3;

    // create joint state
    base::samples::Joints joints;

};

BOOST_FIXTURE_TEST_SUITE( Test_fixture, Fixture )

    BOOST_AUTO_TEST_CASE(should_work_with_1_chain)
    {
        std::vector<base::samples::RigidBodyState> btframes;
        ChainTransformationCalculator *transformer = new ChainTransformationCalculator(chains_1, urdf_path);
        transformer->update_transforms(joints, btframes);
    } 

    BOOST_AUTO_TEST_CASE(should_work_with_2_chain)
    {
        std::vector<base::samples::RigidBodyState> btframes;
        ChainTransformationCalculator *transformer = new ChainTransformationCalculator(chains_2, urdf_path);
        transformer->update_transforms(joints, btframes);

    }

    BOOST_AUTO_TEST_CASE(should_work_with_3_chain)
    {
        std::vector<base::samples::RigidBodyState> btframes;
        ChainTransformationCalculator *transformer = new ChainTransformationCalculator(chains_3, urdf_path);
        transformer->update_transforms(joints, btframes);

        // for(const auto btframe: btframes){
        //     std::cout << btframe.sourceFrame << "-->" << btframe.targetFrame << std::endl;
        //     std::cout << btframe.position[0] << " >=> " << btframe.position[1] << " >=> "<< btframe.position[2] << " >=> " << std::endl;
        // }
    }

BOOST_AUTO_TEST_SUITE_END()

