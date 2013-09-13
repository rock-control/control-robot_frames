#include <iostream>
#include <robot_frames/RobotFrames.hpp>
#include <exception>
#include <iostream>
#include <sstream>

#define PRODUCER_TASK_NAME "robot_frames"

void make_ruby_static_transformer_cfg(robot_frames::TransformationCalculator& calc){
    std::vector<base::samples::RigidBodyState> transforms;
    calc.get_all_static_transforms(transforms);

    std::stringstream ss;

    ss.precision(5);
    ss << std::fixed;
    base::samples::RigidBodyState t;
    for(uint i=0; i<transforms.size(); i++){
        t = transforms[i];
        ss << "static_transform Eigen::Vector3.new(" <<
              t.position.x() << ", " << t.position.y() << ", " << t.position.z() <<
              "),\tEigen::Quaternion.new(" <<
              t.orientation.x() << ", " << t.orientation.y() << ", " << t.orientation.z() << ", " << t.orientation.w() << "),\t"<<
              "\""<<t.sourceFrame << "\" => \"" << t.targetFrame << "\"" << std::endl;
    }
    std::cout << ss.str() << std::endl;
}

void make_ruby_dynamic_transformer_cfg(robot_frames::TransformationCalculator& calc){
    std::stringstream ss;

    std::vector<std::string> joint_names = calc.get_all_joint_names();
    base::samples::RigidBodyState t;
    for(uint i=0; i<joint_names.size(); i++){
        calc.get_transform_by_joint_name(joint_names[i], t);
        ss << "dynamic_transform \"" << PRODUCER_TASK_NAME << "." << joint_names[i] << "\",\t" <<
              "\"" << t.sourceFrame << "\" => " << "\"" << t.targetFrame << "\"" << std::endl;
    }

    std::cout << ss.str() << std::endl;
}

void make_ruby_transformer_cfg(robot_frames::TransformationCalculator& calc){
    make_ruby_static_transformer_cfg(calc);
    make_ruby_dynamic_transformer_cfg(calc);
}


void make_syskit_robot_frames(robot_frames::TransformationCalculator& calc){
    std::vector<std::string> segment_names, tmp;
    segment_names = calc.get_all_segment_names();
    tmp = calc.get_all_static_segment_names();

    segment_names.insert(segment_names.end(), tmp.begin(), tmp.end());

    std::stringstream ss;
    ss << "robot_frames = [";
    for(uint i=0; i<segment_names.size(); i++){
        ss << "'" << segment_names[i]<<"'";
        if(i<segment_names.size()-1)
            ss << ", ";
        else
            ss << "]";
    }
    ss << std::endl;

    std::cout << ss.str() << std::endl;
}

void make_syskit_dynamic_joints(robot_frames::TransformationCalculator& calc){
    std::vector<std::string> joint_names = calc.get_all_joint_names();

    std::stringstream ss;
    ss << "robot_dynamic_joints = {";
    std::string seg_name;
    std::string parent_name;
    for(uint i=0; i< joint_names.size(); i++){
        seg_name = calc.get_segment_name_from_joint_name(joint_names[i]);
        parent_name = calc.get_parent_name_by_segment_name(seg_name);
        ss << "'" << joint_names[i] << "' => " << "['" << parent_name << "', '" << seg_name << "']";
        if(i<joint_names.size()-1)
            ss << ", " << std::endl;
        else
            ss << "}" << std::endl;
    }

    std::cout << ss.str() << std::endl;
}

void make_syskit_robot_model_cfg(robot_frames::TransformationCalculator& calc){
    make_syskit_robot_frames(calc);
    make_syskit_dynamic_joints(calc);
    make_ruby_static_transformer_cfg(calc);
}



int main(int argc, char** argv)
{
    std::string urdf_file;
    if(argc < 2 ){
        std::cerr << "You must provide an urdf file as input argument. No argument was given." << std::endl;
        return -1;
    }
    else{
        urdf_file = argv[1];
    }

    robot_frames::TransformationCalculator calc;

    try{
        calc.load_robot_model(urdf_file);
    }
    catch(std::exception& ex){
        std::cerr << "Error loading the provided URDF file "<<urdf_file<<". \nException:"<<ex.what()<<std::endl;
    }

    //make_ruby_transformer_cfg(calc);
    make_syskit_robot_model_cfg(calc);
    return 0;
}
