#include "RobotFrames.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <algorithm>
#include <base/Logging.hpp>

namespace robot_frames
{

TransformationCalculator::TransformationCalculator(){
    init_blacklist();
}

bool TransformationCalculator::is_valid_joint_name(std::string j_name){
    return std::find(blacklist_.begin(), blacklist_.end(), j_name) == blacklist_.end();
}

bool TransformationCalculator::is_valid_joint_type(const KDL::Joint& joint){
    if(joint.getType() == KDL::Joint::None)
        return false;
    return true;
}

bool TransformationCalculator::is_valid_joint(const KDL::Joint& joint){
    if(!is_valid_joint_type(joint))
        return false;
    if(!is_valid_joint_name(joint.getName()))
        return false;

    return true;
}

bool TransformationCalculator::knownJoint(const std::string& j_name)
{
    std::vector<std::string>::iterator it;
    it = std::find (joint_names_.begin(), joint_names_.end(), j_name);

    //find returns iterator to las element in search if nothing was found
    return it !=joint_names_.end();
}


void TransformationCalculator::load_robot_model(std::string filepath){
    clear_all();

    bool success=false;
    success = kdl_parser::treeFromFile(filepath, kdl_tree_);
    if(!success){
        is_initialized_ = false;
        throw("Could not load Model file");
    }

    //Extract names of all joints defined in urdf file.
    //These are the names that are expected to be referred to in update function.
    joint_names_.clear();
    KDL::SegmentMap map = kdl_tree_.getSegments();
    KDL::Joint joint;
    KDL::Segment segment;
    for(KDL::SegmentMap::const_iterator it = map.begin(); it!=map.end(); ++it){
        segment = it->second.segment;
        joint = it->second.segment.getJoint();
        if(is_fixed(joint)){
            static_segment_names_.push_back(segment.getName());
        }
        if(is_valid_joint(joint)){
            std::string j_name = joint.getName();
            if(std::find(joint_names_.begin(), joint_names_.end(), j_name) == joint_names_.end()){
                joint_names_.push_back(j_name);
                joint_name2seg_name_[j_name] = segment.getName();
            }
        }

    }
    //link_names_ = extract_keys(map);

    //Populate transforms map with invalid transforms
    for(std::vector<std::string>::iterator it = joint_names_.begin();
        it != joint_names_.end(); ++it)
    {
        std::string j_name = *it;
        std::string seg_name = joint_name2seg_name_[j_name];
        transforms_[j_name] = base::samples::RigidBodyState(true);
        transforms_[j_name].sourceFrame = get_tree_element(seg_name).parent->second.segment.getName();
        transforms_[j_name].targetFrame = seg_name;
    }

    //Populate static transforms vector
    for(uint i=0; i<static_segment_names_.size(); i++){
        std::string seg_name = static_segment_names_[i];
        KDL::TreeElement tree_elem = get_tree_element(seg_name);
        KDL::Frame kdl_transform = tree_elem.segment.pose(0);
        base::samples::RigidBodyState rbs;
        convert(kdl_transform, rbs);
        //Skip root
        if(tree_elem.segment.getName() == kdl_tree_.getRootSegment()->first){
            continue;
        }
        rbs.sourceFrame = tree_elem.parent->second.segment.getName();
        rbs.targetFrame = tree_elem.segment.getName();
        static_transforms_.push_back(rbs);
    }


    is_initialized_ = true;
}

void TransformationCalculator::set_blacklist(const std::vector<std::string>& blacklist)
{
    init_blacklist();
    blacklist_.insert(blacklist_.end(), blacklist.begin(), blacklist.end());
}

void TransformationCalculator::init_blacklist()
{
    blacklist_.clear();
    blacklist_.push_back("");
    blacklist_.push_back("NoName");
}

void TransformationCalculator::clear_blacklist()
{
    init_blacklist();
}

void TransformationCalculator::add_to_blacklist(std::string j_name)
{
    if(std::find(blacklist_.begin(), blacklist_.end(), j_name) != blacklist_.end()){
        LOG_INFO("Joint %s was already added to blacklist ealier.", j_name.c_str());
        return;
    }
    blacklist_.push_back(j_name);
}

void TransformationCalculator::update(const base::samples::Joints& joints)
{
    base::JointState j_state;
    std::string  j_name;
    std::string  seg_name;
    KDL::Segment seg;
    KDL::Frame kdl_pose;
    std::map<std::string,base::samples::RigidBodyState>::iterator transform_it;

    if(!is_initialized_)
        throw std::runtime_error("TransformationCalculator was not initialized. Call load_robot_model first");

    for(uint i=0; i<joints.size(); i++){
        j_state = joints.elements[i];
        j_name = joints.names[i];

        if(is_invalid(j_state))
            continue;

        try{
            //Get segment
            seg_name = joint_name2seg_name_[j_name];
            seg = get_segment_by_segment_name(seg_name);

            //Calculate pose
            kdl_pose = seg.pose(j_state.position);

            //Convert transform to eigen
            transform_it = transforms_.find(j_name);
            if(transform_it == transforms_.end())
                throw("Unknown frame name. Should be thrown at this point, but ealier in get_segment.");

            convert(kdl_pose, transform_it->second);

            //Set timestap
            transform_it->second.time = joints.time;
        }
        catch(...){
            LOG_WARN("Joint %s or segment %s are unknown", j_name.c_str(), seg_name.c_str());
            continue;
        }
    }
}

bool TransformationCalculator::get_all_static_transforms(std::vector<base::samples::RigidBodyState>& transforms)
{
    transforms = static_transforms_;
    return true;
}

bool TransformationCalculator::get_all_transforms(std::vector<base::samples::RigidBodyState>& transforms)
{
    transforms = extract_values(transforms_);
    return true;
}

bool TransformationCalculator::get_transform_by_joint_name(const std::string& j_name, base::samples::RigidBodyState& transform)
{
    std::map<std::string,base::samples::RigidBodyState>::iterator transform_it;
    transform_it = transforms_.find(j_name);
    if(transform_it == transforms_.end()){
        transform.invalidate();
        return false;
    }

    transform =  transform_it->second;
    return true;
}

}
