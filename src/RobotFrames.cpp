#include "RobotFrames.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <algorithm>
#include <base/Logging.hpp>
#include <kdl/frames_io.hpp>

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
    it = std::find (moving_joint_names_.begin(), moving_joint_names_.end(), j_name);

    //find returns iterator to las element in search if nothing was found
    return it !=moving_joint_names_.end();
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
    KDL::SegmentMap map = kdl_tree_.getSegments();
    std::string root_name = kdl_tree_.getRootSegment()->first;

    KDL::Joint joint;
    KDL::Segment segment;
    for(KDL::SegmentMap::const_iterator it = map.begin(); it!=map.end(); ++it){
        segment = it->second.segment;
        joint = it->second.segment.getJoint();
        std::string j_name = joint.getName();

        if(j_name == "NoName" || j_name == ""){
            if(segment.getName() != root_name){
                LOG_ERROR("Segment %s has an unnamed joint. This is not okay, but will skip it for now.", segment.getName().c_str());
            }
            LOG_DEBUG("Skipping NonName joint of root segment");
            continue;
        }

        if(is_fixed(joint)){
            if(std::find(static_joint_names_.begin(), static_joint_names_.end(), j_name) == static_joint_names_.end()){
                static_segment_names_.push_back(segment.getName());
                static_joint_names_.push_back(j_name);
                joint_name2seg_name_[j_name] = segment.getName();
                all_joint_names_.push_back(j_name);
            }
        }
        if(is_valid_joint(joint)){
            if(std::find(moving_joint_names_.begin(), moving_joint_names_.end(), j_name) == moving_joint_names_.end()){
                moving_joint_names_.push_back(j_name);
                joint_name2seg_name_[j_name] = segment.getName();
                all_joint_names_.push_back(j_name);
            }
        }

    }
    //link_names_ = extract_keys(map);

    //Populate transforms map with invalid transforms
    for(std::vector<std::string>::iterator it = moving_joint_names_.begin();
        it != moving_joint_names_.end(); ++it)
    {
        std::string j_name = *it;
        std::string seg_name = joint_name2seg_name_[j_name];
        moving_joints_transforms_[j_name] = base::samples::RigidBodyState(true);
        moving_joints_transforms_[j_name].sourceFrame = get_tree_element(seg_name).parent->second.segment.getName();
        moving_joints_transforms_[j_name].targetFrame = seg_name;
    }

    //Populate static transforms vector
    for(uint i=0; i<static_joint_names_.size(); i++){
        std::string seg_name = static_segment_names_[i];
        KDL::TreeElement tree_elem = get_tree_element(seg_name);
        KDL::Frame kdl_transform = tree_elem.segment.pose(0);
        std::string j_name = static_joint_names_[i];
        base::samples::RigidBodyState rbs;
        convert(kdl_transform, rbs);
        //Skip root
        if(tree_elem.segment.getName() == kdl_tree_.getRootSegment()->first){
            continue;
        }
        rbs.sourceFrame = tree_elem.parent->second.segment.getName();
        rbs.targetFrame = tree_elem.segment.getName();
        static_joints_transforms_[j_name] = rbs;
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

        if(is_invalid(j_state.position)){
            LOG_ERROR("Received invalid joint position for joint %s", j_name.c_str());
            continue;
        }

        try{
            //Get segment
            seg_name = joint_name2seg_name_[j_name];
            seg = get_segment_by_segment_name(seg_name);

            //Calculate pose
            kdl_pose = seg.pose(j_state.position);

            //Convert transform to eigen
            transform_it = moving_joints_transforms_.find(j_name);
            if(transform_it == moving_joints_transforms_.end())
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

bool TransformationCalculator::get_static_joints_transforms(std::vector<base::samples::RigidBodyState>& transforms, bool keep_content)
{
    if(!keep_content)
        transforms.clear();
    std::vector<base::samples::RigidBodyState> v = extract_values(static_joints_transforms_);
    transforms.insert(transforms.end(), v.begin(), v.end());
    return true;
}

bool TransformationCalculator::get_moving_joints_transforms(std::vector<base::samples::RigidBodyState>& transforms, bool keep_content)
{
    if(!keep_content)
        transforms.clear();
    std::vector<base::samples::RigidBodyState> v = extract_values(static_joints_transforms_);
    transforms.insert(transforms.end(), v.begin(), v.end());
    return true;
}

bool TransformationCalculator::get_all_transforms(std::vector<base::samples::RigidBodyState>& transforms, bool keep_content)
{
    if(!keep_content)
        transforms.clear();
    get_moving_joints_transforms(transforms, keep_content);
    get_static_joints_transforms(transforms, true);
    return true;
}

bool TransformationCalculator::get_transform_by_joint_name(const std::string& j_name, base::samples::RigidBodyState& transform)
{
    std::map<std::string,base::samples::RigidBodyState>::iterator moving_joints_transform_it;
    std::map<std::string,base::samples::RigidBodyState>::iterator static_joints_transform_it;

    moving_joints_transform_it = moving_joints_transforms_.find(j_name);
    if(moving_joints_transform_it != moving_joints_transforms_.end()){
        transform =  moving_joints_transform_it->second;
        return true;
    }

    static_joints_transform_it = static_joints_transforms_.find(j_name);
    if(static_joints_transform_it != static_joints_transforms_.end()){
        transform =  static_joints_transform_it->second;
        return true;
    }

    LOG_ERROR("Ciulod not find transform for joint %s", j_name.c_str());
    transform.invalidate();
    return false;
}

}
