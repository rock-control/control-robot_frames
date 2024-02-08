#include "RobotFrames.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <algorithm>
#include <base-logging/Logging.hpp>
#include <kdl/frames_io.hpp>

namespace robot_frames
{

TransformationCalculator::TransformationCalculator() : output_only_valid_(false) {
    init_blacklist();
}

void TransformationCalculator::output_only_valid(bool arg)
{
    output_only_valid_ = arg;
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

void TransformationCalculator::load_robot_model(std::string filepath, bool init_invalid){
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
#ifdef KDL_USE_NEW_TREE_INTERFACE
        segment = it->second->segment;
#else
        segment = it->second.segment;
#endif
        joint = segment.getJoint();
        std::string j_name = joint.getName();

        if(j_name == "NoName" || j_name == ""){
            if(segment.getName() != root_name){
                LOG_ERROR("Segment %s has an unnamed joint. This is not okay, but will skip it for now.", segment.getName().c_str());
            }
            LOG_DEBUG("Skipping NoName joint of root segment");
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

    //Populate transforms map with identity transforms
    for(std::vector<std::string>::iterator it = moving_joint_names_.begin();
        it != moving_joint_names_.end(); ++it)
    {
        std::string j_name = *it;
        std::string seg_name = joint_name2seg_name_[j_name];
        moving_joints_transforms_[j_name] = base::samples::RigidBodyState(true);
        if(!init_invalid){
            //initUnknown sets transform to identity. If we don't want invalidating, set to identity.
            moving_joints_transforms_[j_name].initUnknown();
        }
        moving_joints_transforms_[j_name].sourceFrame = seg_name;
#ifdef KDL_USE_NEW_TREE_INTERFACE
        moving_joints_transforms_[j_name].targetFrame = get_tree_element(seg_name).parent->second->segment.getName();
#else
        moving_joints_transforms_[j_name].targetFrame = get_tree_element(seg_name).parent->second.segment.getName();
#endif
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
#ifdef KDL_USE_NEW_TREE_INTERFACE
        rbs.targetFrame = tree_elem.parent->second->segment.getName();
#else
        rbs.targetFrame = tree_elem.parent->second.segment.getName();
#endif
        rbs.sourceFrame = tree_elem.segment.getName();
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
        LOG_INFO("Joint %s was already added to blacklist earlier.", j_name.c_str());
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
                throw("Unknown frame name. Should not be thrown at this point, but earlier in get_segment.");

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
    std::vector<base::samples::RigidBodyState> v = extract_values(moving_joints_transforms_);
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
        if(output_only_valid_ && moving_joints_transform_it->second.time.isNull())
            return false;

        transform =  moving_joints_transform_it->second;
        return true;
    }

    static_joints_transform_it = static_joints_transforms_.find(j_name);
    if(static_joints_transform_it != static_joints_transforms_.end()){
        transform =  static_joints_transform_it->second;
        return true;
    }

    LOG_ERROR("Could not find transform for joint %s", j_name.c_str());
    transform.invalidate();
    return false;
}


ChainTransformationCalculator::ChainTransformationCalculator(const std::vector<robot_frames::Chain>& chain, const std::string& urdf):Chains_(chain), urdf_file_(urdf){
    
    std::vector<robot_frames::Chain> chain_definitions = Chains_;
    n_defined_chains_ = chain_definitions.size();
    if(!n_defined_chains_){
        LOG_WARN_S << "No chains are defined! This component will do nothing..." << std::endl;
    }

    std::string urdf_file_path = urdf_file_;

    bool st;
    st = kdl_parser::treeFromFile(urdf_file_path, tree_);
    if(!st){
        LOG_ERROR_S << "Error parsing urdf file: " << urdf_file_path << std::endl;
        throw std::invalid_argument("Invalid URDF file");
    }

    //Resize stuff
    clear_and_resize_vectors();

    //Receive Chains from urdf
    for(size_t i=0; i<n_defined_chains_; i++){
        std::string root = chain_definitions[i].root_link;

        //By entering '__base__' as root or tip link, one could refer to the actual base-link of the robot model
        if(root == "__base__"){
            root = tree_.getRootSegment()->first;
        }
        std::string tip = chain_definitions[i].tip_link;
        if(tip == "__base__"){
            tip = tree_.getRootSegment()->first;
        }

        std::string name = chain_definitions[i].name;

        st = tree_.getChain(root, tip, chains_[i]);
        if(!st){
            LOG_ERROR("Error extracting chain '%s' with root '%s' and tip '%s'. The urdf_file is: %s",
                      name.c_str(), root.c_str(), tip.c_str(), urdf_file_path.c_str());
            throw std::runtime_error("Error extracting chain");
        }
        chain_names_[i] = name;
    }

    //Determine invloved joints, prepare frames storage and solvers
    for(size_t i=0; i<n_defined_chains_; i++){
        KDL::Chain chain = chains_[i];
        //Determine involved joints for each chain
        for(uint s=0; s<chain.segments.size(); s++){
            KDL::Segment segment = chain.segments[s];
            std::string jname = segment.getJoint().getName();

            if(segment.getJoint().getType() == KDL::Joint::None){
                LOG_DEBUG("Skipping joint %s of chain %s. Joint is fixed.",
                          jname.c_str(),
                          chain_names_[i].c_str());
                continue;
            }

            involved_active_joints_[i].push_back(jname);
        }
        //Resize joint arrays
        joint_arrays_[i].resize(involved_active_joints_[i].size());
        joint_arrays_[i].data.setZero();

        //Prepare frames storage
        base::samples::RigidBodyState rbs;
        if(chain_definitions[i].tip_link_renamed == ""){
            rbs.sourceFrame = chain_definitions[i].tip_link;
        }
        else{
            rbs.sourceFrame = chain_definitions[i].tip_link_renamed;
        }
        if(chain_definitions[i].root_link_renamed == ""){
            rbs.targetFrame = chain_definitions[i].root_link;
        }
        else{
            rbs.targetFrame = chain_definitions[i].root_link_renamed;
        }
        rbs.invalidate();
        bt_frames_[i] = rbs;

        //Prepare solver
        pos_solvers_[i] = new KDL::ChainFkSolverPos_recursive(chains_[i]);
    }
    LOG_INFO("ChainTransformationCalculator initilized successfully");
}

void ChainTransformationCalculator::clear_and_resize_vectors(){

    chains_.clear();
    chains_.resize(n_defined_chains_);

    chain_names_.clear();
    chain_names_.resize(n_defined_chains_);

    for(size_t i=0; i<pos_solvers_.size(); i++){
        delete pos_solvers_[i];
    }
    pos_solvers_.clear();
    pos_solvers_.resize(n_defined_chains_);

    kdl_frames_.clear();
    kdl_frames_.resize(n_defined_chains_);

    bt_frames_.clear();
    bt_frames_.resize(n_defined_chains_);

    joint_arrays_.clear();
    joint_arrays_.resize(n_defined_chains_);

    involved_active_joints_.clear();
    involved_active_joints_.resize(n_defined_chains_);

}

bool ChainTransformationCalculator::unpack_joints(const base::samples::Joints& joint_state,
                    const std::vector<std::string>& involved_joints,
                    KDL::JntArray& joint_array)
{

    assert(involved_joints.size() == joint_array.rows());

    bool ok = true;
    for(size_t i = 0; i<involved_joints.size(); i++){
        try{
            base::JointState js = joint_state.getElementByName(involved_joints[i]);
            joint_array(i) = js.position;
        }
        catch(base::samples::Joints::InvalidName ex){
            LOG_ERROR("Could not find joint %s in joint state vector.", involved_joints[i].c_str());
            ok = false;
        }
    }

    return ok;
}

void ChainTransformationCalculator::update_transforms(base::samples::Joints jointstate, 
                                std::vector<base::samples::RigidBodyState>& btframes)
{
        joint_state_ = jointstate;

        int st;
        //Go thourgh chains
        for(size_t i=0; i<n_defined_chains_; i++){
            KDL::ChainFkSolverPos_recursive* solver = pos_solvers_[i];

            //Extract joint array for chain
            if(!unpack_joints(joint_state_, involved_active_joints_[i], joint_arrays_[i]))
                throw std::runtime_error("Unable to correctly unpack joint values on kinematic chain");

            //Calculate
            st = solver->JntToCart(joint_arrays_[i], kdl_frames_[i]);
            if(st < 0){
                LOG_ERROR_S << "Something went wrong solving forward kineamtics for chain " << chain_names_[i] << std::endl;
            }

            //Convert and write to port
            convert(kdl_frames_[i], bt_frames_[i]);
            bt_frames_[i].time = joint_state_.time;
            btframes = bt_frames_;

        }

}

std::vector<std::string> ChainTransformationCalculator::get_chain_names(){
    return chain_names_;
}

int ChainTransformationCalculator::get_number_chains(){
    return n_defined_chains_;
}


}
