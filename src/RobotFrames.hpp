#ifndef _ROBOTFRAMETRANSFORMATIONS_ROBOTFRAMETRANSFORMATIONS_HPP_
#define _ROBOTFRAMETRANSFORMATIONS_ROBOTFRAMETRANSFORMATIONS_HPP_

#include <iostream>
#include <string>
#include <map>
#include <Eigen/Geometry>
#include "base/samples/Joints.hpp"
#include "base/samples/RigidBodyState.hpp"
#include "kdl/tree.hpp"

template<typename F,typename T>
inline std::vector<F> extract_keys(std::map<F,T> m){
    std::vector<F> v;
    for(typename std::map<F,T>::iterator it = m.begin(); it != m.end(); ++it) {
        v.push_back(it->first);
    }
    return v;
}

template<typename F, typename T>
inline std::vector<T> extract_values(std::map<F,T> m){
    std::vector<T> v;
    for(typename std::map<F,T>::iterator it = m.begin(); it != m.end(); ++it) {
        v.push_back(it->second);
    }
    return v;
}

template<typename T>
inline bool is_invalid(T val){
    return base::isInfinity(val) || base::isNaN(val);
}

inline void convert(const KDL::Frame& from, base::Pose& to){
    double x,y,z,w;
    from.M.GetQuaternion(x,y,z,w);
    to.orientation.x() = x;
    to.orientation.y() = y;
    to.orientation.z() = z;
    to.orientation.w() = w;

    to.position.x() =  from.p.x();
    to.position.y() =  from.p.y();
    to.position.z() =  from.p.z();
}

inline void convert(const KDL::Frame& from, base::samples::RigidBodyState& to){
    base::Pose pose;
    convert(from, pose);
    to.setPose(pose);
}


namespace robot_frames
{

class TransformationCalculator
{
public:
    TransformationCalculator();
    /**
     * Load the model file. Throws exception if file not found
     * or model could not be loaded
     *
     * \param filepath : Path to the robot model definition file
     * \return nothing
     */
    void load_robot_model(std::string filepath);

    bool is_valid_joint_name(std::string j_name);
    bool is_valid_joint_type(const KDL::Joint& joint);
    bool is_valid_joint(const KDL::Joint& joint);

    void set_blacklist(const std::vector<std::string>& blacklist);
    void init_blacklist();
    void clear_blacklist();
    void add_to_blacklist(std::string j_name);

    /**
     * Set state of all the joint of the robot, or just a subset of them.
     * Calculate transforms for the given joints
     *
     * \param joints : Joint state to set.
     */
    void update(const base::samples::Joints& joints);

    /**
     * Get transforms
     *
     * \return true on success, false otherwise
     */
    bool get_all_transforms(std::vector<base::samples::RigidBodyState>& transforms, bool keep_content=false);
    bool get_transform_by_joint_name(const std::string& j_name, base::samples::RigidBodyState& transform);

    bool get_moving_joints_transforms(std::vector<base::samples::RigidBodyState>& transforms, bool keep_content=false);
    bool get_static_joints_transforms(std::vector<base::samples::RigidBodyState>& transforms, bool keep_content=false);

    /**
     * \return true, when segment is known from robot model file, false when not.
     */
    bool knownJoint(const std::string& link_name);

    /**
     * Retrieve segment by name.
     * \throws exception when segment is not known
     * \return reference to segment
     */
    inline const KDL::TreeElement& get_tree_element(const std::string& segment_name){
        KDL::SegmentMap::const_iterator elem = kdl_tree_.getSegment(segment_name);
        if(elem == kdl_tree_.getSegments().end())
            throw(std::runtime_error(segment_name));

        return elem->second;
    }

    inline const KDL::Segment& get_segment_by_segment_name(const std::string& segment_name){
        return get_tree_element(segment_name).segment;
    }

    inline const std::string& get_segment_name_from_joint_name(const std::string& joint_name){
        std::map<std::string, std::string>::iterator it;
        it = joint_name2seg_name_.find(joint_name);
        if(it == joint_name2seg_name_.end())
            throw(std::runtime_error(joint_name));
        return it->second;
    }

    inline std::vector<std::string> get_all_segment_names(){
        return extract_values(joint_name2seg_name_);
    }

    inline std::vector<std::string> get_static_segment_names(){
        return static_segment_names_;
    }

    inline const std::vector<std::string>& get_moving_joint_names(){
        return moving_joint_names_;
    }

    inline const std::vector<std::string>& get_static_joint_names(){
        return static_joint_names_;
    }

    inline const std::vector<std::string>& get_all_joint_names(){
        return all_joint_names_;
    }

    inline const KDL::Segment& get_segment_by_joint(const KDL::Joint joint){
        return get_segment_by_segment_name(
                    get_segment_name_from_joint_name(joint.getName()));
    }

    inline const std::string& get_parent_name_by_segment_name(std::string seg_name){
        std::string root_name = kdl_tree_.getRootSegment()->first;
        if(seg_name == root_name)
            return "";

        KDL::TreeElement elem = get_tree_element(seg_name);
        return elem.parent->second.segment.getName();
    }


    /**
     * \return true, when link provides a fixed transform. False when not.
     */
    inline bool is_fixed(const KDL::Joint& joint){
        return joint.getType() == KDL::Joint::None;
    }

    inline bool is_fixed(const KDL::Segment& segment){
        return is_fixed(segment.getJoint());
    }

    protected:
    inline void clear_all(){
        static_segment_names_.clear();
        moving_joints_transforms_.clear();
        moving_joint_names_.clear();
        static_joint_names_.clear();
        joint_name2seg_name_.clear();
        is_initialized_=false;
        init_blacklist();
    }

    std::map<std::string, base::samples::RigidBodyState> moving_joints_transforms_;
    std::map<std::string, base::samples::RigidBodyState> static_joints_transforms_;
    std::vector<std::string> moving_joint_names_;
    std::vector<std::string> static_joint_names_;
    std::vector<std::string> all_joint_names_;
    std::vector<std::string> static_segment_names_;
    std::vector<std::string> blacklist_;
    std::map<std::string, std::string> joint_name2seg_name_;
    KDL::Tree kdl_tree_;
    bool is_initialized_;
};

} // end namespace robot_frame_transformations

#endif // _ROBOTFRAMETRANSFORMATIONS_ROBOTFRAMETRANSFORMATIONS_HPP_
