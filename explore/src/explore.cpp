/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
    double dx = one.x - two.x;
    double dy = one.y - two.y;
    double dist = sqrt(dx * dx + dy * dy);
    return dist < 0.01;
}

namespace explore
{
    Explore::Explore()
            : private_nh_("~")
            , tf_listener_(ros::Duration(10.0))
            , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
            , move_base_client_("move_base")
            , prev_distance_(0)
            , last_markers_count_(0)
    {
        double timeout;
        double min_frontier_size;
        private_nh_.param("planner_frequency", planner_frequency_, 1.0);
        private_nh_.param("progress_timeout", timeout, 30.0);
        progress_timeout_ = ros::Duration(timeout);
        private_nh_.param("visualize", visualize_, false);
        private_nh_.param("potential_scale", potential_scale_, 1e-3);
        private_nh_.param("orientation_scale", orientation_scale_, 0.0);
        private_nh_.param("gain_scale", gain_scale_, 1.0);
        private_nh_.param("min_frontier_size", min_frontier_size, 0.5);

        search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                       potential_scale_, gain_scale_,
                                                       min_frontier_size);

        if (visualize_) {
            marker_array_publisher_ =
                    private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
        }

        ROS_INFO("Waiting to connect to move_base server");
        move_base_client_.waitForServer();
        ROS_INFO("Connected to move_base server");

        exploring_timer_ =
                relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                                         [this](const ros::TimerEvent&) {
                    tag_pos_subcriber_ = relative_nh_.subscribe("/apriltag_pose", 1, &Explore::makePlan, this);});
    }

    Explore::~Explore()
    {
        stop();
    }


    void Explore::makePlan(const geometry_msgs::Pose &a)
    {
        // find frontiers
        geometry_msgs::Point point = a.position;

        // get frontiers sorted according to cost
        auto frontiers = search_.searchFrom(point);
        // find non blacklisted frontier
        auto frontier =
                std::find_if_not(frontiers.begin(), frontiers.end(),
                                 [this](const frontier_exploration::Frontier& f) {
                                     return goalOnBlacklist(f.centroid);
                                 });;

        geometry_msgs::Point target_position = frontier->centroid;


        // send goal to move_base if we have something new to pursue
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose.position = target_position;
        goal.target_pose.pose.orientation.w = 1.;
        goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
        goal.target_pose.header.stamp = ros::Time::now();
        move_base_client_.sendGoal(
                goal, [this, target_position](
                        const actionlib::SimpleClientGoalState& status,
                        const move_base_msgs::MoveBaseResultConstPtr& result) {
                    reachedGoal(status, result, target_position);
                });
    }

    bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
    {
        constexpr static size_t tolerace = 5;
        costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

        // check if a goal is on the blacklist for goals that we're pursuing
        for (auto& frontier_goal : frontier_blacklist_) {
            double x_diff = fabs(goal.x - frontier_goal.x);
            double y_diff = fabs(goal.y - frontier_goal.y);

            if (x_diff < tolerace * costmap2d->getResolution() &&
                y_diff < tolerace * costmap2d->getResolution())
                return true;
        }
        return false;
    }

    void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                              const move_base_msgs::MoveBaseResultConstPtr&,
                              const geometry_msgs::Point& frontier_goal)
    {
        ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
        if (status == actionlib::SimpleClientGoalState::ABORTED) {
            frontier_blacklist_.push_back(frontier_goal);
            ROS_DEBUG("Adding current goal to black list");
        }

        // find new goal immediatelly regardless of planning frequency.
        // execute via timer to prevent dead lock in move_base_client (this is
        // callback for sendGoal, which is called in makePlan). the timer must live
        // until callback is executed.
        oneshot_ = relative_nh_.createTimer(
                ros::Duration(0, 0), [this](const ros::TimerEvent&) {
                    tag_pos_subcriber_ = relative_nh_.subscribe("/apriltag_pose", 1, &Explore::makePlan, this);},
                true);
    }

    void Explore::start()
    {
        exploring_timer_.start();
    }

    void Explore::stop()
    {
        move_base_client_.cancelAllGoals();
        exploring_timer_.stop();
        ROS_INFO("Exploration stopped.");
    }

}  // namespace explore

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    explore::Explore explore;
    ros::spin();

    return 0;
}
