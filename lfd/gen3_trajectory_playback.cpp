#include <tesseract_core/basic_types.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>
#include <tesseract_planning/trajopt/trajopt_planner.h>

#include <urdf_parser/urdf_parser.h>

#include <trajopt/file_write_callback.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt_utils/logging.hpp>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/JointState.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "trajopt_examples/trajopt_node.h"
#include <geometry_msgs/Pose.h>



control_msgs::FollowJointTrajectoryGoal trajectory_action;
//trajectory_msgs::JointTrajectory traj_msg;

class gen3_traj_replay
{
  public:
    gen3_traj_replay(ros::NodeHandle n) : nh(n)
    {      
      nh.param<int>("steps_per_phase", steps_per_phase, 100);
      nh.param<bool>("plotting", plotting_cb, false);
      nh.param<bool>("file_write_cb", file_write_cb, true);

      /////////////
      /// SETUP ///
      /////////////

      // Pull ROS params
      ROS_INFO("%d", steps_per_phase);
      std::string urdf_xml_string, srdf_xml_string;
      nh.getParam("/robot_description", urdf_xml_string);
      nh.getParam("/robot_description_semantic", srdf_xml_string);

      // Load robot
      urdf_model = urdf::parseURDF(urdf_xml_string);
      srdf_model = srdf::ModelSharedPtr(new srdf::Model);
      srdf_model->initString(*urdf_model, srdf_xml_string);
      assert(urdf_model != nullptr);

      //Create robot model for using moveit time parameterization
      robot_model.reset(new moveit::core::RobotModel(urdf_model, srdf_model));
      ROS_INFO("Robot model loaded"); 
      // sub = nh.subscribe("...", 1000, traj_callback);
    }

    // void traj_callback(const trajectory_msgs::JointTrajectory& msg)
    // {
    //   ROS_INFO("Got the trajectory");
    // }

    void getTrajectoryOnce()
    {
      boost::shared_ptr<trajectory_msgs::JointTrajectory const> traj;
      traj = ros::topic::waitForMessage<trajectory_msgs::JointTrajectory>("/planned_trajectory",nh);
      if(traj != NULL){
        traj_msg_queue = *traj;
      }
    }

    control_msgs::FollowJointTrajectoryGoal trajArrayToJointTrajectory_moveit(std::vector<std::string> joint_names,
                                       tesseract::TrajArray traj_array,
                                       moveit::core::RobotModelPtr robot_model,
                                       bool use_time,
                                       bool interpolate,
                                       ros::Duration time_increment)

    {
      // convert to trajectory msg
      // Create the joint trajectory
      trajectory_msgs::JointTrajectory traj_msg;
      traj_msg.header.stamp = ros::Time::now();
      traj_msg.header.frame_id = "/base_link";
      traj_msg.joint_names = joint_names;

      tesseract::TrajArray pos_mat;
      tesseract::TrajArray time_mat;
      if (use_time)
      {
        // Seperate out the time data in the last column from the joint position data
        pos_mat = traj_array.leftCols(traj_array.cols()-1);
        time_mat = traj_array.rightCols(1);
      }
      else
      {
        pos_mat = traj_array;
      }

      std::cout << "how many waypoints using trajopt: " << traj_array.rows() << std::endl;
      ros::Duration time_from_start(0);
      for (int ind = 0; ind < traj_array.rows(); ind++)
      {
        // Create trajectory point
        trajectory_msgs::JointTrajectoryPoint traj_point;
        auto mat = pos_mat.row(ind);
        // Set the position for this time step
        if (interpolate)
        {
          if (ind == traj_array.rows()-1) continue;
          auto mat_next = pos_mat.row(ind+1);
          auto dist = (time_increment.toSec() / 0.001); // decrease the last number to get valid trajectory
          Eigen::MatrixXd dmat = (mat_next - mat) / dist;

          // if (dmat(0,0) < pow(10, -8) && dmat(0,1) < pow(10, -8) && dmat(0,2) < pow(10, -8) && dmat(0,3) < pow(10, -8) && dmat(0,4) < pow(10, -8) && dmat(0,5) < pow(10, -8) && dmat(0,6) < pow(10, -8))
          // {
          //   goto label;
          // }

          for (int i = 0; i < dist;i++)
          {
            Eigen::MatrixXd temp_mat = mat + i*dmat;
            std::vector<double> vec(temp_mat.data(), temp_mat.data() + temp_mat.rows() * temp_mat.cols());
            traj_point.positions = vec;
            if (use_time)
            {
              time_from_start += ros::Duration(0.001);
            }
            else
            {
              // fit the constraint
              time_from_start += ros::Duration(0.001);
            }
            traj_point.time_from_start = time_from_start;

            traj_msg.points.push_back(traj_point);
          }  
        }
        else {
          std::vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
          traj_point.positions = vec;
          //traj_point.velocities = std::vector<double>(7, 0.0);
          //traj_point.accelerations = std::vector<double>(7, 0.0);

          // Add the current dt to the time_from_start
          if (use_time)
          {
            time_from_start += ros::Duration(time_mat(ind, time_mat.cols() - 1));
          }
          else
          {
            time_from_start += time_increment;
          }
          traj_point.time_from_start = time_from_start;

          traj_msg.points.push_back(traj_point);
        }

      }
  //label:
      std::cout << traj_msg.points.size() << std::endl;

      // convert to moveit trajectory
      int num_dof = traj_msg.points[1].positions.size();
      int num_waypoints = traj_msg.points.size();

      robot_trajectory::RobotTrajectory moveit_trajectory(robot_model, "Manipulator");

      //Convert trajopt array trajectory to moveit trajectory
      const robot_model::JointModelGroup* group = moveit_trajectory.getGroup();
      if (!group)
      {
        ROS_ERROR_NAMED("trajectory_processing", "Need to set the group");
        //return;
      }

      moveit::core::RobotState state(moveit_trajectory.getRobotModel());

      //TODO: Set velocity + acceleration limits
      //Loop through all trajopt waypoints
      for (int i = 0; i < num_waypoints; i++)
      {
        std::vector<double> joint_state;

        for (int j = 0; j < num_dof; j++) {
          joint_state.push_back(traj_msg.points[i].positions[j]);
        }
        state.setVariablePositions(joint_names, joint_state);
        moveit_trajectory.addSuffixWayPoint(state, 0.001);   // it was 0.0, changed to 0.001
      }

      //Smooth trajectory
      trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
      time_parameterization.computeTimeStamps(moveit_trajectory, 0.8, 0.8);  //it was 1.0 and 1.0, changed to 0.8, 0.8

      //Convert smoothed trajectory to ROS trajectory
      moveit_msgs::RobotTrajectory smooth_traj_msg;
      moveit_trajectory.getRobotTrajectoryMsg(smooth_traj_msg);

      // modify the msg
      smooth_traj_msg.joint_trajectory.header.stamp = ros::Time::now();
      smooth_traj_msg.joint_trajectory.header.frame_id = "/base_link";
      smooth_traj_msg.joint_trajectory.joint_names = joint_names;
      for (int i = 0; i < num_waypoints; i++)
      {
        smooth_traj_msg.joint_trajectory.points[i].time_from_start = ros::Duration(0.001*i);
      }


      // hack
      for (int i = 0; i < num_waypoints; i++)
      {
        for (int j = 0; j < 7; j++){
          if (smooth_traj_msg.joint_trajectory.points[i].accelerations[j] < -0.99)
          {
            smooth_traj_msg.joint_trajectory.points[i].accelerations[j] = -0.99;
          }
          if (smooth_traj_msg.joint_trajectory.points[i].accelerations[j] > 0.99)
          {
            smooth_traj_msg.joint_trajectory.points[i].accelerations[j] = 0.99;
          }
        }
      }


      //TODO: Plot smoothed trajectory
      std::cout << "TRAJOPT: Moveit trajectory smoothing is done!\n";


      trajectory_action.trajectory = smooth_traj_msg.joint_trajectory;
      return trajectory_action;
    }

    bool gen3_play()
    {
      //Initialize the environment
      tesseract::tesseract_ros::KDLEnvPtr env(new tesseract::tesseract_ros::KDLEnv);
      assert(env != nullptr);
      bool success = env->init(urdf_model, srdf_model);
      assert(success);

      // Set the initial state of the robot
      std::unordered_map<std::string, double> joint_states;
      bool sim_robot = false;
      if (sim_robot) {
        joint_states["joint_1"] = 0.0;      // 0
        joint_states["joint_2"] = -0.35;    // 0.26
        joint_states["joint_3"] = 3.14;     // 3.14
        joint_states["joint_4"] = -2.54;    // -2.27
        joint_states["joint_5"] = 0.0;      // 0
        joint_states["joint_6"] = -0.87;    // 0.96
        joint_states["joint_7"] = 1.57;     // 1.57
      }
      else
      {
        boost::shared_ptr<const sensor_msgs::JointState> joint_pos =
            ros::topic::waitForMessage<sensor_msgs::JointState>("/my_gen3/joint_states", nh);

        joint_states["joint_1"] = joint_pos.get()->position[0];
        joint_states["joint_2"] = joint_pos.get()->position[1];
        joint_states["joint_3"] = joint_pos.get()->position[2];
        joint_states["joint_4"] = joint_pos.get()->position[3];
        joint_states["joint_5"] = joint_pos.get()->position[4];
        joint_states["joint_6"] = joint_pos.get()->position[5];
        joint_states["joint_7"] = joint_pos.get()->position[6];
      }
      env->setState(joint_states);

      for (auto i : env->getJointNames()) {
        ROS_INFO("Joint Names:%s", i.c_str());
      }

      ///////////////
      /// EXECUTE ///
      ///////////////
      // control the real robot ******************************************************************

      // ROS_ERROR("Press enter to continue");
      // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

      // Create action client to send trajectories
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execution_client("my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory",false);
      trajectory_msgs::JointTrajectory traj_msg;
      execution_client.waitForServer(ros::Duration(1.0));

      // Get data
      std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
      tesseract::TrajArray traj_array;

      int rows = traj_msg_queue.points.size();
      int cols = joint_names.size();
      traj_array.resize(rows, cols);

      // Set the starting point of the robot at the current position
      for (int j=0; j<cols; j++){
        traj_array(0,j) = joint_states[joint_names[j]];
      }

      // Assign the joint trajectories
      for (int i=1; i<rows; i++) 
      {
        for (int j=0; j<cols; j++){
          traj_array(i,j) = traj_msg_queue.points[i].positions[j];
        }
      }
      
      // std::cout << traj_array.row(1) << std::endl;

      bool result;

      ros::Duration t(0.03); // need to modify
      trajArrayToJointTrajectory_moveit(joint_names, traj_array, robot_model,false, true, t);

      ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/smoothed_trajectory", 1000);
      ros::Rate loop_rate(5);
      while (ros::ok()){
        pub.publish(trajectory_action.trajectory);
        ros::spinOnce();
        loop_rate.sleep();
      }

      // // Send to hardware
      // execution_client.sendGoal(trajectory_action);
      // execution_client.waitForResult(ros::Duration(10.0));

      
      // if (execution_client.getState() != actionlib::SimpleClientGoalState::LOST)
      //   {
      //     std::cout << "succeeded! \n";
      //     result = true;
      //   }
      //   else
      //   {
      //     std::cout << "failed \n";
      //     result = false;
      //   }

        return result;
    }

    

  private:
    ros::NodeHandle nh;
    int steps_per_phase;
    bool plotting_cb, file_write_cb;
    urdf::ModelInterfaceSharedPtr urdf_model;
    srdf::ModelSharedPtr srdf_model;
    moveit::core::RobotModelPtr robot_model;
    std::string manip = "Manipulator";
    std::string end_effector = "end_effector_link";
    // ros::Subscriber sub;
    trajectory_msgs::JointTrajectory traj_msg_queue;
    

};

int main(int argc, char **argv) {

    ros::init(argc, argv, "gen3_trajopt_node");
    ros::NodeHandle n("~");

    // Set Log Level
    util::gLogLevel = util::LevelInfo;

    gen3_traj_replay gen3_traj_replay(n);
    gen3_traj_replay.getTrajectoryOnce();
    bool result = gen3_traj_replay.gen3_play();





    ROS_INFO("Ready to replay joint trajectory for Gen3.");
    ros::spin();

    return 0;
}
