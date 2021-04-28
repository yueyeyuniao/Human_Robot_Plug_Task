#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "kalman_common.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "kalman_tracking_node");
    //ros::Rate loop_rate(60);

    
    //ros
    tf::TransformListener listener_;
    tf::StampedTransform transform_tip;
    tf::StampedTransform socket_tf;
    tf::Transform tf_socket_updated;
    tf::TransformBroadcaster br_socket;

    //kalman filter
    int state_num = 7;
    int measurement_size = 7;
    double dt = 1.0/30; 
    Eigen::MatrixXd A(state_num, state_num);
    Eigen::MatrixXd C(measurement_size, state_num);
    Eigen::MatrixXd Q(state_num, state_num);
    Eigen::MatrixXd R(measurement_size, measurement_size);
    Eigen::MatrixXd P0(state_num, state_num);

    A = Eigen::MatrixXd::Identity(state_num, state_num);
    C = Eigen::MatrixXd::Identity(measurement_size, measurement_size);
    Q << Eigen::MatrixXd::Identity(state_num, state_num) * 0.1;
    R << Eigen::MatrixXd::Identity(measurement_size, measurement_size) * 1.0;
    //initial measurement covariance P0
    P0 << 0.05, 0.005, 0.005, 0.0, 0.0, 0.0, 0.0,
          0.005, 0.05, 0.005, 0.0, 0.0, 0.0, 0.0,
          0.005, 0.005, 0.05, 0.0, 0.0, 0.0, 0.0,
          0.0,   0.0,   0.0,  0.05, 0.005, 0.005, 0.005,
          0.0,   0.0,   0.0,  0.005, 0.05, 0.005, 0.005,
          0.0,   0.0,   0.0,  0.005, 0.005, 0.05, 0.005,
          0.0,   0.0,   0.0,  0.005, 0.005, 0.005, 0.05;

    // P0 << 0.1, 0.01, 0.01, 0.0, 0.0, 0.0, 0.0,
    //       0.01, 0.1, 0.01, 0.0, 0.0, 0.0, 0.0,
    //       0.01, 0.01, 0.1, 0.0, 0.0, 0.0, 0.0,
    //       0.0,   0.0,   0.0,  0.1, 0.01, 0.01, 0.01,
    //       0.0,   0.0,   0.0,  0.01, 0.1, 0.01, 0.01,
    //       0.0,   0.0,   0.0,  0.01, 0.01, 0.1, 0.01,
    //       0.0,   0.0,   0.0,  0.01, 0.01, 0.01, 0.1;

    KalmanFilter kf(dt, A, C, Q, R, P0);

    Eigen::VectorXd measurement(state_num);
    //initial guess
    Eigen::VectorXd x0(state_num);
    ros::Time last_time = ros::Time::now();
    x0 << 0.6493, 0.0768, 0.4910, 0.7017, 0.7074, -0.0287, -0.0109; 
    kf.init(0, x0);

    std::cout <<"Initial guess of state is :\n" << kf.get_state().transpose() <<std::endl;
    while(ros::ok()) {
        label:
        try {
            listener_.waitForTransform("base_link", "socket", ros::Time(0), ros::Duration(1.0));
            listener_.lookupTransform("base_link", "socket", ros::Time(0), socket_tf);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            tf_socket_updated.setOrigin(tf::Vector3(0.6493, 0.0768, 0.4910));
            tf::Quaternion q_1(0.7017, 0.7074, -0.0287, -0.0109);
            tf_socket_updated.setRotation(q_1);
            br_socket.sendTransform(tf::StampedTransform(tf_socket_updated, ros::Time::now(), "base_link", "socket_updated"));

            tf_socket_updated.setOrigin(tf::Vector3(0.5493, 0.0768, 0.4910));
            tf::Quaternion q_pre_1(0.7017, 0.7074, -0.0287, -0.0109);
            tf_socket_updated.setRotation(q_pre_1);
            br_socket.sendTransform(tf::StampedTransform(tf_socket_updated, ros::Time::now(), "base_link", "socket_updated_pre"));
            std::cout << "Waiting for the human to provide the socket" << std::endl;
            goto label;
        }

        measurement << socket_tf.getOrigin().x(), socket_tf.getOrigin().y(), socket_tf.getOrigin().z(),
                       socket_tf.getRotation().x(), socket_tf.getRotation().y(), socket_tf.getRotation().z(), socket_tf.getRotation().w();

        dt = socket_tf.stamp_.toSec() - last_time.toSec(); //class tf::StampedTransform’ has no member named ‘toSec’
        kf.predict_and_update(measurement, dt);
        std::cout << "Time: "<<kf.get_time() << std::endl;
        std::cout << "Updated with: \n" << measurement.transpose() <<std::endl;  //‘i’ was not declared in this scope // .col(i) was deleted
        std::cout << "New state is: \n" << kf.get_state().transpose() << std::endl;
        std::cout << "=======================================" << std::endl;
        tf_socket_updated.setOrigin(tf::Vector3(kf.get_state().transpose()(0), kf.get_state().transpose()(1), kf.get_state().transpose()(2)));
        tf::Quaternion q(kf.get_state().transpose()(3), kf.get_state().transpose()(4), kf.get_state().transpose()(5), kf.get_state().transpose()(6));
        tf_socket_updated.setRotation(q);
        br_socket.sendTransform(tf::StampedTransform(tf_socket_updated, ros::Time::now(), "base_link", "socket_updated"));
        
        tf_socket_updated.setOrigin(tf::Vector3(kf.get_state().transpose()(0)-0.10, kf.get_state().transpose()(1), kf.get_state().transpose()(2)));
        tf::Quaternion q_pre(kf.get_state().transpose()(3), kf.get_state().transpose()(4), kf.get_state().transpose()(5), kf.get_state().transpose()(6));
        tf_socket_updated.setRotation(q_pre);
        br_socket.sendTransform(tf::StampedTransform(tf_socket_updated, ros::Time::now(), "base_link", "socket_updated_pre"));

        last_time = socket_tf.stamp_;
        //loop_rate.sleep();

        ros::Rate r(30);
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}