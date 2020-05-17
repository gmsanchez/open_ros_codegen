/**
 * This is an auto-generated file by Optimization Engine (OpEn)
 * OpEn is a free open-source software - see doc.optimization-engine.xyz
 * dually licensed under the MIT and Apache v2 licences.
 *
 * Generated at 2020-05-07 01:22:27.483106.
 */
#include "ros/ros.h"
#include "open_nmpc_controller/OptimizationResult.h"
#include "open_nmpc_controller/OptimizationParameters.h"
#include "mpc_controller_bindings.hpp"
#include "open_optimizer.hpp"

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"


namespace open_nmpc_controller {
/**
 * Class open_nmpc_controller::OptimizationEngineManager manages the
 * exchange of data between the input and output topics
 * of this node
 */
class OptimizationEngineManager {

private:
    open_nmpc_controller::OptimizationParameters params;
    open_nmpc_controller::OptimizationResult results;
    double p[MPC_CONTROLLER_NUM_PARAMETERS] = { 0 };
    double u[MPC_CONTROLLER_NUM_DECISION_VARIABLES] = { 0 };
    double *y = NULL;

    static const int NX = 3;
    static const int NU = 2;

    double current_pos[NX] = {0};
    double current_ref[NX] = {0};

    mpc_controllerCache* cache;
    double init_penalty = ROS_NODE_MPC_CONTROLLER_DEFAULT_INITIAL_PENALTY;

    /**
     * Publish obtained results to output topic
     */
    void publishToTopic(ros::Publisher& publisher)
    {
        publisher.publish(results);
    }

    /**
     * Updates the input data based on the data that are posted
     * on /mpc/open_parameters (copies value from topic data to
     * local variables). This method is responsible for parsing
     * the data announced on the input topic.
     */
    void updateInputData()
    {
        init_penalty = (params.initial_penalty > 1.0)
            ? params.initial_penalty
            : ROS_NODE_MPC_CONTROLLER_DEFAULT_INITIAL_PENALTY;

        if (params.parameter.size() > 0) {
            for (size_t i = 0; i < MPC_CONTROLLER_NUM_PARAMETERS; ++i)
                p[i] = params.parameter[i];
        }

        if (params.initial_guess.size() == MPC_CONTROLLER_NUM_DECISION_VARIABLES) {
            for (size_t i = 0; i < MPC_CONTROLLER_NUM_DECISION_VARIABLES; ++i)
                u[i] = params.initial_guess[i];
        }

        if (params.initial_y.size() == MPC_CONTROLLER_N1) {
            for (size_t i = 0; i < MPC_CONTROLLER_N1; ++i)
                y[i] = params.initial_y[i];
        }

    }

    /**
     * Call OpEn to solve the problem
     */
    mpc_controllerSolverStatus solve()
    {
        return mpc_controller_solve(cache, u, p, y, &init_penalty);
    }


public:
    /**
     * Constructor of OptimizationEngineManager
     */
    OptimizationEngineManager()
    {
        y = new double[MPC_CONTROLLER_N1];
        cache = mpc_controller_new();
    }

    /**
     * Destructor of OptimizationEngineManager
     */
    ~OptimizationEngineManager()
    {
        if (y!=NULL) delete[] y;
        mpc_controller_free(cache);
    }

    /**
     * Copies results from `status` to the local field `results`
     */
    void updateResults(mpc_controllerSolverStatus& status)
    {
        std::vector<double> sol(u, u + MPC_CONTROLLER_NUM_DECISION_VARIABLES);
        results.solution = sol;
        std::vector<double> y(status.lagrange, status.lagrange + MPC_CONTROLLER_N1);
        results.lagrange_multipliers = y;
        results.inner_iterations = status.num_inner_iterations;
        results.outer_iterations = status.num_outer_iterations;
        results.norm_fpr = status.last_problem_norm_fpr;
        results.penalty = status.penalty;
        results.status = (int)status.exit_status;
        results.solve_time_ms = (double)status.solve_time_ns / 1000000.0;
        results.infeasibility_f2 = status.f2_norm;
        results.infeasibility_f1 = status.delta_y_norm_over_c;
    }

    /**
     * Callback that obtains data from topic `/open_nmpc_controller/open_params`
     */
    void mpcReceiveRequestCallback(
        const open_nmpc_controller::OptimizationParameters::ConstPtr& msg)
    {
        params = *msg;
    }

    void solveAndPublish(ros::Publisher& publisher)
    {
        updateInputData(); /* get input data */
        mpc_controllerSolverStatus status = solve(); /* solve!  */
        updateResults(status); /* pack results into `results` */
        publishToTopic(publisher);
    }

    void commandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        current_ref[0] = msg->pose.position.x;
        current_ref[1] = msg->pose.position.y;
        current_ref[2] = msg->pose.orientation.w;
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        double roll, pitch, yaw;

        tf::Quaternion quat(msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z,
                            msg->pose.pose.orientation.w);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        current_pos[0] = msg->pose.pose.position.x;
        current_pos[1] = msg->pose.pose.position.y;
        current_pos[2] = yaw;
    }

    void solveAndPublishCmdVel(ros::Publisher& publisher)
    {
        double current_par [MPC_CONTROLLER_NUM_PARAMETERS] = {0};
        double current_var [MPC_CONTROLLER_NUM_DECISION_VARIABLES] = {0};
        double lin_vel_cmd, ang_vel_cmd = 0;

        for (int i=0; i<NX; i++) {
            current_par[i] = current_pos[i];
            current_par[i+NX] = current_ref[i];
        }

        /* solve                  */
        mpc_controllerSolverStatus status
            = mpc_controller_solve(cache, current_var, current_par, 0, &init_penalty);

        lin_vel_cmd = current_var[0];
        ang_vel_cmd = current_var[1];

        geometry_msgs::Twist twist;
        twist.linear.x = lin_vel_cmd;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = ang_vel_cmd;
        publisher.publish(twist);

        ROS_INFO("x: %f, y: %f, yaw: %f", current_pos[0], current_pos[1], current_pos[2]);
        ROS_INFO("Solve time: %f ms. I will send %f %f \n",
                 (double)status.solve_time_ns / 1000000.0, lin_vel_cmd, ang_vel_cmd);

    }

}; /* end of class OptimizationEngineManager */

} /* end of namespace open_nmpc_controller */

/**
 * Main method
 *
 * This advertises a new (private) topic to which the optimizer
 * announces its solution and solution status and details. The
 * publisher topic is 'open_nmpc_controller/result'.
 *
 * It obtains inputs from 'open_nmpc_controller/parameters'.
 *
 */
int main(int argc, char** argv)
{

    std::string result_topic, params_topic;  /* parameter and result topics */
    std::string out_twist, in_odometry;
    double rate; /* rate of node (specified by parameter) */

    open_nmpc_controller::OptimizationEngineManager mng;
    ros::init(argc, argv, ROS_NODE_MPC_CONTROLLER_NODE_NAME);
    ros::NodeHandle nh, private_nh("~");

    /* obtain parameters from config/open_params.yaml file */
    private_nh.param("result_topic", result_topic,
                     std::string(ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC));
    private_nh.param("params_topic", params_topic,
                     std::string(ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC));
    private_nh.param("rate", rate,
                     double(ROS_NODE_MPC_CONTROLLER_RATE));

    private_nh.param("out_twist_name", out_twist, std::string("/husky_velocity_controller/cmd_vel"));
    private_nh.param("in_odom_name", in_odometry, std::string("/odometry/filtered"));

    ros::Publisher mpc_pub
        = private_nh.advertise<open_nmpc_controller::OptimizationResult>(
            result_topic,
            ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC_QUEUE_SIZE);
    ros::Subscriber sub
        = private_nh.subscribe(
            params_topic,
            ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC_QUEUE_SIZE,
            &open_nmpc_controller::OptimizationEngineManager::mpcReceiveRequestCallback,
            &mng);
    ros::Subscriber pos_sub
        = nh.subscribe(in_odometry,
                       1,
                       &open_nmpc_controller::OptimizationEngineManager::odometryCallback,
                       &mng);
    ros::Subscriber command_trajectory_subscriber
        = private_nh.subscribe("command/pose",
                               1,
                               &open_nmpc_controller::OptimizationEngineManager::commandPoseCallback,
                               &mng);
    ros::Publisher pub_twist_cmd = nh.advertise<geometry_msgs::Twist>(out_twist, 1);
    ros::Rate loop_rate(rate);

    while (ros::ok()) {
        mng.solveAndPublishCmdVel(pub_twist_cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
