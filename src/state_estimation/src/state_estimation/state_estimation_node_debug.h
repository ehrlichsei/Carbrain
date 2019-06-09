#ifndef STATE_ESTIMATION_NODE_DEBUG_H
#define STATE_ESTIMATION_NODE_DEBUG_H

#include "state_estimation_node.h"

THIRD_PARTY_HEADERS_BEGIN
#include "controller_msgs/StateMeasure.h"
#include "common_msgs/Float32Stamped.h"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_count.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#include <boost/optional.hpp>
THIRD_PARTY_HEADERS_END

namespace acc = boost::accumulators;

/*!
 * \brief In debug-mode StateEstimationNodeDebug will be started instead of
 * StateEstimationNode.
 * StateEstimationNodeDebug accepts state measurements by ros topic (while
 * StateEstimationNode does not), but launches
 * StateEstimation instad of StateEstimationRealtime, which doesn't allow
 * communication by shared memory.
 * This allows debugging of the state estimation with rosbags, but doesn't give
 * any real-time-guarantees.
 * StateEstimationNodeDebug also publishes the state covariance matrix, the
 * kalman gain matrix and their condition numbers.
 */
class StateEstimationNodeDebug : public StateEstimationNode {
 public:
  StateEstimationNodeDebug(ros::NodeHandle &node_handle);

  /*!
   * \brief handleMessages callback of subscribed topics
   * \param meas StateMeasure topic
   * \param fservo front servo command topic
   * \param rservo rear servo command topic
   * \param engine engine power command topic
   */
  void handleMessages(const controller_msgs::StateMeasure &meas,
                      const boost::optional<common_msgs::Float32Stamped> &fservo,
                      const boost::optional<common_msgs::Float32Stamped> &rservo,
                      const boost::optional<common_msgs::Float32Stamped> &engine);

 protected:
  /*!
   * \brief startModule is called, if the node shall be turned active. In here
   * the subrscribers an publishers are started.
   */
  virtual void startModule() override;
  /*!
   * \brief stopModule is called, if the node shall be turned inactive. In this
   * function subscribers and publishers are shutdown.
   */
  virtual void stopModule() override;

 private:
  void resetMsgCaches();

  /*!
   * \brief checkSeqNumbers outputs a warning if the messages of subscribed
   * topics are not sequential.
   */
  void seqNumberPostFilteringCheck(const controller_msgs::StateMeasure &meas,
                                   const boost::optional<common_msgs::Float32Stamped> &fservo,
                                   const boost::optional<common_msgs::Float32Stamped> &rservo,
                                   const boost::optional<common_msgs::Float32Stamped> &engine);


  /*!
   * \brief time_of_last_measurement needed for to check for time-jumps in the
   * messages (due to a restarting rosbag or missing messages)
   */
  ros::Time time_of_last_measurement;
  /*!
   * \brief time_of_last_measurement needed for to check for time-jumps in the
   * messages (due to a restarting rosbag or missing messages)
   */
  ros::Duration time_since_last_measurement;

  /*!
   * \brief state_measure_subscriber ros subscriber for the state measure topic
   */
  ros::Subscriber state_measure_sub;

  /*!
   * \brief front_servo_cmd_sub ros subscriber for the front
   * servo command topic
   */
  ros::Subscriber front_servo_cmd_sub;

  /*!
   * \brief back_servo_cmd_sub ros subscriber for the back
   * servo command topic
   */
  ros::Subscriber back_servo_cmd_sub;

  /*!
   * \brief engine_command_sub ros subscriber for the engine
   * command topic
   */
  ros::Subscriber engine_cmd_sub;
  /*!
   * \brief state_measure_cache collects state measure messages from callback
   * before message synchronization
   */
  std::deque<controller_msgs::StateMeasure> state_measure_cache;
  /*!
   * \brief front_servo_cache collects front servo command messages from
   * callback before message synchronization
   */
  std::deque<common_msgs::Float32Stamped> front_servo_cache;
  /*!
   * \brief back_servo_cache collects back servo command messages from callback
   * before message synchronization
   */
  std::deque<common_msgs::Float32Stamped> back_servo_cache;
  /*!
   * \brief engine_cache collects engine power command messages from callback
   * before message synchronization
   */
  std::deque<common_msgs::Float32Stamped> engine_cache;

  /*!
   * \brief stateMeasureCallback pre-filtering sequence number checks and
   * filling state_measure cache
   * \param msg the message
   */
  void stateMeasureCallback(const controller_msgs::StateMeasure &msg);

  /*!
   * \brief frontServoCallback pre-filtering sequence number checks and filling
   * front servo cache
   * \param msg the message
   */
  void frontServoCallback(const common_msgs::Float32Stamped &msg);
  /*!
   * \brief backServoCallback pre-filtering sequence number checks and filling
   * back servo cache
   * \param msg the message
   */
  void backServoCallback(const common_msgs::Float32Stamped &msg);
  /*!
   * \brief engineCallback pre-filtering sequence number checks and filling
   * engine cache
   * \param msg the message
   */
  void engineCallback(const common_msgs::Float32Stamped &msg);

  /*!
   * \brief sync_timer timer for message synchronization. Calls
   * syncTimerCallback in constant time intervals.
   */
  ros::Timer sync_timer;

  /*!
   * \brief syncTimerCallback callback of sync_timer. Contains the message
   * synchronization logic for the state_measure and the command topics. For
   * each state measure message from the state measure cache, the message with
   * the closest time stamp to the state measure message from each advertized
   * command topic gets selected. These messages are then passed as
   * boost::optional to handleMessages. For not advertized topics boost::none is
   * passed to handleMessages.
   */
  void syncTimerCallback(const ros::TimerEvent &);

  /*!
  * \brief kalman_update_rate_check_acc boost accumulator needed to check if the
  * publishing frequency of messages on the state measurements topic and the
  * update rate of the kalman filte are approximately the same
  */
  acc::accumulator_set<double, acc::stats<acc::tag::rolling_mean, acc::tag::rolling_count> > kalman_update_rate_check_acc;

  /*!
    * \brief accumulation_window the mean of the time_since_last_measurement of
   * the last accumulation_window messages is the estimated publishing frequency
   * of the state measurements topic
    */
  static constexpr int accumulation_window = 80;

  /*!
   * \brief check_seq_numbers toggle sequence number check
   */
  bool check_seq_numbers = false;

  /*!
   * \brief message_sync_rate_multiplier times the state_estimation update rate
   * is the rate of the message synchronization timer callback.
   */
  double message_sync_rate_multiplier = 4.0;


  /*!
   * \brief The SeqNumberCheck class checks the sequence numbers of incoming
   * topics with seqNumberPreFilteringCheck for inconsistencies (like e.g.
   * restarting rosbag) and checks the sequence numbers after synchronization to
   * evaluate the quality of the filtering/synchronization with
   * seqNumberPostFilteringCheck
   */
  class SeqNumberCheck {
   public:
    SeqNumberCheck(std::string topic_name) : topic_name(topic_name) {}

    /*!
     * \brief seqNumberPreFilteringCheck checks the sequence numbers of incoming
     * topics for inconsitencies (like e.g. restarting rosbag)
     * \param seq the sequence number.
     * \return false if sequence numbers aren't sequential, true otherwise
     */
    bool seqNumberPreFilteringCheck(uint32_t seq);

    /*!
     * \brief seqNumberPostFilteringCheck checks the sequence numbers after
     * synchronization to evaluate the quality of the filtering/synchronization
     * \param input_seq the sequence number.
     */
    void seqNumberPostFilteringCheck(uint32_t input_seq);

    void resetSeqCounter(uint32_t seq);

    bool initialized() const { return init; }

   private:
    /*!
     * \brief topic_name name of the checked topic
     */
    std::string topic_name;
    bool init = false;
    /*!
     * \brief msg_seq message sequence number counter
     */
    uint32_t msg_seq = 0;
    /*!
     * \brief init_msg_seq initial sequence number counter
     */
    uint32_t init_msg_seq = 0;
    /*!
     * \brief acc_msg_err initial sequence error accumulator (reused messages)
     */
    uint32_t acc_msg_reused_err = 0;
    /*!
     * \brief acc_msg_drop_err initial sequence error accumulator (dropped
     * messages)
     */
    uint32_t acc_msg_drop_err = 0;
    /*!
     * \brief last_seq sequence number of last received message, used for pre
     * check
     */
    uint32_t last_seq = 0;
  };
  /*!
   * \brief meas_check checks state_measure topic for sequence number
   * inconsistencies
   */
  SeqNumberCheck meas_check;
  /*!
   * \brief front_servo_check checks front servo command topic for sequence
   * number inconsistencies
   */
  SeqNumberCheck front_servo_check;
  /*!
   * \brief back_servo_check checks back servo command topic for sequence number
   * inconsistencies
   */
  SeqNumberCheck back_servo_check;
  /*!
   * \brief engine_check checks engine power command topic for sequence number
   * inconsistencies
   */
  SeqNumberCheck engine_check;
};

#endif  // STATE_ESTIMATION_NODE_DEBUG_H
