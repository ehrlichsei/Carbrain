#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
#from navigation.msg import *
from navigation_msgs.srv import *
from mission_control.srv import *

# define start state Idle
class Idle(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['begin_parking'])
    self.counter = 0

  def execute(self, userdata):
    rospy.loginfo('Executing state Idle')
    return 'begin_parking'

class ServiceState(smach.State):
  def __init__(self, servname, servclazz, serv_args = []):
    smach.State.__init__(self, outcomes=['succeeded', 'failed'])
    self._servname = servname
    self._servclazz = servclazz
    self._serv_args = serv_args

  def execute(self, userdata):
    rospy.loginfo('Executing Service state')
    parkserv = rospy.ServiceProxy(self._servname, self._servclazz)
    #rospy.wait_for_service(parkserv)
    try:
      self._service_response = parkserv(*self._serv_args)
      return 'succeeded'
    except rospy.ServiceException as exc:
      return 'failed'

def run_parking_state_machine(req):
 # Create a SMACH state machine
  sm = smach.StateMachine(outcomes=['done'])

  # Open the container
  with sm:
    # Add states to the container
    smach.StateMachine.add('Idle', Idle(),
                transitions={'begin_parking':'FindParkingSlot'})

    smach.StateMachine.add('FindParkingSlot', ServiceState('/navigation/find_parking_slot/find_slot', FindParkingSlot),
                transitions={'failed':'done',
                     'succeeded':'ExecuteParkingSequence'})

    # Calls the ROS Action 'execute_parking_sequence' which is handled by the parking node
    smach.StateMachine.add('ExecuteParkingSequence', ServiceState('drive_into_parking_slot', DriveIntoParkingSlot, [0.55, 0.28, 0.1, 0.02, 0.0]),
                            transitions={'succeeded':'done', 'failed':'done'})

  # Execute SMACH plan
  outcome = sm.execute()
  return StartParkingDisciplineResponse()

def run():
  rospy.init_node('parking_mission_control')
  rospy.loginfo("parking mission control starting up")

  s = rospy.Service('start_parking_discipline', StartParkingDiscipline, run_parking_state_machine)
  rospy.spin()


if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass
