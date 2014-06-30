#!/usr/bin/env python
import roslib; roslib.load_manifest('core')
import rospy
import sys, traceback, threading
from utils.common import Shared, TimeoutState
from core.msg import plan, task, missionPlanStamped
from std_msgs.msg import Header

import smach

class MissionPlanner(object):
    def __init__(self, mission, shared):
        self.mission = mission
        self.sm0 = None
        self.shared = shared
        self.available_states = dict(
                ('%s_STATE' % p.name.upper(), self._load_task(p.name))
                    for p in self.mission.plan
            )
        self.pub = rospy.Publisher('currentmission', missionPlanStamped)
        self.timer = rospy.Timer(rospy.Duration(0.1), lambda d:
                                 self.get_stamped_plan())

    def _load_task(self, t):
        t = 'states.%s' % (t)
        try:
            __import__(t)
        except ImportError, e:
            rospy.logerr('error importing state: %s' % (str(e)))
            traceback.print_exc()
            cls = None
        finally:
            module = sys.modules[t]
            cls = module.State
            return cls

    def _make_individual_sm(self, _task):
        if _task.timeout > 0:
            sm = smach.Concurrence(outcomes=['succeeded', 'failed',
                                             'preempted'],
                                   child_termination_cb=lambda so: True,
                                   default_outcome='preempted',
                                   outcome_map={
                                    'succeeded' : {'TASK' : 'succeeded'},
                                    'preempted' : {'TIMEOUT' : 'succeeded'},
                                    'failed' : {'TASK' : 'failed'}
                                   })
            with sm:
                smach.Concurrence.add('TIMEOUT', TimeoutState(_task.timeout))
                smach.Concurrence.add('TASK',
                                      self.available_states['%s_STATE' %
                                                            _task.name.upper()](self.shared))
            return sm
        else:
            return self.available_states['%s_STATE' % _task.name.upper()](self.shared)

    def _make_transitions(self, _task):
        transitions = {'succeeded' : 'succeeded', 'preempted' : 'preempted',
                       'failed' : 'failed'}
        taskset = [p.name for p in self.mission.plan]
        if _task.has_fallback:
            if _task.fallback in taskset:
                transitions['failed'] = '%s_TASK' % _task.fallback
            else:
                rospy.logerr('task %s wants %s as fallback but it is not loaded' % 
                             (_task.name, _task.fallback))
        if taskset.index(_task.name) == len(taskset) - 1:
            return transitions
        else:
            n = '%s_TASK' % taskset[taskset.index(_task.name) + 1].upper()
            transitions['succeeded'] = n
            return transitions

    def make_sm(self):
        self.sm0 = smach.StateMachine(outcomes=['succeeded', 'preempted',
                                                'failed'])
        with self.sm0:
            for m in self.mission.plan:
                sm = self._make_individual_sm(m)
                smach.StateMachine.add('%s_TASK' % m.name.upper(),
                                        sm,
                                        transitions=self._make_transitions(m))
        if self.mission.master_timeout > 0:
            smcc = smach.Concurrence(outcomes=['succeeded', 'preempted',
                                               'failed'],
                                     child_termination_cb=lambda so: True,
                                     default_outcome='preempted',
                                     outcome_map={
                                        'succeeded' : {'MISSION': 'succeeded'},
                                        'failed' : {'MISSION' : 'failed'},
                                        'preempted' : {'MASTER_TIMEOUT' :
                                                       'succeeded'}
                                     })
            with smcc:
                smach.Concurrence.add('MISSION', self.sm0)
                smach.Concurrence.add('MASTER_TIMEOUT',
                                      TimeoutState(self.mission.master_timeout))
            self.sm0 = smcc

    def run(self):
        self.shared.start_time = rospy.Time.now()
        self.sm0.execute()

    def stop(self):
        rospy.logwarn('sigint caught')
        self.sm0.request_preempt()
        self.timer.shutdown()

    def get_stamped_plan(self):
        out = missionPlanStamped(
                header=Header(stamp=rospy.Time.now(),
                              frame_id='mission_plan'),
                mission=self.mission
            )
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('MissionPlanner', anonymous=False)

    robosub = plan()
    robosub.master_timeout = 60
    robosub.plan.append(task('init', 10, False, ''))
    robosub.plan.append(task('gate', 45, False, ''))

    m = MissionPlanner(robosub, Shared())
    m.make_sm()
    st = threading.Thread(target=m.run)
    st.start()
    rospy.spin()
    m.stop()
    st.join()
