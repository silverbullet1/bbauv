#!/usr/bin/env python
import roslib; roslib.load_manifest('core')
import rospy
from core.msg import task, plan
import sys, traceback
import smach

from utils.common import TimeoutState

class Shared(object):
    def __init__(self):
        self.yaw = None

class MissionPlanner(object):
    def __init__(self, mission):
        self.mission = mission
        self._shared = Shared()
        self.available_states = dict(
                ('%s_STATE' % p.name.upper(), self._load_task(p.name))
                    for p in self.mission.plan
            )

    def _make_transitions(self, st):
        transitions = {'succeeded' : 'succeeded', 'failed' : 'failed',
                       'preempted' : 'preempted'}
        states = [m.name for m in self.mission.plan]
        if st.has_fallback:
            if st.fallback not in states:
                return transitions
            transitions['failed'] = self.get_state_name(st.fallback)
        if states.index(st.name) == len(states) - 1:
            return transitions
        n = states[states.index(st.name) + 1]
        transitions['succeeded'] = self.get_state_name(n)
        return transitions

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

    @staticmethod
    def get_state_name(n):
        n = '%s_STATE' % n.upper()
        return n

    def _get_state(self, st):
        sm = self.available_states[self.get_state_name(st.name)](self._shared)
        if st.timeout > 0:
            smcc = smach.Concurrence(outcomes=['succeeded', 'preempted',
                                               'failed'],
                                     child_termination_cb=lambda d: True,
                                     outcome_map={
                                        'succeeded' : {'TASK' : 'succeeded'},
                                        'failed' : {'TASK' : 'failed'},
                                        'failed' : {'TIMEOUT': 'succeeded'}
                                     },
                                     default_outcome='preempted')
            with smcc:
                smach.Concurrence.add('TASK', sm)
                smach.Concurrence.add('TIMEOUT', TimeoutState(st.timeout))
            return smcc
        return sm

    def build_state_machine(self):
        sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'failed'])
        for m in self.mission.plan:
            with sm:
                smach.StateMachine.add(self.get_state_name(m.name), self._get_state(m),
                                       self._make_transitions(m))
        return sm.execute()



def main():
    rospy.init_node('mission_planner', anonymous=False)
    #rospy.loginfo('bbauv mission planner 2.0')
    #robosub = plan()
    #robosub.master_timeout = 150
    #robosub.plan.append(task('wait', 0, False, ''))
    #robosub.plan.append(task('lane', 10, True, 'wait'))
    #m = MissionPlanner(robosub)
    #m.build_state_machine()
    sm = smach.Concurrence(outcomes=['succeeded', 'preempted', 'failed'],
                           child_termination_cb=lambda so: True,
                           default_outcome='preempted',
                           outcome_map={
                            'succeeded' : {'TIMEOUT1' : 'succeeded'},
                            'preempted' : {'TIMEOUT2' : 'succeeded',
                                           'TIMEOUT1' : 'preempted'}
                           })
    with sm:
        smach.Concurrence.add('TIMEOUT1', TimeoutState(10))
        smach.Concurrence.add('TIMEOUT2', TimeoutState(5))

    return sm.execute()

if __name__ == '__main__':
    main()
