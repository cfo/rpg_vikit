#!/usr/bin/python

import os
import rospy

class RosNode:
    def __init__(self, package, executable):
        self._package = package
        self._executable = executable
        self._param_string = ''
    
    def add_parameters(self, namespace, parameter_dictionary):
        for key in parameter_dictionary.keys():
            if type(parameter_dictionary[key]) is dict:
                self.add_parameters(namespace+key+'/', parameter_dictionary[key])
            else:
                self._param_string += ' _'+namespace+key+':='+str(parameter_dictionary[key])
        
    def clear_all_parameters(self):
        params = rospy.get_param_names()
        for p in params:
            rospy.delete_param(p)

    def run(self, parameter_dictionary, namespace=''):
        self.clear_all_parameters()
        self.add_parameters(namespace, parameter_dictionary)
        print('Starting ROS node with parameters: '+self._param_string)
        
        os.system('rosrun ' + self._package + ' ' + self._executable + ' ' + self._param_string)
        print('ROS node finished processing.')