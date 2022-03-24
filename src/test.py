#!/usr/bin/env python

import rospy

def main():
    rospy.init_node("test")
    rospy.loginfo("Hello, World!")
    rospy.spin()

if __name__ == "__main__":
    main()
