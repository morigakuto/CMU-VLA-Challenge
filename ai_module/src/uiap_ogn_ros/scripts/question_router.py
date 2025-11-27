#!/usr/bin/env python3

import rospy

from uiap_ogn_ros.question_router import QuestionRouter


def main() -> None:
    rospy.init_node("uiap_question_router")
    params = rospy.get_param("~", {})
    QuestionRouter(params)
    rospy.spin()


if __name__ == "__main__":
    main()
