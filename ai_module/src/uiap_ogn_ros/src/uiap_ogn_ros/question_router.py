import re
from typing import Tuple

import rospy
from std_msgs.msg import String


class QuestionRouter:
    """
    Lightweight classifier for /challenge_question that extracts an object goal and a coarse query type.
    """

    def __init__(self, params: dict) -> None:
        self._question_topic = params.get("question_topic", "/challenge_question")
        self._object_goal_pub = rospy.Publisher("/uiap_ogn/object_goal", String, queue_size=1, latch=True)
        self._query_type_pub = rospy.Publisher("/uiap_ogn/query_type", String, queue_size=1, latch=True)
        rospy.Subscriber(self._question_topic, String, self._question_cb, queue_size=1)

    def _question_cb(self, msg: String) -> None:
        target, qtype = self._parse_question(msg.data)
        rospy.loginfo(f"[uiap_ogn_ros] Parsed question -> target='{target}' type='{qtype}'")
        self._object_goal_pub.publish(String(data=target))
        self._query_type_pub.publish(String(data=qtype))

    def _parse_question(self, text: str) -> Tuple[str, str]:
        clean = text.strip()
        lowered = clean.lower()
        if lowered.startswith("how many") or "how many" in lowered:
            target = clean.split("how many", 1)[-1].strip(" ?.")
            return target if target else clean, "count"
        if lowered.startswith("find") or lowered.startswith("locate") or "find the" in lowered:
            target = re.sub(r"^find( the)?", "", lowered, count=1).strip(" .?")
            return target if target else clean, "find"
        if lowered.startswith("go to") or lowered.startswith("navigate") or "go to the" in lowered:
            target = re.sub(r"^(go to|navigate to)", "", lowered, count=1).strip(" .?")
            return target if target else clean, "navigate"
        return clean, "find"
