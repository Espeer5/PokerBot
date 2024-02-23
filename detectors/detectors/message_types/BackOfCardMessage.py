from detectors.message_types.CardPose import CardPose
import json

class BackOfCardMessage():

    def __init__(self, poses):
        self.poses = poses

    def to_string(self):
        lst = []
        for pose in self.poses:
            lst.append(pose.to_string())
        
        return json.dumps(lst)
    
    @staticmethod
    def from_string(str):
        print("BOCM from_string str=", str)
        str_poses = json.loads(str)

        poses = []
        for str_pose in str_poses:
            poses.append(CardPose.from_string(str_pose))

        return BackOfCardMessage(poses)