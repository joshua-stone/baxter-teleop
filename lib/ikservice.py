import rospy
 
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from rospy import Time

class IKService(object):
	def __init__(self, limb):
		self._limb_service = '/ExternalTools/{}/PositionKinematicsNode/IKService'.format(limb)

		self._iksvc = rospy.ServiceProxy(self._limb_service, SolvePositionIK)
		self._ikreq = SolvePositionIKRequest()
	#	self._limb_position = limb_position

	def solve_position(self, new_position):
		position, orientat = new_position['position'], new_position['orientation']
		header = Header(stamp=Time.now(), frame_id='base')

		new_pose = PoseStamped(
			header=header,
			pose=Pose(
				position=position,
				orientation=orientat
			)
		)
		
		self._ikreq.pose_stamp = [new_pose]
		
    		try:
			rospy.wait_for_service(self._limb_service, 5.0)
			resp = self._iksvc(self._ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
		
		if (resp.isValid[0]):
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
		else:
			limb_joints = {}
		
		return limb_joints
