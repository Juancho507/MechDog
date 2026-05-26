# goal.ps1 — Envía un goal de navegación al robot MechDog
# Uso: .\goal.ps1 [x] [y]
# Ej:  .\goal.ps1 2.0 2.0   (default)
param([double]$x=15.0, [double]$y=2.0)

Set-Content -Path "$env:TEMP\goal.py" -Value @"
import rospy
from geometry_msgs.msg import PoseStamped
rospy.init_node('gp', anonymous=True)
pub = rospy.Publisher('/mechdog/goal', PoseStamped, queue_size=1, latch=True)
rospy.sleep(0.5)
m = PoseStamped()
m.header.frame_id = 'map'
m.header.stamp = rospy.Time.now()
m.pose.position.x = $x
m.pose.position.y = $y
m.pose.orientation.w = 1.0
pub.publish(m)
print("Goal ($x, $y) enviado")
"@

docker compose cp "$env:TEMP\goal.py" roscore:/tmp/goal.py
docker compose exec roscore bash -c "source /app/catkin_ws/devel/setup.bash && python3 /tmp/goal.py"
Remove-Item "$env:TEMP\goal.py"
