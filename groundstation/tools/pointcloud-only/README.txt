单向点云桥职责

cloud_sender.py：机载运行，仅订阅PointCloud2，保留世界坐标，使用原地面站PCD1/LZ4协议发送到固定上位机。
manage.py：上位机管理专属SSH会话；start/stop/status，不上传机载源码、不安装服务、不杀其他ROS节点。
capture_cloud.py：指定源IP的真实分片接收检查；验证时使用独立端口19000，避免抢占地面站9000。
check_qt_decode.cpp：以地面站原始Qt解码器验证真实抓取的头和LZ4数据。

上位机独立管理：python3 manage.py start|stop|status
默认机载orangepi@192.168.1.235、上位机自动路由源地址、UDP9000；可用--host/--pc-ip/--port指定。
一键连接窗口使用工作目录根部 connect-live-pointcloud.sh，停止使用 stop-live-pointcloud.sh。
桥接临时停流不会停止或重置FAST-LIO。确认仅显示点云，不代表已连接任务执行。

可选位姿：manage.py start --with-pose，默认FAST-LIO odometry、POS1/9001。与点云同一原点，无ROS发布。停止同一SSH会话会同时停止两种遥测。body+X需与物理机头安装关系核对。
