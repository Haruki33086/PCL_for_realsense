# PCL_for_realsense
realsensed435iからの点群を受取り、KD-Treeを用いて最近傍探索を行うプログラム。ちなみにROS2用です。ちなみにロボットの障害物検知用です。Jetson nanoで動かしてみたけど重いので、Jetson Xavierでやってみたけどやっぱ重い。結局Voxel gridを使うことで処理する点群の数を減らしました。
~~~
sudo apt install ros-rosのバージョン-perception-pcl
~~~
