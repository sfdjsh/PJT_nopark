## 풀이

---

<br>

```bash
rostopic pub /ctrl_cmd morai_msgs/CtrlCmd "{longlCmdType: 1, accel: 0.6, brake: 0.0, steering: 0.1, velocity: 0.0, acceleration: 0.0}"
```

<br>

## 시행착오(?)

---

<br>

환경설정까지 모두 마친 뒤에 rosbridge를 실행했을 때 이와 같은 에러가 발생하였다.<br><br>

<center>
<img
src="./README_pic/시행착오 1.png"
width="70%"
/>
</center>

<br>

이를 해결하기 위해 `vi ~/.bashrc`를 수행하고 마지막 부분에 조교님의 이 두 줄을 추가하였다.

<br>

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

<br>

그랬더니 오류도 사라졌고, 정상적으로 메세지 송수신까지 성공하였다! 성공 영상은 깃에 올려 두었고, 성공 메세지는 다음과 같다.<br><br>

<center>
<img
src="./README_pic/시행착오 2.png"
width="70%"
/>
</center>
