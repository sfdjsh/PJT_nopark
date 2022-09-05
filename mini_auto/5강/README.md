## 풀이

---

<br>

```py
while not rospy.is_shutdown():
    if self.is_collision:       # 충돌했다면?
        self.send_gear_cmd(Gear.R.value)
        for _ in range(20):     # 적당한 시간 동안
            # 옆으로 조향하여 30의 속도로 후진한다.
            self.send_ctrl_cmd(0.2, 30.0)
            self.rate.sleep()
        self.send_gear_cmd(Gear.D.value)    # 그리고 다시 전진기어
    else:       # 평상시에는?
        # 따로 조향하지 않고 30의 속도로 전진한다.
        self.send_ctrl_cmd(0.0, 30.0)
        self.rate.sleep()
```

<br>

## 느낀 점

---

<br>

> 저번에는 싸피레이스를 C++로 했는데 이번에는 그것을 python으로 하는 느낌이었다.  
> 굉장히 신선했고 그 기억이 새록새록 떠오른다.  
> 동작 영상은 따로 업로드하였다.
