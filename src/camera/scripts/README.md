# 사전 작업

```bash
pip2 install boto3 mysql-connector-python==8.0.23 awscli
aws configure
# ACCESS_KEY_ID = 'AKIA2LPAPXE5XYE327MZ'
# ACCESS_SECRET_KEY = '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'
# region name = 'ap-northeast-2'
# output format 부분은 그냥 엔터
```

<br>

# 카메라 세팅

> Topic, FrameID를 모두 수정한 다음에 ROS Network 연결할 것!  
카메라 세팅 값은 노션의 Private Documents를 참고  

<br>

# DB data 확인 방법
> Private Documents에서 MySQL 아이디 및 비번 참고해서 tb_car_illegal data 참고  
img_data column에 있는 url이 차 사진 data이다.

<br>

# file 실행방법
> 따로 launch file은 없고 rosrun camera [실행할 .py file]을 하면 된다.