# Gazebo 실행

- `ros2 launch shift_ostrich gazebo.launch.py`: 지면의 READY 자세로 모델·제어기 실행.
- `ros2 launch shift_ostrich transform.launch.py`: Gazebo 실행 후 저장된 변신을 한 번 실행.
- `display.launch.py`: RViz 형상 확인.

모델이 켜진 뒤 동작은 `ros2 run shift_ostrich transform.py`, `ros2 run shift_ostrich revert.py`를 사용합니다.
전체 설명은 패키지 루트의 README.md를 참고하세요.
