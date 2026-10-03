# 센서 랙 + Piper

`sensor_rack_piper.urdf`: 원본 Piper 팔/그리퍼 mimic 관계 유지.
`sensor_rack_piper_gazebo.urdf`: 기존 Gazebo용 Piper 팔/그리퍼 모델 사용.

연결: `base_link → cad_Manipulator_mount_1 → piper_base_link`.
`rack_to_piper_base`는 fixed joint이며 베이스만 rigid 고정하고 팔 관절은 유지합니다.
Piper의 독립 world 링크를 제거하고 이름에 `piper_`를 붙여 충돌을 방지했습니다.
메시는 기존 폴더를 상대 경로로 참조합니다.

마운트 판 경계박스: 중심 Z=0.7945 m, 두께 0.009 m, 윗면 Z=0.799 m.
Piper 베이스 메시의 바닥 Z=0이 윗면에 닿도록 CAD 마운트 프레임에서 Z=0.0038 m 이동합니다.
베이스 원점은 랙 기준 (0.006751615091, 0, 0.799) m, 회전은 (0,0,0)입니다.
X/Y는 기존 CAD 마운트 원점을 사용한 배치입니다. 실제 체결 구멍 중심과 설치 방향은 확인이 필요합니다.

재생성: `python3 build.py`.
