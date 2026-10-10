# 기본 카메라 마운트 아래쪽 15° · 2026-10-10

Windows에서 기존/10°/15° 모델을 같은 시뮬레이션 위치로 비교한 뒤 사용자가 15°를 선택했습니다. 실제 Fusion `final_assembly_fix`의 `3D_LiDAR_Camera_mount_15:1`과 `D435f_Solid v1:1`에서 추출한 형상·좌표·물성을 본 저장소의 기본 파일에 반영했습니다.

- [robocup.urdf](../../../../SW/simulation/robot_description/robocup.urdf): nominal 모델.
- [robocup.calibrated.urdf](../../../../SW/simulation/robot_description/robocup.calibrated.urdf): 기존 보정에 같은 CAD 강체 변화를 적용한 모델.
- `../meshes/base_link.stl`, `../meshes/camera_link.stl`: 선택한 랙/카메라 메시. URDF는 기존 기본 경로를 그대로 사용합니다.

랙의 50개 부품, Mid360/G2, PiPER, 손목 카메라/TCP는 유지했습니다. 카메라 원점은 기존보다 x +4.283680 mm, y +1.250000 mm, z −9.310983 mm 이동하고 방향은 아래로 15° 바뀝니다. 랙 질량은 15.646368103 → 15.692130457 kg입니다. 카메라 joint뿐 아니라 랙에 직접 붙은 Gazebo RGB/depth sensor pose, 랙 관성 및 마운트 충돌 상자도 갱신했습니다.

`source_current.f3d`, `rack_camera_tilt15.f3d`, `current_audit.json`, `mesh_export.json`, `camera_frame_cm.json`, `export_fusion.py`는 실제 추출 자료입니다. 내보내기 코드의 Windows 경로는 당시 경로이므로 다른 PC에서는 조정해야 합니다. `application.json`은 적용 변환과 현재 URDF 해시, `validation.json`은 모델 검사 결과입니다. `previous_*`는 변경 전 랙/보정 기록입니다. 다른 과거 캘리브레이션 세션은 수정하지 않았습니다.

기존 Head hand-eye 결과에 CAD 강체 변화만 적용했으며, 새 15° 마운트에서 다시 측정한 보정은 아닙니다. `SW/simulation/robot_description/head_mount_delta.json`을 보정 통합 스크립트가 읽어 재생성 후에도 15°를 유지합니다. 기록된 Head 입력의 canonical JSON 해시와 nominal 카메라 좌표가 다르면 재생성을 중단해 중복 적용을 방지합니다. 새 마운트에서 보정을 다시 수행하면 해당 delta 기록도 함께 갱신/제거해야 합니다.

검증 PASS: nominal 94 links / 93 joints, calibrated 95 links / 94 joints, 연결된 트리, 모든 메시, 양의 관성, TF/sensor 상대 관계, 다른 링크·관절·메시 30개 보존. 실제 보정 통합 재생성에서도 모든 관절·센서 좌표가 선택 모델과 2e-10 이내로 일치했습니다. Windows 정적 시야 미리보기는 완료했고 ROS/Gazebo 물리 실행은 수행하지 않았습니다.
