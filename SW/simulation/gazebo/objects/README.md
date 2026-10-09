# 테이블 위 물체

`make_sim.py --world room`이 방 가운데 테이블 위에 아래 물체를 올립니다. 위치·질량·충돌 형상은 [make_sim.py](../make_sim.py)의 `OBJECTS`에 있습니다.

과제는 **사람이 요청한 물체를 집어 테이블의 비어 있는 다른 자리에 놓는 것**입니다.
물체 6개는 긴 변에서 약 0.2 m 안쪽, 도킹한 팔이 닿는 곳에 있고, 남쪽 변 가운데는 비워 두었습니다. 북쪽 긴 변이 책상 앞쪽(배선 홈이 있는 쪽)입니다.

| 물체 | 위치 (x, y) m | 놓인 변 | 모델 출처 |
|---|---|---|---|
| mug (컵) | (1.50, -0.20) | 남쪽(y -0.4) | YCB `025_mug` |
| banana (바나나) | (2.50, -0.22) | 남쪽 | YCB `011_banana` |
| fanta_can (환타 캔) | (1.45, 0.20) | 북쪽(y 0.4) | 직접 생성한 캔(지름 66 mm, 높이 122 mm), 350 ml 가득 참 |
| peach (복숭아) | (1.85, 0.22) | 북쪽 | YCB `015_peach` |
| apple (사과) | (2.15, 0.20) | 북쪽 | YCB `013_apple` |
| green_apple (청사과) | (2.50, 0.22) | 북쪽 | YCB `013_apple` 형상에 텍스처 색만 초록으로 변경 |

- YCB 모델은 `google_16k` 스캔 메시이며 텍스처만 1024 px JPG로 줄였습니다. 실제 크기(m)이고 바닥이 z=0입니다.
- 질량은 실물 기준입니다. YCB 과일은 가벼운 플라스틱 모형이라 실제 과일 무게(사과 180 g, 복숭아 130 g, 바나나 120 g)로 바꿨고, 캔은 음료 350 ml(약 364 g)와 캔(약 13 g)을 합한 377 g입니다.
- 충돌은 메시 대신 박스·구·원기둥으로 단순화했습니다. 시각 메시는 원본 형상 그대로입니다.
- Gazebo는 메시·텍스처를 **파일 이름으로 캐시**합니다. 물체마다 `<이름>.obj`, `<이름>.mtl`, `<이름>.jpg`처럼 고유한 이름을 써야 다른 물체의 텍스처가 섞이지 않습니다.

## 라이선스

YCB Object and Model Set — B. Calli et al., "The YCB Object and Model Set", ICAR 2015.
[ycbbenchmarks.com](https://www.ycbbenchmarks.com/), Creative Commons Attribution 4.0 International.
