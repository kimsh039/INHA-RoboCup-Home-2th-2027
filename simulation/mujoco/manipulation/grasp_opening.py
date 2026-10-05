"""실제 개구로 큐브를 끼울 수 있는지 CPU에서 검사한다."""
from itertools import product
import numpy as np


def cube_opening_check(grasp, dimensions, opening, margin=0.001):
    # 물체 전체의 여덟 꼭짓점을 파지 좌표계로 옮긴다. Y가 손가락 개구 축이다.
    corners = np.asarray(list(product((-1, 1), repeat=3))) * np.asarray(dimensions) / 2
    local = (corners - np.asarray(grasp['translation'])) @ np.asarray(grasp['rotation_matrix'])
    lower, upper = float(local[:, 1].min()), float(local[:, 1].max())
    limit = opening / 2 - margin
    # 단순 물체 폭뿐 아니라 파지 중심의 편심도 검사한다. 접촉 성공 판정은 별도로 수행한다.
    return {'fits': bool(limit > 0 and lower >= -limit and upper <= limit),
            'jaw_axis_min_m': lower, 'jaw_axis_max_m': upper,
            'opening_m': float(opening), 'margin_per_side_m': float(margin)}
