"""走査の計画（撮るヨー）と、実際に撮った画像から見た方向の範囲をまとめる。

角度はすべて rad。`normalize` だけは外からも使う（`panels.py`）。
ROS 2 を import しない純関数だけを置く。
"""
import math

TAU = 2.0 * math.pi


def normalize(a):
    """角度を −π 以上 π 未満に直す。"""
    return (a + math.pi) % TAU - math.pi


def hfov_from_fx(fx, width_px=1280):
    """水平の画角。`fx` は画素単位の焦点距離。"""
    return 2.0 * math.atan(width_px / 2.0 / fx)


def plan_yaws(azimuth_from_rad, azimuth_span_rad, hfov_rad, overlap_ratio):
    """依頼の範囲を、画角が少し重なる刻みで撮るヨーの一覧（開始から反時計回り）。"""
    span = float(azimuth_span_rad)
    hfov = float(hfov_rad)
    if span <= 0.0 or hfov <= 0.0:
        return []

    step = hfov * (1.0 - float(overlap_ratio))

    if span >= TAU - 1e-12:
        # 全周: 枚数を決めて等間隔にする
        n = max(1, int(math.ceil(TAU / step)))
        return [normalize(azimuth_from_rad + i * TAU / n) for i in range(n)]

    if span <= hfov:
        # 1 枚で足りる: 範囲の中央を向く
        return [normalize(azimuth_from_rad + span / 2.0)]

    # 最初の画像の左端を範囲の開始、最後の画像の右端を範囲の終わりに合わせる
    n = max(2, int(math.ceil((span - hfov) / step)) + 1)
    first = azimuth_from_rad + hfov / 2.0
    last = azimuth_from_rad + span - hfov / 2.0
    return [normalize(first + (last - first) * i / (n - 1)) for i in range(n)]


def _merge(intervals, eps=1e-9):
    """重なり・接している区間をまとめる。"""
    out = []
    for lo, hi in sorted(intervals):
        if hi - lo <= eps:
            continue
        if out and lo <= out[-1][1] + eps:
            out[-1][1] = max(out[-1][1], hi)
        else:
            out.append([lo, hi])
    return out


def seen_ranges(yaws_done, hfov_rad, azimuth_from_rad, azimuth_span_rad):
    """撮ったヨーごとの画角を依頼の範囲で切り、重なりをまとめた (開始, 幅) の一覧。

    全周を全部見たら `([開始], [2π])` を返す。
    """
    span = min(max(float(azimuth_span_rad), 0.0), TAU)
    hfov = float(hfov_rad)
    start = float(azimuth_from_rad)

    # 依頼の開始を 0 にした相対角で扱う（0〜2π の線として切る）
    pieces = []
    for y in yaws_done:
        lo = (float(y) - hfov / 2.0 - start) % TAU
        hi = lo + hfov
        if hi <= TAU:
            pieces.append((lo, hi))
        else:
            pieces.append((lo, TAU))
            pieces.append((0.0, hi - TAU))

    clipped = []
    for lo, hi in pieces:
        lo2, hi2 = max(lo, 0.0), min(hi, span)
        if hi2 - lo2 > 1e-9:
            clipped.append((lo2, hi2))

    merged = _merge(clipped)

    if span > 0.0 and sum(hi - lo for lo, hi in merged) >= span - 1e-6:
        # 依頼の範囲を全部見た
        return [normalize(start)], [span]

    if (span >= TAU - 1e-12 and len(merged) > 1
            and merged[0][0] <= 1e-9 and merged[-1][1] >= TAU - 1e-9):
        # 全周のときに 0/2π をまたぐ範囲を 1 つにまとめる
        first = merged.pop(0)
        last = merged.pop()
        merged.append([last[0], first[1] + TAU])

    froms = [normalize(start + lo) for lo, _ in merged]
    spans = [hi - lo for lo, hi in merged]
    return froms, spans
