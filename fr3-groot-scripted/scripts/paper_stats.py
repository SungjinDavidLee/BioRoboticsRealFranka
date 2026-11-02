#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""paper_stats.py — 논문용 데이터 통계 (로봇 PC, 읽기만 — 로봇 안 움직임, 아무것도 안 고침)

  python3 paper_stats.py                         # ~/gello_data/medbio_redslot_v1
  python3 paper_stats.py --session ~/gello_data/<세션> > stats.md

출력(Markdown): 수집 개요 · 폐기 사유 · 시연 계획 무작위 값 분포 · 잡기/놓기 실제 정확도 · 위치 분포 · 보정 · 세션 설정 · 모델 실행 결과
"""
import argparse
import collections
import datetime
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path.home() / 'gello_tools'))
from gello_to_lerobot import fk   # 스테이션과 같은 순기구학 (fr3_link0 → fr3_hand_tcp)


def tcp(q):
    q = np.asarray(q, float)
    return fk(q)[:3, 3] if np.all(np.isfinite(q)) else np.full(3, np.nan)


def yaw(q):
    R = fk(np.asarray(q, float))[:3, :3]
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


def st(v, unit='', k=1.0, nd=1):
    v = np.asarray([x for x in v if x is not None and np.isfinite(x)], float) * k
    if len(v) == 0:
        return '–'
    return (f"{np.mean(v):.{nd}f} ± {np.std(v):.{nd}f}{unit} (중간 {np.median(v):.{nd}f}, "
            f"{np.min(v):.{nd}f}–{np.max(v):.{nd}f}, n={len(v)})")


def pct(v, p, k=1.0):
    v = np.asarray([x for x in v if x is not None and np.isfinite(x)], float) * k
    return float(np.percentile(v, p)) if len(v) else float('nan')


def load_json(p):
    try:
        return json.loads(Path(p).read_text())
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--session', default='~/gello_data/medbio_redslot_v1')
    a = ap.parse_args()
    S = Path(a.session).expanduser()
    eps = sorted(p for p in S.glob('episode_*') if p.is_dir())
    trash = sorted(p for p in (S / '_discarded').glob('episode_*') if p.is_dir()) if (S / '_discarded').exists() else []
    print(f'# 논문용 통계 — {S.name}  ({datetime.datetime.now():%Y-%m-%d %H:%M})\n')

    # ── 1. 수집 개요 ──
    metas = [(e, load_json(e / 'meta.json') or {}) for e in eps]
    tmetas = [(e, load_json(e / 'meta.json') or {}) for e in trash]
    ver = collections.Counter(((m.get('quality') or {}).get('verdict') or '?') for _, m in metas)
    src = collections.Counter((m.get('source') or '?') for _, m in metas)
    dates = sorted(str(m.get('recorded_at') or m.get('start', {}).get('t') or '')[:10] for _, m in metas)
    dates = [d for d in dates if d]
    durs = [m.get('duration_s') for _, m in metas]
    nfr = [m.get('n_frames') for _, m in metas]
    print('## 1. 수집 개요\n')
    print(f'- 저장된 시행: **{len(eps)}** (판정 {dict(ver)}) · 출처 {dict(src)}')
    print(f'- 휴지통(자동·수동 폐기): **{len(tmetas)}** → 저장 비율 {len(eps) / max(1, len(eps) + len(tmetas)) * 100:.1f}%')
    if dates:
        print(f"- 수집 날짜: {dates[0]} ~ {dates[-1]} ({len(set(dates))}일) · 날짜별 {dict(collections.Counter(dates))}")
    print(f'- 시행 길이(기록): {st(durs, " s")}')
    print(f'- 원본 프레임 수: 합계 {int(np.nansum([x or 0 for x in nfr]))} · 시행당 {st(nfr, "", nd=0)}')
    tasks = collections.Counter((m.get('task') or '') for _, m in metas)
    print(f'- 지시문: {dict(tasks)}\n')

    # 폐기·주의 사유
    why_bad, why_warn = collections.Counter(), collections.Counter()
    for _, m in tmetas:
        for r in (m.get('quality') or {}).get('bad', []) or ['(사유 없음/수동)']:
            why_bad[str(r).split(' (')[0]] += 1
    for _, m in metas:
        for r in (m.get('quality') or {}).get('warn', []):
            why_warn[str(r).split(' (')[0]] += 1
    print('### 폐기 사유 (휴지통)\n')
    for r, c in why_bad.most_common(15):
        print(f'- {r}: {c}')
    print('\n### 저장분 중 ‘주의’ 사유\n')
    for r, c in why_warn.most_common(15):
        print(f'- {r}: {c}')
    print()

    # ── 2. 시연 계획 (start.auto) ──
    A = [((m.get('start') or {}).get('auto') or {}) for _, m in metas]
    print('## 2. 자동 시연 계획 값 (meta.json start.auto)\n')
    print(f"- 놓는 곳: {dict(collections.Counter(x.get('place') for x in A))} · 꽂을 칸 정한 방식: "
          f"{dict(collections.Counter((x.get('place_info') or {}).get('target_src') for x in A))}")
    print(f"- 칸 위 맞춤(align) 켠 시행: {sum(1 for x in A if x.get('align'))} / {len(A)}")
    print(f"- 잡는 높이 조정 grasp_dz(mm): {dict(collections.Counter(x.get('grasp_dz_mm') for x in A))}")
    print(f"- 놓는 높이 release_h(cm): {dict(collections.Counter(x.get('release_h_cm') for x in A))}")
    print(f"- 바로 다 엶(open_full): {sum(1 for x in A if x.get('open_full'))} / {len(A)}")
    print(f"- 실행 속도 배율×무작위: {st([x.get('speed') for x in A], '', nd=3)}")
    print(f"- 접근 높이 a(m): {st([x.get('approach_dz') for x in A], '', nd=3)}")
    print(f"- 들기 높이 l(m): {st([x.get('lift_m') for x in A], '', nd=3)}")
    print(f"- 호 높이 h(m): {st([x.get('arc_h') for x in A], '', nd=3)}")
    print(f"- 경유점 치우침 |via|(m): {st([math.hypot(*x['via'][:2]) if x.get('via') else None for x in A], '', nd=3)}")
    print(f"- 계획 동작 시간 move_s: {st([x.get('move_s') for x in A], ' s')}")
    print(f"- 7번 관절 움직임 범위: {st([x.get('j7_range_deg') for x in A], '°')}")
    print(f"- 꽂을 때 추가 회전 yaw_add: {st([x.get('yaw_add_deg') for x in A], '°')} · 0이 아닌 시행 "
          f"{sum(1 for x in A if abs(x.get('yaw_add_deg') or 0) > 0.5)}")
    pis = [x.get('place_info') or {} for x in A]
    print(f"- 원근 보정 D(m): {st([p.get('D_m') for p in pis], '', nd=3)} · 보정 이동량: {st([p.get('shift_mm') for p in pis], ' mm')}")
    hc = [p.get('hole_corr_mm') for p in pis if p.get('hole_corr_mm')]
    print(f"- 꽂은 기록 보정(hole_learn) 적용 시행: {len(hc)}" + (f" · 보정량 {st([math.hypot(*h) for h in hc], ' mm')}" if hc else ''))
    rk = [x.get('rack_deg') for x in A if x.get('rack_deg') is not None]
    if rk:
        h = collections.Counter(int(float(r) % 180 // 30) * 30 for r in rk)
        print(f"- 원래 랙 방향(로봇 기준, 30° 칸): {dict(sorted(h.items()))}")
    print(f"- 장면 뚜껑 수: {dict(collections.Counter(x.get('caps') for x in A))}")
    cal = [x.get('calib') or {} for x in A]
    if cal and cal[-1]:
        print(f"- 보정(시행 기록): n {dict(collections.Counter(c.get('n') for c in cal))}, rms {st([c.get('rms_mm') for c in cal], ' mm')}, "
              f"LOO 최대 {st([c.get('loo_mm') for c in cal], ' mm')}")
    G = np.array([x['grasp_xyz'] for x in A if x.get('grasp_xyz')], float)
    Dp = np.array([x['drop_xyz'] for x in A if x.get('drop_xyz')], float)
    if len(G):
        print(f"- 계획 잡을 위치 범위(mm): x {G[:, 0].min() * 1e3:.0f}–{G[:, 0].max() * 1e3:.0f}, "
              f"y {G[:, 1].min() * 1e3:.0f}–{G[:, 1].max() * 1e3:.0f}, z {G[:, 2].min() * 1e3:.0f}–{G[:, 2].max() * 1e3:.0f}")
    if len(Dp):
        print(f"- 계획 놓을 위치 범위(mm): x {Dp[:, 0].min() * 1e3:.0f}–{Dp[:, 0].max() * 1e3:.0f}, "
              f"y {Dp[:, 1].min() * 1e3:.0f}–{Dp[:, 1].max() * 1e3:.0f}, z {Dp[:, 2].min() * 1e3:.0f}–{Dp[:, 2].max() * 1e3:.0f}")
    if len(G) and len(Dp) == len(G):
        cfg = []
        for g, d in zip(G, Dp):
            if not any(np.hypot(*(g[:2] - c[0][:2])) < 0.015 and np.hypot(*(d[:2] - c[1][:2])) < 0.015 for c in cfg):
                cfg.append((g, d))
        print(f"- 서로 다른 (잡을 칸, 꽂을 칸) 위치 조합(1.5 cm 안은 같게 봄): **{len(cfg)}**")
    print()

    # ── 3. 실제 실행 정확도 (data.npz) ──
    eg, er, wg, tl, tw, zg, ya, cmdg, cmdr, vcl = [], [], [], [], [], [], [], [], [], []
    for (e, m), x in zip(metas, A):
        try:
            d = np.load(e / 'data.npz')
            t, q, c, lg, w = (np.asarray(d[k], float) for k in ('t', 'q', 'cmd_q', 'leader_grip', 'grip_w'))
        except Exception:
            continue
        j = next((i for i in range(1, len(lg)) if lg[i - 1] > 0.5 and lg[i] < 0.5), None)
        if j is None:
            continue
        o = next((i for i in range(j + 1, len(lg)) if lg[i - 1] < 0.5 and lg[i] > 0.5), None)
        pj = tcp(q[j])
        if x.get('grasp_xyz'):
            g = np.array(x['grasp_xyz'])
            eg.append(np.hypot(*(pj[:2] - g[:2])) * 1e3)
            zg.append((pj[2] - g[2]) * 1e3)
        cmdg.append(np.linalg.norm(pj - tcp(c[j])) * 1e3)
        ya.append(yaw(q[j]))
        k2 = min(len(w) - 1, int(np.searchsorted(t, t[j] + 3.0)), (o - 1) if o else len(w) - 1)
        wg.append(w[k2] * 1e3)
        P = [tcp(qq) for qq in q[j:j + 200]]
        lift = next((k for k, p in enumerate(P) if np.isfinite(p[2]) and p[2] > pj[2] + 0.005), None)
        if lift is not None:
            tl.append(t[j + lift] - t[j])
        if o is not None:
            po = tcp(q[o])
            if x.get('drop_xyz'):
                dd = np.array(x['drop_xyz'])
                er.append(np.hypot(*(po[:2] - dd[:2])) * 1e3)
            cmdr.append(np.linalg.norm(po - tcp(c[o])) * 1e3)
    print('## 3. 자동 시연의 실제 실행 (관절 실측 FK 기준)\n')
    print(f'- 쥘 때 손끝 − 계획 (수평): {st(eg, " mm")} · 95% {pct(eg, 95):.1f} mm')
    print(f'- 쥘 때 높이 실제 − 계획: {st(zg, " mm")}')
    print(f'- 놓을 때 손끝 − 계획 (수평): {st(er, " mm")} · 95% {pct(er, 95):.1f} mm')
    print(f'- 쥘 때 |실제 − 명령|: {st(cmdg, " mm")} · 놓을 때: {st(cmdr, " mm")}  (임피던스 제어의 정상 상태 오차)')
    print(f'- 쥔 폭(3 s 뒤): {st(wg, " mm")}')
    print(f'- 쥔 순간 그리퍼 각도(바닥 기준 yaw): {st(ya, "°")}')
    print(f'- 닫기 명령 → 5 mm 들기 시작: {st(tl, " s")}\n')

    # ── 4. 보정 파일 ──
    print('## 4. 화면 → 로봇 보정 (auto_calib.json)\n')
    ac = load_json(S / 'auto_calib.json')
    if ac and ac.get('points'):
        pts = ac['points']
        try:
            import gello_scene as gsc
            px = np.array([p['px'] for p in pts], float)
            xy = np.array([p['xy'] for p in pts], float)
            Hm, rms, loo, kind = gsc.fit_homography(px, xy)
            print(f"- 점 {len(pts)}개 · 선택된 모델 **{kind}** · 맞춤 오차 rms {rms:.2f} mm · 하나 빼고 맞춘 최대 오차 {loo if loo is None else round(loo, 2)} mm")
        except Exception as ex:
            print(f'- 점 {len(pts)}개 (모델 다시 맞추기 실패: {ex})')
        z = [p['z'] for p in pts]
        print(f"- 잡는 높이(점들의 중앙값): {np.median(z) * 1e3:.1f} mm (범위 {min(z) * 1e3:.1f}–{max(z) * 1e3:.1f})")
        print(f"- 보정한 날: {sorted(set(str(p.get('t', ''))[:10] for p in pts))}")
    else:
        print('- auto_calib.json 없음')
    print()

    # ── 5. 세션·스테이션 설정 ──
    print('## 5. 설정\n')
    at = load_json(S / 'auto_task.json')
    if at:
        print(f"- auto_task.json: {json.dumps(at, ensure_ascii=False)}")
    stj = load_json(Path.home() / 'gello_tools' / 'station.json')
    if stj:
        print(f"- station.json auto: {json.dumps(stj.get('auto'), ensure_ascii=False)}")
        print(f"- 그리퍼: {json.dumps({k: (stj.get('gripper') or {}).get(k) for k in ('force', 'speed', 'epsilon_m')}, ensure_ascii=False)}")
        print(f"- 모델 실행: {json.dumps(stj.get('rollout'), ensure_ascii=False)}")
    hj = load_json(S / 'home.json')
    if hj:
        p = tcp(hj['q'])
        print(f"- 홈 TCP (mm): ({p[0] * 1e3:.1f}, {p[1] * 1e3:.1f}, {p[2] * 1e3:.1f}) · 저장 {hj.get('saved_at')}")
    ex = [((m.get('start') or {}).get('exposure') or {}) for _, m in metas]
    if any(ex):
        for cam in ('third', 'wrist'):
            v = [e.get(cam) for e in ex if isinstance(e.get(cam), (int, float))]
            if v:
                print(f'- {cam} 노출(µs): {min(v)}–{max(v)} (중간 {np.median(v):.0f})')
    print()

    # ── 6. 모델 실행 ──
    R = S / '_rollouts'
    runs = sorted(p for p in R.glob('rollout_*') if (p / 'meta.json').exists()) if R.exists() else []
    print(f'## 6. 모델 실행 기록 ({len(runs)}번)\n')
    if runs:
        print('| 시각 | 체크포인트 | 결과 | 자동 판정 | 끝난 이유 | 시간(s) | 묶음 | 칸/묶음 | 왕복 중간(ms) | 추론 중간(ms) |')
        print('|---|---|---|---|---|---|---|---|---|---|')
        for r in runs:
            m = load_json(r / 'meta.json') or {}
            inf = []
            try:
                for ln in (r / 'chunks.jsonl').read_text().splitlines():
                    v = json.loads(ln).get('infer_ms')
                    if v is not None:
                        inf.append(v)
            except Exception:
                pass
            ck = str(m.get('ckpt') or (m.get('server') or {}).get('ckpt') or '?').split('checkpoints/')[-1].split('/')[0]
            print(f"| {str(m.get('started_at', ''))[5:16]} | {ck} | {m.get('result')} | {m.get('result_auto', '–')} | {m.get('reason')} | "
                  f"{m.get('duration_s')} | {m.get('chunks')} | {m.get('exec_steps') or (m.get('cfg') or {}).get('exec_steps')} | "
                  f"{m.get('total_ms_median')} | {np.median(inf) if inf else '–'} |")
        ms = [load_json(r / 'meta.json') or {} for r in runs]
        chk = [m for m in ms if m.get('labeled_by') == 'human' and m.get('result_auto') and m.get('result') != 'void']
        if chk:
            agree = sum(1 for m in chk if m['result'] == m['result_auto'])
            print(f"\n- 자동 판정과 사람 판정 일치: {agree}/{len(chk)} ({100 * agree / len(chk):.0f}%) — 무효 제외, 사람이 확인한 판만")
            for m in chk:
                if m['result'] != m['result_auto']:
                    print(f"  - {str(m.get('started_at', ''))[5:16]}: 자동 {m['result_auto']} → 사람 {m['result']} ({m.get('note', '')})")
        nv = sum(1 for m in ms if m.get('result') == 'void')
        if nv:
            print(f'- 무효 {nv}번 (통계 제외): ' + '; '.join(f"{str(m.get('started_at', ''))[5:16]} {m.get('note', '')}" for m in ms if m.get('result') == 'void'))


if __name__ == '__main__':
    main()
