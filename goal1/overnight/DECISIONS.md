# DECISIONS

Format: time / situation / alternatives / choice and reason / original criterion and its verdict.

## D0-1 23:24 regression baseline
- Situation: baseline runs on_base_gk000_s0 and on_base_gain_s0 (V2 front, single pass, 304 steps, B1 log off, cost aggregation off) vs the V2 records
  v1_front_0.38h1_gk000_s0 / v1_front_0.38h1_gain_s0 (B1 log on, aggregation on): joint commands bit-equal over 304 steps (max diff 0.0).
  Repeat of R1 r1_v050_gk000_s0 (60 s round trip): bit-equal over 3600 steps.
- Choice: the V2 records are tonight's regression reference. After any code change, on_base_gk000_s0 and on_base_gain_s0 are re-run and compared with them.
- Original criterion: bit-equal with V2 records. Verdict: pass.
