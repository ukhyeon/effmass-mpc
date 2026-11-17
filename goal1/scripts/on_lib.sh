# on_lib.sh - shared argument sets for overnight runs (source it). Fixed settings: profile t1, shapes v2, u nearest, max_all, gain mode
# (d_base 0.065, mu_ref 0.53, eta_max 0.25), precomputed human, static human motion model, B1 log off, rollout cost aggregation off.
G=$HOME/effmass_mpc/goal1/b2a/g2; D=$HOME/effmass_mpc/goal1/data; RUN=$HOME/effmass_mpc/goal1/scripts/on_run.sh
COMMON="--scene g2 --shapes v2 --human-dir $G --human-viz none --u-def nearest --meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --b1-log 0 --cost-agg 0 --profile t1"
kgain() { case $1 in gk000) echo 0.0 ;; gain) echo 0.10 ;; gk005) echo 0.05 ;; gk015) echo 0.15 ;; *) echo BAD; return 2 ;; esac; }
