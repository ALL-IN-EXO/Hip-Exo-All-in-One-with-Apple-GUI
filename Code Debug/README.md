# Code Debug Notes

本目录本次重点问题说明见：

- `README_RL_LATENCY_AND_SYNC_SPIKE.md`

该文档对应 `Docs/CHANGELOG.md` 中 `## [Unreleased] -> ### Fixed` 的三条修复：

1. RPi backlog 导致的秒级陈旧（GUI 看起来“慢很多”）  
2. Sync 路径偶发爆帧（±300° / ±3000 dps）  
3. GUI POWER OFF 按钮在 Max Torque 被手动编辑后需要点两次才能关机
