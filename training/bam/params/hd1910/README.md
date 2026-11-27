# HD-1910 BAM M6 参数

这份 [1910_m6.json](1910_m6.json) 由 Luwu Dynamics（陆吾）公开，保留原始字节。适用于 [Rhoban BAM](https://github.com/Rhoban/bam) 的 M6 执行器仿真。

- [固定来源 xgoduck_rl @ 326d77a](https://github.com/LuwuDynamics/xgoduck_rl/blob/326d77a1122870bdefa2c36403937502c958e69c/src/mjlab_microduck/robot/xgoduck/params/1910_m6.json)
- SHA-256 `ef2d51adfb1cc0831b9b02ca19aa9176fceecdf725148d084378d7d9a64afceb`
- [Apache-2.0 许可](LICENSE)

DuckBuddy 用这组数值续训 r14 的 8498 行走策略。部署时 `robotd` 加载 ONNX，并向舵机下发[小黑实测的 PD](../../../../deploy/opi-image/r14/hd1910-pd.toml)。KissQY 的 HD-1910 适配是训练和部署的参考。
