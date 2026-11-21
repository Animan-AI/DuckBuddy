# DuckBuddy r8 系统镜像

适用于 Orange Pi Zero 3W，基于 Debian 13 Trixie。支持 HL-2915（3S）和 HD-1910（2S）。r8 已通过文件系统、默认配置和 Key 切换回归检查，整卡启动和两种机型的运动仍待实机验证。

## 这次更新

- 可在网页「大模型与声音」选择智谱 GLM、阿里千问、火山方舟、DeepSeek、OpenRouter 或自定义兼容接口。默认选智谱 GLM-4.6V-Flash。
- 可选微软 Edge TTS、智谱 GLM-TTS、硅基流动或自定义兼容接口。默认用 Edge 晓晓，支持人声流式播报和嘴部张合。可用网页或语音切换人声和鸭语。
- 模型和 TTS 的 Key 按供应商和接口地址各自保存。切回已配置的服务时自动恢复对应的 Key。镜像中没有预填 Key，后台会提示填写。
- 保留首次开机语音扫码配网、IP 播报、手柄配对和行走起步补偿。整机运动服务在标定后启用。

## 下载与烧录

1. 在 [GitHub Releases](https://github.com/Animan-AI/DuckBuddy/releases/tag/opi-image-r8) 下载 `.7z` 压缩包和 `.img.sha256`，放进同一个文件夹。
2. 用 7-Zip 解压 `.7z`，得到 `DuckBuddy-opizero3w-trixie-20261120-r8.img`。打开 PowerShell 校验 SHA-256，结果应和 `.img.sha256` 的首列一致。

   ```powershell
   Get-FileHash .\DuckBuddy-opizero3w-trixie-20261120-r8.img -Algorithm SHA256
   ```

3. 用 balenaEtcher 把 `.img` 烧录到至少 16 GB 的 TF 卡。已标定的机器换卡前，先备份原卡的机器人配置。
4. 按[新手教程](https://animan-ai.github.io/DuckBuddy/tutorial.html)连网和标定。联网后在浏览器打开 `http://鸭子的IP:8088/`，在「大模型与声音」填写自己的 API Key。HD-1910 版先执行 `duck-model hd1910`，并使用 2S 电池。

已有经校验的 r7 镜像时，可在 Linux 或 WSL 运行以下命令重建 r8。r7 的 `.sha256` 文件需与镜像同目录。

```bash
sudo bash deploy/opi-image/build-r8-from-r7.sh /path/to/r7.img /path/to/r8.img
```

Animan
