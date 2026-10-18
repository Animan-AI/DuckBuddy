# persona — 鸭子是谁

框架（生命层、语音、视觉、大脑服务）和"这只鸭子是谁"是分开的。名字、性格、说话的风格、声音都放在一个 JSON 人设文件里，各个服务启动时读取它。

| 文件 | 内容 |
|---|---|
| `persona.py` | 加载器，字段说明也在这里 |
| `example.json` | 示例鸭子"豆豆"。开源仓库只带这一份 |
| `test_persona.py` | 测试 |

**用到人设的地方**：

| 服务 | 用到什么 |
|---|---|
| voiced | 它的名字，以及语音识别常听错成的字 |
| lifed | 名字、活泼程度、话多程度；离线小脑里报名字的台词 |
| duckweb | 网页上显示的名字 |
| duckbrain | 鸭语回应提示词里的"你是谁、什么性格、怎么说话"，以及语音合成的风格和参考音 |
| duckbrain/judge.py | 给 Jev 的"这只鸭子是什么"，以及各个问题里的名字 |

回应的格式（JSON 字段、动作清单）和"不知道就说不知道"这类规则属于框架，不在人设里。

## 怎么找到人设文件

按这个顺序找，用第一个存在的：
1. 环境变量 `$DUCK_PERSONA` 指定的文件；
2. `~/.config/duck/persona.json`；
3. 本目录的 `example.json`。

找到的文件会叠加在 `example.json` 之上，所以只需写和示例不同的字段。写错字段名会直接报错，不会悄悄退回示例。

文本字段可以写成字符串列表，加载时直接拼接、不加分隔，需要换行的地方自己写 `\n`。

## 部署

各服务从自己目录的**同级** `persona/` 目录导入 `persona.py`：
- 仓库里是 `tools/persona`；
- 机器人上是 `~/persona`；
- 大脑服务器上是大脑目录的同级目录。

**机器人（Orange Pi）**：

```bash
ssh <robot> 'mkdir -p ~/persona ~/.config/duck'
scp tools/persona/persona.py tools/persona/example.json <robot>:persona/
scp my-duck.json <robot>:.config/duck/persona.json
```

然后重启 lifed、voiced、duckweb。

**大脑服务器**：把 `persona.py` 和 `example.json` 放到 `duckbrain/` 的同级目录 `persona/`，人设放到运行大脑服务那个用户的 `~/.config/duck/persona.json`，然后重启 duckbrain 和 duckjev。

⚠ 新版 duckbrain 和 lifed 少了 `persona/` 目录会直接起不来。**先部署人设，再更新服务。**

lifed 启动时会在日志里打印 `persona: <名字> (<文件>)`，可以确认用的是哪个人设。
