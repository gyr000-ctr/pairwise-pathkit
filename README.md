# pairwise-pathkit

## 题面（可直接粘贴）

矢量编辑器接入手写路径后出现精度事故：扁平化只按固定步数采样，容差形同虚设；弧长估计在缩放后偏差超过 2%；按长度切分返回的切点不在曲线上；撤销重放后结果漂移。pathkit/core.py 是核心，tests/test_core.py 15 个红。要修的是：flatten 必须自适应细分，折线与真实曲线的最大偏差不超过容差；arc_length 与稠密采样参考误差不超过 0.5%；split_at_length 切点落在曲线上且两段弧长之和等于原长；快照回放一致；点数上限抛 ValueError 且状态不变。

## 运行方式

```bash
python -m pytest tests -q
```

初始结果应为 `15 failed`（全部因 `NotImplementedError` 失败，无 collection error）。

## 字段

- 所属领域：游戏、图形与多媒体
- 子方向：图形渲染
- 任务类型：Bugfix
- 语言/框架：Python 3, pytest
- 环境可复现等级：无外部依赖
