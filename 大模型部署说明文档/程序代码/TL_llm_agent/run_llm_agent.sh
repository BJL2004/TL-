#!/usr/bin/env bash
# ==================================================
# TL 种子抓放 · 大模型(MQTT)控制代理 启动脚本
# 用法:
#   bash run_llm_agent.sh                    # dry-run（只解析，不发送）
#   bash run_llm_agent.sh --execute --allow-real   # 真机：真实主题必须显式允许
#   bash run_llm_agent.sh --execute \
#       --topic-down node/arm/99/down --topic-up node/arm/99/up   # 演示/测试
#   bash run_llm_agent.sh --once "放到第三排第十七盆"
#   bash run_llm_agent.sh --selftest         # 离线自检
# 前提:
#   1) 板上已启动 TL 程序（driver + run_seed_mqtt.sh）在等 MQTT 指令
#   2) Ollama 已运行: ollama serve / ollama pull phi3:mini
# ==================================================
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
exec python3 llm_tl_agent.py "$@"
