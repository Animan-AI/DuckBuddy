# -*- coding: utf-8 -*-
"""Run a BAM module (fit, plot, process…) with the HL-2915 actuator registered.

    uv run python fit_hl2915.py bam.process --raw raw/m105 --logdir processed --dt 0.005
    uv run python fit_hl2915.py bam.fit --logdir processed --actuator hl2915 --model m6 \
        --method cmaes --trials 2000 --workers 16 --output params/hl2915/m6.json
    uv run python fit_hl2915.py bam.plot --logdir processed --sim --params params/hl2915/m6.json
"""
import os
import runpy
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hl2915  # noqa: E402,F401  registers "hl2915"

module = sys.argv[1]
sys.argv = [module] + sys.argv[2:]
runpy.run_module(module, run_name="__main__", alter_sys=True)
