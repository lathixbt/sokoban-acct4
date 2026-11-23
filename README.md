# Sokoban Solver — Account 4

[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/lathixbt/sokoban-acct4/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/lathixbt/sokoban-acct4/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

Push-optimal Sokoban solver (Node 22, stdlib only).

Levels are a 1-wide hallway plus 1–4 rooms, and a box pushed into the
hallway can never come back, so every box stays in its own room. This solver
decomposes the level into rooms, solves each room independently with an exact
push-optimal A* (Manhattan-assignment heuristic, dead-square / wall-freeze /
2x2-freeze pruning, canonical player region), then tries every room visiting
order and keeps the one with the least walking. A generic whole-level A*
fallback covers levels where the hallway/room structure does not hold.

Run: `MAKEFUN_INPUT_DIR=<input> MAKEFUN_OUTPUT_DIR=<output> node main.ts`
