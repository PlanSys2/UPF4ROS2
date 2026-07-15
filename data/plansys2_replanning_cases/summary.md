## Planning-Time Metrics
| Metric | Native n | Native mean | Native SD | UPF4ROS2 n | UPF4ROS2 mean | UPF4ROS2 SD | Mean diff. (UPF4ROS2-native), s | Diff. 95% CI low | Diff. 95% CI high | Welch t | Welch p | Holm p | Mann-Whitney U | Mann-Whitney p | Hedges g |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Initial planning latency (s) | 20 | 0.1946 | 0.02315 | 20 | 0.6675 | 0.06754 | 0.4729 | 0.4399 | 0.5058 | 29.62 | 4.641e-20 | 9.282e-20 | 400 | 6.796e-08 | 9.18 |
| First replanning latency (s) | 20 | 0.1731 | 0.01118 | 20 | 0.08406 | 0.005822 | -0.08902 | -0.09479 | -0.08325 | -31.59 | 8.694e-24 | 2.608e-23 | 0 | 6.796e-08 | -9.79 |
| Second replanning latency (s) | 20 | 0.1736 | 0.009508 | 20 | 0.08534 | 0.0076 | -0.08826 | -0.09378 | -0.08274 | -32.43 | 2.275e-28 | 9.098e-28 | 0 | 6.796e-08 | -10.05 |
| Total planning-service time (s) | 20 | 0.5413 | 0.03118 | 20 | 0.8369 | 0.06364 | 0.2956 | 0.2631 | 0.3281 | 18.65 | 3.456e-17 | 3.456e-17 | 400 | 6.796e-08 | 5.781 |

## Descriptive Mission-Level Outcomes
| Metric | Native n | Native mean | Native median | Native SD | Native 95% CI low | Native 95% CI high | UPF4ROS2 n | UPF4ROS2 mean | UPF4ROS2 median | UPF4ROS2 SD | UPF4ROS2 95% CI low | UPF4ROS2 95% CI high | Note |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Total mission time (s) | 20 | 153.4 | 148.5 | 15.64 | 146.1 | 160.7 | 20 | 140.6 | 138.5 | 20.88 | 130.9 | 150.4 | Descriptive only; mission duration includes execution-level factors and is not interpreted as a direct UPF4ROS2 effect. |
| Mission success | 20 | 1 | 1 | 0 | 1 | 1 | 20 | 1 | 1 | 0 | 1 | 1 | Descriptive only; no inferential tests reported for constant outcomes. |
| Number of replans | 20 | 2 | 2 | 0 | 2 | 2 | 20 | 2 | 2 | 0 | 2 | 2 | Descriptive only; no inferential tests reported for constant outcomes. |
| Initial plan length | 20 | 4 | 4 | 0 | 4 | 4 | 20 | 4 | 4 | 0 | 4 | 4 | Descriptive only; no inferential tests reported for constant outcomes. |
| First replanning plan length | 20 | 3 | 3 | 0 | 3 | 3 | 20 | 3 | 3 | 0 | 3 | 3 | Descriptive only; no inferential tests reported for constant outcomes. |
| Second replanning plan length | 20 | 1 | 1 | 0 | 1 | 1 | 20 | 1 | 1 | 0 | 1 | 1 | Descriptive only; no inferential tests reported for constant outcomes. |

Note: Holm correction was applied only across the four planning-time metrics. Mean differences are UPF4ROS2 minus native. Mission time and mission-level outcomes are descriptive only.
