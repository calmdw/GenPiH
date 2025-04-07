# GenPiH
# GenPiH: A General Peg-in-Hole Assembly Policy Based on Domain Randomized Reinforcement Learning
Xinyu Liu, Aljaz Kramberger, Leon Bodenhagen

The Maersk Mc-Kinney Moller Institute, University of Southern Denmark, Odense, Denmark

34th International Conference on Robotics in Alpe-Adria-Danube Region(oral presentation)

Abstract:
Generalization is important for peg-in-hole assembly, a fundamental industrial operation, to adapt to dynamic industrial scenarios and enhance manufacturing efficiency. While prior work has enhanced generalization ability for pose variations, spatial generalization to six degrees of freedom (6-DOF) is less researched, limiting application in real-world scenarios. This paper addresses this limitation by developing a general policy GenPiH using Proximal Policy Optimization(PPO) and dynamic simulation with domain randomization. The policy learning experiment demonstrates the policy's generalization ability with nearly 100\% success insertion across over eight thousand unique hole poses in parallel environments, and sim-to-real validation on a UR10e robot confirms the policy’s performance through direct trajectory execution without task-specific tuning.

GenPiH Training Pipeline:
![GenPiH Training Pipeline](https://github.com/user-attachments/assets/afec79b5-856f-40a1-b944-4241a4c056a0)

---

## Pose Variation:

\begin{table}
\centering
\caption{\centering Hole Pose Range}
\setlength{\tabcolsep}{6.6mm}%{6.6mm}{% Change this value to adjust the width of the table.
\begin{tabular}{cc}
    \hline \rule{0pt}{8pt}% Tables with three horizontal lines are recommended.
    Variables & Range \\ %& Direction\\
    \hline
    X  & $[-0.2, 0.2]m$ \\
    Y  & $[-0.26, 0.26]m$\\
    Z   & $[0.0, 0.16]m$\\
    RPY   & $[-25, 25][deg]$\\
    \hline
\end{tabular}
\label{tab: Hole Pose Range}%
\end{table}%

![simulation_scenario](https://github.com/user-attachments/assets/6c2b165f-0cb2-4f29-bd0c-d27510ede56c)


## Training Performance:

![training performance](https://github.com/user-attachments/assets/1cac5868-0c91-4885-b4bd-e72ddb1efa42)


## Sim2Real:

![sim2real](https://github.com/user-attachments/assets/1ba56528-c9b1-49d8-b478-9257c4e5b645)

https://github.com/user-attachments/assets/63d0ae99-2fb2-4d64-bac9-11c1ba8340d3

https://github.com/user-attachments/assets/d2026395-65d9-4330-9eb1-b47981118b7e


