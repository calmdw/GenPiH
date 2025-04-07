# GenPiH
# GenPiH: A General Peg-in-Hole Assembly Policy Based on Domain Randomized Reinforcement Learning
Xinyu Liu, Aljaz Kramberger, Leon Bodenhagen

The Maersk Mc-Kinney Moller Institute, University of Southern Denmark, Odense, Denmark

34th International Conference on Robotics in Alpe-Adria-Danube Region
[![Paper_will be released soon](https://img.shields.io/badge/oral presentation-Paper-blue)]

Abstract:
Generalization is important for peg-in-hole assembly, a fundamental industrial operation, to adapt to dynamic industrial scenarios and enhance manufacturing efficiency. While prior work has enhanced generalization ability for pose variations, spatial generalization to six degrees of freedom (6-DOF) is less researched, limiting application in real-world scenarios. This paper addresses this limitation by developing a general policy GenPiH using Proximal Policy Optimization(PPO) and dynamic simulation with domain randomization. The policy learning experiment demonstrates the policy's generalization ability with nearly 100\% success insertion across over eight thousand unique hole poses in parallel environments, and sim-to-real validation on a UR10e robot confirms the policy’s performance through direct trajectory execution without task-specific tuning.

GenPiH Training Pipeline:
![GenPiH Training Pipeline](https://github.com/user-attachments/assets/afec79b5-856f-40a1-b944-4241a4c056a0)

---

## 🧠 Key Features

- **Hybrid Control Strategy**: Combines learned RL-based policy with a traditional force controller for precise insertion.
- **Generalization**: Trained policy can handle arbitrary orientations of the hole (up to ±90° around X and Y axes).
- **High Precision**: Achieves successful insertions with tolerances as tight as 0.5mm.
- **Two-Stage Policy**: First stage performs rapid coarse alignment; second stage handles contact-rich, precise insertion using force feedback.
- **Transferable**: Successfully deployed from simulation (Isaac Sim) to a real robot setup.

---

## 🛠️ System Architecture

