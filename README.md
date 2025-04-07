# GenPiH
# GenPiH: A General Peg-in-Hole Assembly Policy Based on Domain Randomized Reinforcement Learning

[![Paper_will be released soon](https://img.shields.io/badge/RAAD%202025-Paper-blue)]

GenPiH is a learning-based framework for solving challenging **peg-in-hole (PiH) insertion tasks** with **diverse hole orientations** and **tight tolerances**, including as small as **0.5mm**. It combines *reinforcement learning* with *traditional force control* to achieve robust and adaptive insertion in both simulation and real-world environments.

![GenPiH Overview](link_to_overview_image.png)

---

## 🧠 Key Features

- **Hybrid Control Strategy**: Combines learned RL-based policy with a traditional force controller for precise insertion.
- **Generalization**: Trained policy can handle arbitrary orientations of the hole (up to ±90° around X and Y axes).
- **High Precision**: Achieves successful insertions with tolerances as tight as 0.5mm.
- **Two-Stage Policy**: First stage performs rapid coarse alignment; second stage handles contact-rich, precise insertion using force feedback.
- **Transferable**: Successfully deployed from simulation (Isaac Sim) to a real robot setup.

---

## 🛠️ System Architecture

