# GenPiH: A General Peg-in-Hole Assembly Policy Based on Domain Randomized Reinforcement Learning

**Xinyu Liu**, **Aljaž Kramberger**, **Leon Bodenhagen**  
*The Maersk Mc-Kinney Moller Institute, University of Southern Denmark, Odense, Denmark*  
📍 *Oral Presentation at the 34th International Conference on Robotics in Alpe-Adria-Danube Region (RAAD 2025)*

---

## 🧩 Abstract
Generalization is important for peg-in-hole assembly, a fundamental industrial operation, to adapt to dynamic industrial scenarios and enhance manufacturing efficiency. While prior work has enhanced generalization ability for pose variations, spatial generalization to six degrees of freedom (6-DOF) is less researched, limiting application in real-world scenarios. This paper addresses this limitation by developing a general policy GenPiH using Proximal Policy Optimization(PPO) and dynamic simulation with domain randomization. The policy learning experiment demonstrates the policy's generalization ability with nearly 100\% success insertion across over eight thousand unique hole poses in parallel environments, and sim-to-real validation on a UR10e robot confirms the policy’s performance through direct trajectory execution without task-specific tuning.

🧠 GenPiH Training Pipeline:
![GenPiH Training Pipeline](https://github.com/user-attachments/assets/afec79b5-856f-40a1-b944-4241a4c056a0)

---

## 🔄 Hole Pose Variation

| Variable | Range               |
|----------|---------------------|
| **X**    | \[-0.20, 0.20\] m   |
| **Y**    | \[-0.26, 0.26\] m   |
| **Z**    | \[0.00, 0.16\] m    |
| **RPY**  | \[-25°, 25°\]       |

![simulation_scenario](https://github.com/user-attachments/assets/6c2b165f-0cb2-4f29-bd0c-d27510ede56c)

---

## 📈 Training Performance

https://github.com/user-attachments/assets/e6193d95-75fe-4ef3-a828-b49235480bcf

![training performance](https://github.com/user-attachments/assets/1cac5868-0c91-4885-b4bd-e72ddb1efa42)

*Fast convergence and stable reward evolution over multiple parallel environments.*

---

## 🤖 Sim-to-Real Transfer

**Real-world Execution Pose:**  
**XYZ:** [0.131, -0.703, 0.198] m  
**RPY:** [0°, 0°, 25°]

![sim2real](https://github.com/user-attachments/assets/1ba56528-c9b1-49d8-b478-9257c4e5b645)

> GenPiH performs insertion directly on a real robot without manual retuning.

---

## 📄 Citation[waiting...]

