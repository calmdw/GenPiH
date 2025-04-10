# GenPiH: A General Peg-in-Hole Assembly Policy Based on Domain Randomized Reinforcement Learning

**Xinyu Liu**, **Aljaž Kramberger**, **Leon Bodenhagen**  
*The Maersk Mc-Kinney Moller Institute, University of Southern Denmark, Odense, Denmark*  
📍 ***Accepted Oral Presentation**

---

## 🧩 Features

1. Generalization ability to various 6-DOF hole poses.
2. 1mm tolerance assembly with Cranfield assembly benchmark.
3. Efficient training and nearly 100% success rate.

GenPiH Training Pipeline:
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

*Fast convergence and stable performance over parallel environments.*

---

## 🤖 Sim-to-Real Transfer

**Real-world Execution Pose:**  
**XYZ:** [0.131, -0.703, 0.198] m  
**RPY:** [0°, 0°, 25°]

![sim2real](https://github.com/user-attachments/assets/1ba56528-c9b1-49d8-b478-9257c4e5b645)

> GenPiH performs insertion directly on a real robot without manual retuning.

---

## 📄 Citation[waiting...]

