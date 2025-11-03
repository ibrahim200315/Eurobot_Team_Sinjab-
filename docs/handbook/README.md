# Sinjab Project Handbook v0.1 🐿️

> _Eurobot 2026 — Team Sinjab (Squirrel Spirit, Engineering Discipline)_

---

## 1️⃣ Purpose
This handbook defines how the Sinjab team works, collaborates, and delivers updates for the Eurobot 2026 challenge.  
It ensures that any teammate or next-year student can quickly understand **how to contribute, test, and review work**.

---

## 2️⃣ Team Philosophy
**Sinjab (سنجاب)** means **squirrel** — a small but clever creature.  
Our goal is to emulate that spirit: **agile, curious, and prepared**.  
We work with precision, respect, and documentation so our future teammates can build on our foundations.

---

## 3️⃣ Repository Structure
Eurobot_Team_Sinjab-/
   ┣ sinjab_ws/ → ROS 2 workspace (simulation-first)
   ┣ docs/ → Design, requirements, handbook, rules
   ┣ .github/ → Issue/PR templates, workflows
      ┗ README.md

---

## 4️⃣ Roles and Responsibilities
| Role | Example Tasks |
|------|----------------|
| **Programming** | ROS 2 code, Nav2 configs, ArUco detection, scoring logic |
| **Mechanical** | CAD updates, envelope checks, mount designs |
| **Documentation** | Maintaining `/docs/`, updating issues/milestones, reports |
| **Project Manager** | Syncing with teacher, updating Probants and GitHub milestones |

All members are expected to:
- Read and follow this handbook.  
- Communicate progress weekly on GitHub.  
- Leave the repo cleaner than you found it.

---

## 5️⃣ Workflow (GitHub Flow)
1. **Create a branch** from `main`:
   - `feat/short-description` or `fix/short-description`
2. **Work locally**, commit small and descriptive changes.
3. **Open a Pull Request** (PR) → link it to its issue (`Fixes #12`).
4. **Request review** from at least one teammate.
5. CI (tests, linter) must pass before merging.
6. Merge via “Squash & merge” → keep history clean.

> 🚫 Never push directly to `main`.

---

## 6️⃣ Issues & Labels
- Every task must have a **GitHub Issue** (and be linked to Probants if needed).
- Label conventions:
  - `type/*` (feature, bug, docs, research, chore)
  - `area/*` (nav, vision, sim, cad, gameplay, sima)
  - `P0-now`, `P1-soon`, `P2-later`
  - `risk/safety`, `risk/schedule`, `risk/unknowns`

---

## 7️⃣ Definitions of Ready / Done

| Status | Description |
|---------|--------------|
| **Ready** | Clear goal, acceptance criteria, owner assigned, no blockers. |
| **In Progress** | Actively being developed on a branch. |
| **Done** | PR merged, CI passed, demo evidence attached (video/rosbag), documentation updated. |

---

## 8️⃣ Documentation & Evidence

Each deliverable must include:
- **Demo evidence** (simulation video, rosbag, screenshot, or test log).  
- **Documentation** in `/docs/` or comments inside the issue.  
- **Reference to rules** if relevant.

Teachers and reviewers should always be able to:
1. Open an issue → see what was done.
2. Follow the link → watch the evidence.

---

## 9️⃣ Meeting Rhythm

| Type | Frequency | Owner |
|------|------------|-------|
| **Weekly Sync (roughly 1 hour)** | Every week | Project lead |
| **Milestone Review** | At each milestone end | Entire team |
| **Probants Update** | Weekly | Project manager |

---

## 🔟 Communication
- Main async channel: **GitHub Issues / PR comments**
- Optional: internal chat (Teams/WhatsApp) for coordination.
- Decisions with long-term effects → recorded in `/docs/adr/`.

---

## 💡 Core Values
- **Respect:** everyone’s time and code.
- **Transparency:** all work tracked in Issues.
- **Documentation:** write once, help forever.
- **Iteration:** deliver small, test fast, improve steadily.

---

**Next step:**  
Use this handbook to onboard new teammates and to justify your workflow during your teacher presentation.
