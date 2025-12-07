<!--
Sync Impact Report:
Version change: None -> 0.1.0
Modified principles: None
Added sections: Project Scope & Requirements, Constraints & Goals, Modality
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
- .specify/templates/commands/*.md: ✅ updated
- README.md: ⚠ pending
Follow-up TODOs: None
-->
# AI/Spec-Driven Book Creation — Physical AI & Humanoid Robotics Constitution

## Core Principles

### Clarity
All specifications, plans, tasks, and code must be clear and unambiguous, leaving no room for misinterpretation.

### Precision
Exactness in specifications, acceptance criteria, and implementation is paramount.

### Reusability
Design and implement components and solutions that can be reused across different parts of the project or future projects.

### Determinism
All processes, tests, and outputs must be predictable and repeatable, ensuring consistent results.

### Zero Ambiguity
Tasks and acceptance criteria must be defined with absolute clarity, ensuring that their completion can be objectively verified without any uncertainty.

### Modality
All outputs must be in markdown format. Code snippets must be fully copy-paste ready for direct use.

## Project Scope & Requirements

### Mission Statement
Create a multi-chapter educational book using Docusaurus (TypeScript) and deploy it to GitHub Pages. All work is Spec-Driven and produced through iterative cycles using Spec-Kit Plus.

### Tech Stack
- Docusaurus (TypeScript)
- Node.js + npm
- GitHub Pages Deployment
- Gemini CLI as the agentic execution environment

### Primary Domain
Physical AI, Humanoid Robotics, ROS 2, Gazebo, Unity, NVIDIA Isaac, Vision-Language-Action Robotics.

### SCOPE — ITERATION 1
- Frontend-only:
    - Docusaurus project initialization
    - Home landing page
    - Use Tailwind CSS for styling
    - Book structure with chapters & submodules
    - Written content for all pages
    - No backend, no automation scripts

### CONTENT REQUIREMENTS
- Include 6 Chapters:
    1. Introduction to Physical AI
    2. ROS 2: The Robotic Nervous System
    3. Simulation: Gazebo, Unity & Digital Twins
    4. NVIDIA Isaac & AI-Powered Robotics
    5. Vision-Language-Action (VLA)
    6. Capstone: The Autonomous Humanoid
- Each chapter must contain 3–5 submodules.
- The home page must explain the course details, themes, modules, outcomes, and hardware requirements.

## Constraints & Goals

### CONSTRAINTS
- Must remain fully TypeScript compliant.
- Everything must be deployable to GitHub Pages.
- No backend or dynamic API usage.
- All assets must be local or Docusaurus defaults.

### GOALS
- Produce:
    - Specification (/sp.specify)
    - Plan (/sp.plan)
    - Tasks (/sp.tasks)
    - Implementation (/sp.implementation)

### FOLLOW-UP
- Wait for user to call /sp.specify.

## Governance
This constitution serves as the foundational document for project development. Amendments require review and approval by the project architect. All changes to the constitution must follow semantic versioning rules. Compliance with these principles will be reviewed as needed, particularly before major releases or significant milestones.

**Version**: 0.1.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07