---
id: 1
title: Physical AI Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: master
user: ruser
command: /sp.constitution
labels: ["constitution", "spec-driven", "ai-textbook"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
Project: Physical AI & Humanoid Robotics – AI-Native Textbook (Project 1)
(AI/Spec-Driven Book Creation using Docusaurus, GitHub Pages or Vercel, and Spec-Kit Plus)

Core principles:

* Spec-driven authoring: No chapter written without a prior approved specification
* AI-augmented but human-verified content (Claude Code + human-in-the-loop review)
* Ground truth anchored in official ROS 2, NVIDIA Isaac, Gazebo, and Unity documentation
* Progressive pedagogy: beginner → intermediate → capstone autonomy
* Open educational resource (OER) compliant: reusable, remixable, redistributable
* Automation-first publishing: CI/CD-ready Docusaurus deployment to GitHub Pages or Vercel

Key standards:

* Every chapter must stem from a `/sp.specify`-approved design
* All technical assertions validated against trusted sources (ROS docs, Isaac SDK, etc.)
* Code snippets must be executable in a standard Ubuntu 22.04 + ROS 2 Humble environment
* Markdown-only content with Docusaurus frontmatter (title, description, slug, sidebar)
* Repository must include:
  - Clean branch strategy (main + feature branches)
  - Semantic commit messages
  - Public GitHub Issues for community feedback
* Writing must follow: Concept → Visual → Code → Lab → Reflection
* AI output must be edited for clarity, safety, and pedagogical effectiveness

Constraints:

* Output format: Static site via Docusaurus v3+
* Hosting: GitHub Pages or Vercel only (no Vercel/Netlify for base submission)
* Source control: Public GitHub repository
* AI stack: Spec-Kit Plus + Claude Code only (no external LLM APIs in draft phase)
* Content pipeline: `/sp.specify` → AI draft → human edit → test → GitHub Pages or Vercel deploy
* All content must be original, non-infringing, and free of hallucinated APIs or libraries
* No prose generated without explicit spec alignment
* SEO-optimized headings, meta descriptions, and internal linking required

Success criteria:

* Live book deployed at `<username>.github.io/<repo>`
* Zero broken links or image 404s
* All labs verified in simulation (Gazebo/Isaac Sim)
* Docusaurus builds cleanly with no warnings in production mode
* Repository serves as a template for future Panaversity AI-native books
* Learner can complete the capstone project using only this book
```

## Response snapshot

Updated the constitution file with the specified principles and constraints for the Physical AI & Humanoid Robotics textbook project.

## Outcome

- ✅ Impact: Updated constitution with project-specific principles and constraints
- 🧪 Tests: none
- 📁 Files: 1 file updated (.specify/memory/constitution.md)
- 🔁 Next prompts: /sp.specify to create feature specifications
- 🧠 Reflection: Constitution now properly reflects the AI textbook project requirements

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating feature specifications using /sp.specify